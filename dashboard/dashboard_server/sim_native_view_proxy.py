"""Optional SIM GET byte relay. No server, ROS, process or map owner at import."""
import contextlib
import dataclasses
import json
import re
import socket
import threading
import time

PREFIX = '/api/sim/native-map'
SOCKET_PATH = '/run/ubts-sim/native-view.sock'
MAX_CLIENTS = 2
MAX_BODY_BYTES = 32 * 1024 * 1024
MAX_JSON_BYTES = 1024 * 1024
TOTAL_SECONDS = 5.0
BODY_PATH = re.compile(re.escape(PREFIX) + r'/body/[0-9a-f]{32}/[1-9][0-9]{0,15}/[1-9][0-9]{0,15}/[0-9a-f]{64}')
FORWARD_HEADERS = {'content-type', 'content-length', 'content-encoding', 'etag', 'vary', 'cache-control', 'x-content-type-options', 'allow'}


def require(value, reason):
    if not value:
        raise ValueError(reason)


@dataclasses.dataclass(frozen=True)
class Response:
    status: int
    headers: tuple
    body: bytes
    deadline: float


def error_response(status, reason, deadline):
    body = json.dumps(dict(ok=False, error=reason), separators=(',', ':')).encode()
    headers = [('Content-Type', 'application/json; charset=utf-8'), ('Content-Length', str(len(body))),
               ('Cache-Control', 'no-store'), ('X-Content-Type-Options', 'nosniff')]
    if status == 405:
        headers.append(('Allow', 'GET'))
    return Response(status, tuple(headers), body, deadline)


class _Reader:
    def __init__(self, connection, deadline, clock):
        self.connection, self.deadline, self.clock = connection, deadline, clock
        self.buffer = bytearray()

    def receive(self, limit):
        remaining = self.deadline - self.clock()
        require(remaining > 0, 'native_view_transfer_deadline')
        self.connection.settimeout(remaining)
        value = self.connection.recv(limit)
        require(bool(value), 'native_view_response_truncated')
        self.buffer.extend(value)

    def line(self, cap=2048):
        while True:
            end = self.buffer.find(b'\r\n')
            if end >= 0:
                require(end <= cap, 'native_view_header_line_cap')
                value = bytes(self.buffer[:end])
                del self.buffer[:end+2]
                return value
            require(len(self.buffer) <= cap, 'native_view_header_line_cap')
            self.receive(min(4096, cap + 2 - len(self.buffer)))

    def body(self, length):
        result = bytearray()
        while len(result) < length:
            if not self.buffer:
                self.receive(min(65536, length - len(result)))
            count = min(length - len(result), len(self.buffer))
            result.extend(self.buffer[:count])
            del self.buffer[:count]
        require(not self.buffer, 'native_view_buffered_extra_response')
        require(self.clock() < self.deadline, 'native_view_transfer_deadline')
        return bytes(result)


def _connect(path, timeout):
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        connection.settimeout(timeout)
        connection.connect(path)
        return connection
    except BaseException:
        connection.close()
        raise


class NativeViewProxy:
    def __init__(self, *, enabled=False, socket_factory=None, clock=time.monotonic):
        require(type(enabled) is bool, 'typed_native_view_enabled')
        self.enabled, self._factory, self._clock = enabled, socket_factory or _connect, clock
        self._slots = threading.BoundedSemaphore(MAX_CLIENTS)
        self._lock, self._active = threading.Lock(), 0

    @property
    def active_requests(self):
        with self._lock:
            return self._active

    def _fetch(self, path, headers, deadline):
        require(type(path) is str and len(path) <= 256 and ('?' not in path), 'native_view_exact_path')
        is_body = BODY_PATH.fullmatch(path) is not None
        require(path in (PREFIX+'/status', PREFIX+'/snapshot') or is_body, 'native_view_exact_path')
        wanted = []
        for key in ('Accept-Encoding', 'If-None-Match'):
            value = headers.get(key, headers.get(key.lower(), ''))
            if value:
                require(type(value) is str and len(value) <= 512 and all(32 <= ord(c) < 127 for c in value), 'native_view_request_header')
                wanted.append(key+': '+value+'\r\n')
        remaining = deadline - self._clock()
        require(remaining > 0, 'native_view_transfer_deadline')
        connection = self._factory(SOCKET_PATH, remaining)
        try:
            request = ('GET '+path+' HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n'+''.join(wanted)+'\r\n').encode('ascii')
            remaining = deadline-self._clock()
            require(remaining > 0, 'native_view_transfer_deadline')
            connection.settimeout(remaining)
            connection.sendall(request)
            reader = _Reader(connection, deadline, self._clock)
            first = reader.line()
            match = re.fullmatch(br'HTTP/1\.[01] ([0-9]{3})(?: [\x20-\x7e]*)?', first)
            require(match is not None, 'native_view_HTTP_status')
            status = int(match[1])
            require(status in (200, 304, 400, 404, 405, 410, 429, 500, 503), 'native_view_HTTP_status')
            total, values, output = len(first)+2, {}, []
            while True:
                line = reader.line(); total += len(line)+2
                require(total <= 8192, 'native_view_response_headers_cap')
                if not line:
                    break
                require(b':' in line and all(32 <= n < 127 for n in line), 'native_view_response_header')
                key, value = line.split(b':', 1)
                require(re.fullmatch(br'[A-Za-z0-9-]+', key) is not None, 'native_view_response_header')
                lower, text = key.decode().lower(), value.strip().decode('ascii')
                require(lower not in values, 'native_view_duplicate_header')
                require(lower in FORWARD_HEADERS | {'connection', 'date', 'server'}, 'native_view_unexpected_header')
                values[lower] = text
                if lower in FORWARD_HEADERS:
                    output.append((key.decode('ascii'), text))
            require(values.get('content-encoding') in (None, 'gzip'), 'native_view_response_encoding')
            require(values.get('content-type') in ('application/json; charset=utf-8', 'application/octet-stream'), 'native_view_response_type')
            length = values.get('content-length')
            if status == 304:
                require(length in (None, '0'), 'native_view_304_body')
                body = reader.body(0)
            else:
                require(length is not None and re.fullmatch('[0-9]{1,9}', length) is not None, 'native_view_response_length')
                length = int(length)
                require(length <= (MAX_BODY_BYTES if is_body else MAX_JSON_BYTES), 'native_view_response_cap')
                body = reader.body(length)
            return Response(status, tuple(output), body, deadline)
        finally:
            connection.close()

    @contextlib.contextmanager
    def response(self, method, path, headers=None):
        """Hold the client permit through caller's bounded response write."""
        deadline = self._clock()+TOTAL_SECONDS
        if not self.enabled:
            yield error_response(404, 'native_view_disabled', deadline)
            return
        if method != 'GET':
            yield error_response(405, 'native_view_GET_only', deadline)
            return
        if type(path) is not str or len(path) > 256 or '?' in path:
            yield error_response(400, 'native_view_exact_path', deadline)
            return
        if path not in (PREFIX+'/status', PREFIX+'/snapshot') and BODY_PATH.fullmatch(path) is None:
            yield error_response(404, 'native_view_route_not_found', deadline)
            return
        if not self._slots.acquire(blocking=False):
            yield error_response(503, 'native_view_client_limit', deadline)
            return
        with self._lock:
            self._active += 1
        try:
            try:
                reply = self._fetch(path, headers or {}, deadline)
            except ValueError as exc:
                reply = error_response(502, str(exc), deadline)
            except (OSError, TimeoutError):
                reply = error_response(503, 'native_view_service_unavailable', deadline)
            yield reply
        finally:
            with self._lock:
                self._active -= 1
            self._slots.release()

"""Bounded opaque sensor-WebSocket transport, without decoding or dropping frames.

Only the authenticated camera/LiDAR/IMU/GPS upgrade path uses this relay. It
never interprets control messages and has no bridge or device discovery code.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import select
import time


MAX_PENDING_BYTES = 65536  # Per direction, independent of source frame size.
WRITE_STALL_S = 8.0
SELECT_WAKE_S = 30.0


@dataclass(frozen=True)
class RelayResult:
    reason: str
    to_client_bytes: int
    to_upstream_bytes: int
    peak_to_client_bytes: int
    peak_to_upstream_bytes: int


def relay_sensor_stream(client, upstream, initial_upstream=b"", *,
                        session_deadline=None,
                        clock=time.monotonic, wall_clock=time.time,
                        select_ready=select.select, write_stall_s=WRITE_STALL_S):
    """Relay partial writes in both directions with a fixed memory/stall bound.

    Reading pauses when that direction's 64 KiB buffer is full. The opposite
    direction can still deliver browser ACKs. Byte order is unchanged: dropping
    arbitrary bytes would corrupt a WebSocket frame, so an unresponsive viewer
    is disconnected instead. Other viewers use separate buffers and threads.
    Socket ownership and closing remain with DashboardHandler.
    """
    # session_deadline is a wall-clock time, or a function giving the current one
    # (NET-11: a session that is still used is extended, and its streams with it).
    deadline_of = session_deadline if callable(session_deadline) else None
    if (type(initial_upstream) is not bytes or len(initial_upstream) > MAX_PENDING_BYTES
            or type(write_stall_s) not in (int, float)
            or not math.isfinite(write_stall_s) or write_stall_s <= 0
            or (session_deadline is not None and deadline_of is None and
                (type(session_deadline) not in (int, float) or not math.isfinite(session_deadline)))):
        raise ValueError("invalid_bounded_sensor_relay_input")
    now = clock()
    # Rows: source, destination, pending bytes, last successful write/queue time.
    rows = [[upstream, client, bytearray(initial_upstream), now],
            [client, upstream, bytearray(), now]]
    written = [0, 0]
    peaks = [len(initial_upstream), 0]
    timeouts = [client.gettimeout(), upstream.gettimeout()]
    reason = "io_error"
    eof = False
    try:
        client.setblocking(False)
        upstream.setblocking(False)
        while True:
            now = clock()
            current_deadline = deadline_of() if deadline_of is not None else session_deadline
            remaining = None if current_deadline is None else current_deadline - wall_clock()
            if remaining is not None and remaining <= 0:
                reason = "session_expired"
                # No injected status/close bytes: an empty TCP buffer is not
                # proof of a WebSocket frame boundary. The owner closes TCP.
                break
            if any(row[2] and now - row[3] >= write_stall_s for row in rows):
                reason = "write_stalled"
                break
            if eof and not any(row[2] for row in rows):
                reason = "peer_closed"
                break
            readers = [] if eof else [row[0] for row in rows if len(row[2]) < MAX_PENDING_BYTES]
            writers = [row[1] for row in rows if row[2]]
            wait = SELECT_WAKE_S
            if remaining is not None:
                wait = min(wait, max(0.0, remaining))
            for row in rows:
                if row[2]:
                    wait = min(wait, max(0.0, write_stall_s - (now - row[3])))
            readable, writable, errored = select_ready(readers, writers, [client, upstream], wait)
            if errored:
                break
            # Flush without blocking, then give both input directions a turn.
            for index, row in enumerate(rows):
                if row[2] and row[1] in writable:
                    try:
                        count = row[1].send(row[2])
                    except (BlockingIOError, InterruptedError):
                        continue
                    if count <= 0:
                        return RelayResult("peer_closed", *written, *peaks)
                    del row[2][:count]
                    written[index] += count
                    row[3] = clock()
            for index, row in enumerate(rows):
                if not eof and row[0] in readable:
                    try:
                        chunk = row[0].recv(MAX_PENDING_BYTES - len(row[2]))
                    except (BlockingIOError, InterruptedError):
                        continue
                    if not chunk:
                        eof = True
                        break
                    if not row[2]:
                        row[3] = clock()
                    row[2].extend(chunk)
                    peaks[index] = max(peaks[index], len(row[2]))
    except OSError:
        reason = "io_error"
    finally:
        # Restore per-socket settings for the owning handler's existing cleanup.
        for sock, timeout in zip((client, upstream), timeouts):
            try:
                sock.settimeout(timeout)
            except OSError:
                pass
    return RelayResult(reason, *written, *peaks)

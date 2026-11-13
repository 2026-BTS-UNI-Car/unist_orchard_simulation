"""Content-hash addresses and long caching for the dashboard's static files (NET-04).

Every static file gets a version: the first characters of the SHA-256 of the body the server sends. The page and
the files it loads name their dependencies with that version (``/static/app/02-shell.js?v=3fa9c01b2e``):

- index.html: ``src="/static/..."`` and ``href="/static/..."``;
- JavaScript modules: relative ``import ... from './x.mjs'``, ``export ... from``, ``import './x.mjs'`` and
  ``import('./x.mjs')`` (``.mjs`` files and the page's own ``.js`` files; classic vendor scripts are not rewritten);
- CSS: ``url(/static/...)`` and relative ``url(...)``;
- the page's own scripts (not vendor/): a whole string literal naming an existing file, ``"/static/vendor/x.js"``
  (a script loaded when it is first needed). Template strings and missing files are left alone.

A request whose ``v`` matches the current version is answered with a one-year immutable cache, so a reload asks the
server only for index.html. Any other request (no ``v``, or an old one) gets the current body with ``no-cache`` and an
ETag, so it is checked again next time. Because a file's body includes the versions of what it imports, changing a
module changes the version of every file that imports it, up to index.html.

Bodies, versions and gzip results are cached in memory, keyed by the file's mtime and size, so the server does not
hash or compress a file again on every request (the gzip cache is bounded). Files are read again only when they change.
"""
from __future__ import annotations

import gzip
import hashlib
import posixpath
import re
import threading
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

LONG_CACHE = "private, max-age=31536000, immutable"
REVALIDATE = "private, no-cache"
VERSION_LENGTH = 10
GZIP_SUFFIXES = (".js", ".mjs", ".css", ".html", ".json", ".svg")
GZIP_CACHE_BYTES = 8 * 1024 * 1024
KEEP_BODY_BYTES = 4 * 1024 * 1024  # larger files are hashed but read again when sent

_HTML_REF = re.compile(r'(\b(?:src|href)=")/static/([^"?#]+)(")')
_JS_REF = re.compile(r'''(\bfrom\s*|\bimport\s*\(\s*|\bimport\s+)(["'])(\.{1,2}/[^"'?#\s]+)\2''')
_CSS_REF = re.compile(r'''(url\(\s*)(["']?)(/static/[^"')?#\s]+|\.{1,2}/[^"')?#\s]+)\2(\s*\))''')
# A whole string literal naming a static file in the page's own scripts, e.g. a script loaded when first needed.
_JS_STATIC_LITERAL = re.compile(r'''(["'])/static/([^"'?#\s$`{}]+)\1''')


@dataclass(frozen=True)
class Served:
    """What to send for one static request."""
    path: Path
    body: bytes
    version: str
    etag: str
    cache_control: str
    gzip: bool


@dataclass
class _Entry:
    stat: tuple
    deps: tuple  # ((relative path, version used), ...)
    body: bytes | None
    version: str


class StaticAssets:
    def __init__(self, root: Path, gzip_cache_bytes: int = GZIP_CACHE_BYTES):
        self.root = Path(root).resolve()
        self.gzip_cache_bytes = gzip_cache_bytes
        self._lock = threading.RLock()
        self._entries: dict[str, _Entry] = {}
        self._gzip: OrderedDict[tuple, bytes] = OrderedDict()
        self._gzip_bytes = 0
        self.stats = {"hashed": 0, "gzip_made": 0, "gzip_reused": 0}
        self._page_filter = None

    def set_page_filter(self, page_filter) -> None:
        """A function that changes the text of the page (index.html) before it is versioned, or None (D-12:
        simulation mode and the vehicle kind). Cached bodies are dropped so the next request uses it."""
        with self._lock:
            if page_filter is self._page_filter:
                return
            self._page_filter = page_filter
            self._entries.clear()
            self._gzip.clear()
            self._gzip_bytes = 0

    # --- paths -------------------------------------------------------------------------------------------------
    def resolve(self, rel: str) -> Path | None:
        """The file for a path below /static/, or None if it is outside the root or not a file."""
        rel = rel.lstrip("/")
        if not rel or "\x00" in rel:
            return None
        try:
            path = (self.root / rel).resolve()
        except (OSError, ValueError):
            return None
        if self.root not in path.parents or not path.is_file():
            return None
        return path

    def _relative(self, path: Path) -> str:
        return path.relative_to(self.root).as_posix()

    # --- versions ----------------------------------------------------------------------------------------------
    def version(self, rel: str) -> str | None:
        entry = self._entry(rel, ())
        return entry.version if entry else None

    def _entry(self, rel: str, stack: tuple) -> _Entry | None:
        path = self.resolve(rel)
        if path is None:
            return None
        rel = self._relative(path)
        try:
            info = path.stat()
        except OSError:
            return None
        stat = (info.st_mtime_ns, info.st_size)
        with self._lock:
            cached = self._entries.get(rel)
        if cached is not None and cached.stat == stat and self._deps_current(cached.deps, stack + (rel,)):
            return cached
        try:
            raw = path.read_bytes()
        except OSError:
            return None
        body, deps = self._rewrite(rel, raw, stack + (rel,))
        version = hashlib.sha256(body).hexdigest()[:VERSION_LENGTH]
        entry = _Entry(stat, deps, body if len(body) <= KEEP_BODY_BYTES else None, version)
        with self._lock:
            self._entries[rel] = entry
            self.stats["hashed"] += 1
        return entry

    def _deps_current(self, deps: tuple, stack: tuple) -> bool:
        for dep, used in deps:
            if dep in stack:
                return False
            entry = self._entry(dep, stack)
            if entry is None or entry.version != used:
                return False
        return True

    # --- rewriting ---------------------------------------------------------------------------------------------
    def _rewrite(self, rel: str, raw: bytes, stack: tuple) -> tuple[bytes, tuple]:
        suffix = posixpath.splitext(rel)[1].lower()
        if suffix == ".html":
            pattern, kind = _HTML_REF, "html"
        elif suffix == ".mjs" or (suffix == ".js" and not rel.startswith("vendor/")):
            pattern, kind = _JS_REF, "js"
        elif suffix == ".css":
            pattern, kind = _CSS_REF, "css"
        else:
            return raw, ()
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            return raw, ()
        if rel == "index.html" and self._page_filter is not None:
            text = self._page_filter(text)
        deps: dict[str, str] = {}
        base = posixpath.dirname(rel)

        def target_of(ref: str) -> str | None:
            if ref.startswith("/static/"):
                target = ref[len("/static/"):]
            else:
                target = posixpath.normpath(posixpath.join(base, ref))
            if target.startswith("../") or target == ".." or target.startswith("/"):
                return None
            return target

        def versioned(target: str | None) -> str | None:
            if target is None or target in stack:
                return None  # outside the root, or an import cycle: leave that reference as it is
            entry = self._entry(target, stack)
            if entry is None:
                return None
            deps[self._relative(self.resolve(target))] = entry.version
            return entry.version

        def html_ref(match):
            version = versioned(target_of("/static/" + match.group(2)))
            if version is None:
                return match.group(0)
            return f"{match.group(1)}/static/{match.group(2)}?v={version}{match.group(3)}"

        def js_ref(match):
            version = versioned(target_of(match.group(3)))
            if version is None:
                return match.group(0)
            quote = match.group(2)
            return f"{match.group(1)}{quote}{match.group(3)}?v={version}{quote}"

        def css_ref(match):
            version = versioned(target_of(match.group(3)))
            if version is None:
                return match.group(0)
            quote = match.group(2)
            return f"{match.group(1)}{quote}{match.group(3)}?v={version}{quote}{match.group(4)}"

        def js_literal(match):
            version = versioned(target_of("/static/" + match.group(2)))
            if version is None:
                return match.group(0)
            quote = match.group(1)
            return f"{quote}/static/{match.group(2)}?v={version}{quote}"

        text = pattern.sub({"html": html_ref, "js": js_ref, "css": css_ref}[kind], text)
        if kind == "js" and not rel.startswith("vendor/"):
            text = _JS_STATIC_LITERAL.sub(js_literal, text)
        return text.encode("utf-8"), tuple(sorted(deps.items()))

    # --- serving -----------------------------------------------------------------------------------------------
    def serve(self, rel: str, requested_version: str | None, accept_gzip: bool) -> Served | None:
        """The body and headers for /static/<rel> (or index.html), or None when there is no such file."""
        path = self.resolve(rel)
        entry = self._entry(rel, ()) if path is not None else None
        if entry is None:
            return None
        body = entry.body
        if body is None:  # a large file: not kept in memory, never rewritten
            try:
                body = path.read_bytes()
            except OSError:
                return None
        use_gzip = accept_gzip and path.suffix.lower() in GZIP_SUFFIXES
        if use_gzip:
            body = self._gzipped(self._relative(path), entry.version, body)
        cache = LONG_CACHE if requested_version and requested_version == entry.version else REVALIDATE
        etag = f'"{entry.version}{"-gz" if use_gzip else ""}"'
        return Served(path, body, entry.version, etag, cache, use_gzip)

    def _gzipped(self, rel: str, version: str, body: bytes) -> bytes:
        key = (rel, version)
        with self._lock:
            cached = self._gzip.get(key)
            if cached is not None:
                self._gzip.move_to_end(key)
                self.stats["gzip_reused"] += 1
                return cached
        compressed = gzip.compress(body, compresslevel=6, mtime=0)
        with self._lock:
            self.stats["gzip_made"] += 1
            if len(compressed) <= self.gzip_cache_bytes // 2 and key not in self._gzip:
                self._gzip[key] = compressed
                self._gzip_bytes += len(compressed)
                while self._gzip_bytes > self.gzip_cache_bytes and self._gzip:
                    _, old = self._gzip.popitem(last=False)
                    self._gzip_bytes -= len(old)
        return compressed

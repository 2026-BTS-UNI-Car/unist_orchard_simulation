"""Small SVG line / scatter plots (standard library + numpy; no matplotlib, so the tool also runs on the Jetson).

figure(path, panels, title, width): panels = [Panel, ...] stacked vertically. A Panel holds series (line, points,
circles), axis labels, a title, an optional equal aspect (1 m on x = 1 m on y: the panel height follows the data) and
fixed limits. Text is UTF-8 (Korean labels are fine).
"""
from __future__ import annotations

import math
from xml.sax.saxutils import escape

import numpy as np

COLOURS = ('#1f5fbf', '#d9661f', '#2e8b57', '#c0392b', '#7d3c98', '#6b6b6b', '#b8860b')
FONT = 'font-family="sans-serif"'


def nice_ticks(lo, hi, count=6):
    if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
        return [lo]
    raw = (hi - lo) / max(count, 1)
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=10 * mag)
    if (hi - lo) / step < 2.:
        step /= 2.
    first = math.ceil(lo / step) * step
    ticks, v = [], first
    while v <= hi + step * 1e-9:
        ticks.append(round(v, 10))
        v += step
    return ticks


def _text_width(text, size=10):
    return int(sum(size * (.6 if ord(ch) < 128 else 1.0) for ch in text))


def _fmt(v):
    if v == 0:
        return '0'
    a = abs(v)
    if a >= 100:
        return '%.0f' % v
    if a >= 10:
        return ('%.1f' % v).rstrip('0').rstrip('.')
    if a >= 1:
        return ('%.2f' % v).rstrip('0').rstrip('.')
    return ('%.3f' % v).rstrip('0').rstrip('.')


class Panel:
    def __init__(self, title='', xlabel='', ylabel='', equal=False, xlim=None, ylim=None, height=220, note=None):
        self.title, self.xlabel, self.ylabel = title, xlabel, ylabel
        self.equal, self.xlim, self.ylim, self.height, self.note = equal, xlim, ylim, height, note
        self.series = []

    def line(self, x, y, label=None, colour=None, width=1.4, dash=None):
        self.series.append(dict(kind='line', x=np.asarray(x, float), y=np.asarray(y, float), label=label,
                                colour=colour, width=width, dash=dash))
        return self

    def points(self, x, y, label=None, colour=None, size=2.2):
        self.series.append(dict(kind='points', x=np.asarray(x, float), y=np.asarray(y, float), label=label,
                                colour=colour, size=size))
        return self

    def circles(self, x, y, r, label=None, colour=None):
        """Circles of data radius r (equal-aspect panels; at least 1.5 px)."""
        self.series.append(dict(kind='circles', x=np.asarray(x, float), y=np.asarray(y, float),
                                r=np.broadcast_to(np.asarray(r, float), np.shape(x)), label=label, colour=colour))
        return self

    def limits(self):
        xs, ys = [], []
        for s in self.series:
            ok = np.isfinite(s['x']) & np.isfinite(s['y'])
            if s['kind'] == 'circles':
                xs += [s['x'][ok] - s['r'][ok], s['x'][ok] + s['r'][ok]]
                ys += [s['y'][ok] - s['r'][ok], s['y'][ok] + s['r'][ok]]
            else:
                xs.append(s['x'][ok])
                ys.append(s['y'][ok])
        xs = np.concatenate(xs) if xs else np.zeros(0)
        ys = np.concatenate(ys) if ys else np.zeros(0)
        x0, x1 = (float(xs.min()), float(xs.max())) if len(xs) else (0., 1.)
        y0, y1 = (float(ys.min()), float(ys.max())) if len(ys) else (0., 1.)
        if x1 - x0 < 1e-9:
            x0, x1 = x0 - .5, x1 + .5
        if y1 - y0 < 1e-9:
            y0, y1 = y0 - .5, y1 + .5
        mx, my = .05 * (x1 - x0), .05 * (y1 - y0)
        x0, x1, y0, y1 = x0 - mx, x1 + mx, y0 - my, y1 + my
        if self.xlim:
            x0, x1 = self.xlim
        if self.ylim:
            y0, y1 = self.ylim
        return x0, x1, y0, y1


def _panel_svg(p, top, width, colours):
    left, right = 64, 16
    w = width - left - right
    x0, x1, y0, y1 = p.limits()
    if p.equal:
        h = max(60., min(900., w * (y1 - y0) / (x1 - x0)))
    else:
        h = float(p.height)
    title_h = 22
    oy = top + title_h
    sx = w / (x1 - x0)
    sy = h / (y1 - y0)

    def X(v):
        return left + (v - x0) * sx

    def Y(v):
        return oy + h - (v - y0) * sy
    out = []
    out.append('<text x="%d" y="%d" %s font-size="13" font-weight="bold">%s</text>'
               % (left, top + 15, FONT, escape(p.title)))
    out.append('<rect x="%d" y="%.1f" width="%.1f" height="%.1f" fill="none" stroke="#999"/>' % (left, oy, w, h))
    out.append('<clipPath id="c%d"><rect x="%d" y="%.1f" width="%.1f" height="%.1f"/></clipPath>' % (top, left, oy, w, h))
    for v in nice_ticks(x0, x1, max(3, int(w / 90))):
        if x0 <= v <= x1:
            out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e3e3e3"/>' % (X(v), oy, X(v), oy + h))
            out.append('<text x="%.1f" y="%.1f" %s font-size="10" text-anchor="middle">%s</text>'
                       % (X(v), oy + h + 13, FONT, _fmt(v)))
    for v in nice_ticks(y0, y1, max(4, int(h / 40))):
        if y0 <= v <= y1:
            out.append('<line x1="%d" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e3e3e3"/>' % (left, Y(v), left + w, Y(v)))
            out.append('<text x="%d" y="%.1f" %s font-size="10" text-anchor="end">%s</text>'
                       % (left - 4, Y(v) + 3, FONT, _fmt(v)))
    out.append('<text x="%.1f" y="%.1f" %s font-size="11" text-anchor="middle">%s</text>'
               % (left + w / 2, oy + h + 28, FONT, escape(p.xlabel)))
    out.append('<text x="14" y="%.1f" %s font-size="11" text-anchor="middle" transform="rotate(-90 14 %.1f)">%s</text>'
               % (oy + h / 2, FONT, oy + h / 2, escape(p.ylabel)))
    out.append('<g clip-path="url(#c%d)">' % top)
    legend = []
    for k, s in enumerate(p.series):
        colour = s['colour'] or colours[k % len(colours)]
        ok = np.isfinite(s['x']) & np.isfinite(s['y'])
        if s['kind'] == 'line':
            segs, cur = [], []
            for xv, yv, good in zip(s['x'], s['y'], ok):
                if good:
                    cur.append('%.1f,%.1f' % (X(xv), Y(yv)))
                elif cur:
                    segs.append(cur)
                    cur = []
            if cur:
                segs.append(cur)
            dash = '' if not s['dash'] else ' stroke-dasharray="%s"' % s['dash']
            for seg in segs:
                if len(seg) == 1:
                    seg = seg * 2
                out.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="%.1f"%s/>'
                           % (' '.join(seg), colour, s['width'], dash))
        elif s['kind'] == 'points':
            for xv, yv in zip(s['x'][ok], s['y'][ok]):
                out.append('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" fill-opacity="0.7"/>'
                           % (X(xv), Y(yv), s['size'], colour))
        else:
            for xv, yv, rv in zip(s['x'][ok], s['y'][ok], s['r'][ok]):
                out.append('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="none" stroke="%s"/>'
                           % (X(xv), Y(yv), max(1.5, rv * sx), colour))
        if s.get('label'):
            legend.append((s['label'], colour, s['kind']))
    out.append('</g>')
    bottom = oy + h + 36
    if legend:
        lx, ly = left, bottom + 4
        for label, colour, kind in legend:
            width_px = 30 + _text_width(label)
            if lx + width_px > left + w and lx > left:
                lx, ly = left, ly + 15
            if kind == 'line':
                out.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="%s" stroke-width="2"/>'
                           % (lx, ly - 4, lx + 16, ly - 4, colour))
            elif kind == 'circles':
                out.append('<circle cx="%d" cy="%.1f" r="3.5" fill="none" stroke="%s"/>' % (lx + 8, ly - 4, colour))
            else:
                out.append('<circle cx="%d" cy="%.1f" r="3" fill="%s"/>' % (lx + 8, ly - 4, colour))
            out.append('<text x="%d" y="%.1f" %s font-size="10">%s</text>' % (lx + 21, ly, FONT, escape(label)))
            lx += width_px
        bottom = ly + 8
    if p.note:
        out.append('<text x="%d" y="%.1f" %s font-size="10" fill="#555">%s</text>' % (left, bottom + 2, FONT,
                                                                                      escape(p.note)))
        bottom += 14
    return out, bottom


def figure(path, panels, title='', width=900, caption=None):
    body, top = [], 8
    if title:
        body.append('<text x="12" y="22" %s font-size="15" font-weight="bold">%s</text>' % (FONT, escape(title)))
        top = 32
    for p in panels:
        part, top = _panel_svg(p, top, width, COLOURS)
        body += part
        top += 6
    if caption:
        for line in caption if isinstance(caption, (list, tuple)) else [caption]:
            body.append('<text x="12" y="%d" %s font-size="10" fill="#444">%s</text>' % (top + 10, FONT, escape(line)))
            top += 14
    height = int(top + 8)
    svg = ('<?xml version="1.0" encoding="UTF-8"?>\n'
           '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">\n'
           '<rect width="100%%" height="100%%" fill="white"/>\n%s\n</svg>\n'
           % (width, height, width, height, '\n'.join(body)))
    with open(path, 'wb') as fh:
        fh.write(svg.encode('utf-8'))
    return path

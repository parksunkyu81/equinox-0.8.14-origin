"""Navigation route for the onroad mini-map.

TmapNda's "경로선 콤마 표시" toggle POSTs the Kakao route to the comma on HTTP
7713 (/api/navi/tmapnda) once a second: {"rgdata": {...}, "vrtx": [{"x": lon,
"y": lat}, ...]}. Kakao's polyline is thinned (about 100 points for 7 km).

The EON has no GPS, so the car's position is the phone's: TmapNda puts its last
known fix (latitude / longitude / heading / gps_speed) into the apilot packet on
UDP. That fix has no timestamp, arrives at 1-2 Hz and can be stale (a parked
phone reported a position 200 km away), so it is only used when it lies within
OFF_ROUTE_M of the route. Between fixes the car is advanced along the route by
carState vEgo, so the map moves smoothly and points along the road.

Cost: the 1 s POST is hashed (route part only) and parsed only when the route
changes; matching is numpy over the route's few hundred segments.
"""
import bisect
import hashlib
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from common.realtime import sec_since_boot

HTTP_PORT = 7713
ROUTE_STALE_S = 10.     # no POST for this long: drop the route
FIX_STALE_S = 5.        # no new phone fix for this long: stop drawing
OFF_ROUTE_M = 60.       # a fix farther than this from the route is not used
BEHIND_M = 80.          # route published behind the car
AHEAD_M = 700.          # ... and ahead of it
FIX_BLEND = 0.5         # share of the fix-vs-dead-reckoning gap closed per fix
SNAP_M = 50.            # a larger gap (re-route, long outage) jumps straight to the fix
EARTH_M_PER_DEG = 111320.
SLICE_STEP_M = 10.      # rebuild the published route slice after moving this far


class _Handler(BaseHTTPRequestHandler):
  owner = None

  def do_POST(self):  # noqa: N802 (http.server API)
    try:
      n = int(self.headers.get('Content-Length', 0))
      body = self.rfile.read(n) if 0 < n < 4 * 1024 * 1024 else b''
      if body:
        self.owner.on_post(body)
      self.send_response(200)
      self.send_header('Content-Type', 'application/json')
      self.end_headers()
      self.wfile.write(b'{"ok":true}')
    except Exception:
      try:
        self.send_response(400)
        self.end_headers()
      except Exception:
        pass

  def log_message(self, *args):
    pass


def route_from_body(body):
  """vrtx [(lon, lat)] from a POST body, or None when it carries no route."""
  obj = json.loads(body.decode('utf-8'))
  v = obj.get('vrtx') if isinstance(obj, dict) else None
  if not isinstance(v, list):
    return None
  pts = []
  for p in v:
    try:
      x, y = float(p['x']), float(p['y'])
    except (KeyError, TypeError, ValueError):
      continue
    if 120. < x < 135. and 30. < y < 45. and (not pts or (x, y) != pts[-1]):
      pts.append((x, y))
  return pts if len(pts) >= 2 else None


class NaviRoute:
  def __init__(self, port=HTTP_PORT, start_server=True):
    self.lock = threading.Lock()
    self.route_hash = None
    self.route_ll = None       # (N, 2) lon, lat
    self.route_t = -1e9
    self.new_route = False
    self.posts = 0
    # local frame of the current route
    self.lat0 = self.lon0 = 0.
    self.kx = EARTH_M_PER_DEG
    self.xy = None             # (N, 2) east, north [m]
    self.cum = None            # (N,) distance along the route [m]
    self.xy_l = self.cum_l = None   # plain-list copies: per-tick lookups stay out of numpy
    self.slice_s = None
    self.slice = None
    # car state along the route
    self.s = None
    self.last_fix = None
    self.fix_t = -1e9
    self.last_t = None
    self.server = None
    if start_server:
      try:
        _Handler.owner = self
        self.server = ThreadingHTTPServer(('0.0.0.0', port), _Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
      except OSError as e:
        print('navi_route: HTTP %d unavailable: %s' % (port, e))

  # --- HTTP thread ---
  def on_post(self, body, now=None):
    now = sec_since_boot() if now is None else now
    i = body.find(b'"vrtx"')
    if i < 0:
      return
    h = hashlib.md5(body[i:]).digest()
    with self.lock:
      self.posts += 1
      self.route_t = now
      if h == self.route_hash:
        return
    pts = route_from_body(body)
    if pts is None:
      return
    with self.lock:
      self.route_hash = h
      self.route_ll = np.array(pts, dtype=np.float64)
      self.new_route = True

  # --- main loop ---
  def _take_route(self):
    with self.lock:
      if not self.new_route:
        return
      ll, self.new_route = self.route_ll, False
    self.lat0 = float(ll[:, 1].mean())
    self.lon0 = float(ll[:, 0].mean())
    self.kx = EARTH_M_PER_DEG * math.cos(math.radians(self.lat0))
    self.xy = np.stack([(ll[:, 0] - self.lon0) * self.kx, (ll[:, 1] - self.lat0) * EARTH_M_PER_DEG], axis=1)
    self.cum = np.concatenate([[0.], np.cumsum(np.hypot(*np.diff(self.xy, axis=0).T))])
    self.xy_l, self.cum_l = self.xy.tolist(), self.cum.tolist()
    self.slice_s = None
    self.s = None   # re-match on the next fix

  def _match(self, x, y):
    """(distance along the route, distance off it) of a point [m]."""
    a, b = self.xy[:-1], self.xy[1:]
    ab = b - a
    t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / np.maximum((ab * ab).sum(1), 1e-9), 0., 1.)
    px, py = a[:, 0] + t * ab[:, 0], a[:, 1] + t * ab[:, 1]
    d = np.hypot(px - x, py - y)
    k = int(np.argmin(d))
    return float(self.cum[k] + t[k] * np.hypot(*ab[k])), float(d[k])

  def _point_at(self, s):
    cum, xy = self.cum_l, self.xy_l
    s = min(max(s, 0.), cum[-1])
    k = min(max(bisect.bisect_right(cum, s) - 1, 0), len(cum) - 2)
    seg = cum[k + 1] - cum[k]
    t = 0. if seg <= 0 else (s - cum[k]) / seg
    a, b = xy[k], xy[k + 1]
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))

  def _to_ll(self, xy):
    return xy[0] / self.kx + self.lon0, xy[1] / EARTH_M_PER_DEG + self.lat0

  def update(self, now, apilot, v_ego):
    self._take_route()
    dt = 0. if self.last_t is None else min(max(now - self.last_t, 0.), 0.5)
    self.last_t = now
    if self.xy is None:
      return
    if self.s is not None:
      self.s += v_ego * dt
    if isinstance(apilot, dict):
      try:
        lat, lon = float(apilot['latitude']), float(apilot['longitude'])
      except (KeyError, TypeError, ValueError):
        lat = lon = None
      if lat is not None and (lat, lon) != self.last_fix:
        self.last_fix = (lat, lon)
        s_fix, off = self._match((lon - self.lon0) * self.kx, (lat - self.lat0) * EARTH_M_PER_DEG)
        if off <= OFF_ROUTE_M:
          self.fix_t = now
          if self.s is None:
            self.s = s_fix
          else:
            gap = s_fix - self.s
            self.s = s_fix if abs(gap) > SNAP_M else self.s + gap * FIX_BLEND
    if self.s is not None:
      self.s = min(max(self.s, 0.), float(self.cum[-1]))

  def valid(self, now):
    with self.lock:
      fresh = now - self.route_t < ROUTE_STALE_S
    return fresh and self.xy is not None and self.s is not None and now - self.fix_t < FIX_STALE_S

  def fill(self, msg, now, dist_to_turn=0.):
    ok = self.valid(now)
    msg.miniMapValid = bool(ok)
    if not ok:
      return
    s = self.s
    car = self._point_at(s)
    a, b = self._point_at(s - 5.), self._point_at(s + 15.)
    heading = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.   # 0 = north, clockwise
    lon, lat = self._to_ll(car)
    msg.carLat, msg.carLon, msg.carHeading = float(lat), float(lon), float(heading)
    if self.slice_s is None or abs(s - self.slice_s) > SLICE_STEP_M:
      lo, hi = s - BEHIND_M - SLICE_STEP_M, s + AHEAD_M
      cum = self.cum_l
      k0, k1 = bisect.bisect_right(cum, lo), bisect.bisect_left(cum, hi)
      pts = [self._point_at(lo)] + self.xy_l[k0:k1] + [self._point_at(hi)]
      ll = [self._to_ll(p) for p in pts]
      self.slice = ([p[1] for p in ll], [p[0] for p in ll])
      self.slice_s = s
    msg.routeLat, msg.routeLon = self.slice
    if dist_to_turn > 0.:
      tl = self._to_ll(self._point_at(s + dist_to_turn))
      msg.turnLon, msg.turnLat = float(tl[0]), float(tl[1])

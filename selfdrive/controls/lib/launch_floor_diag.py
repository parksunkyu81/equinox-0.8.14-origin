"""Why the stop-and-go launch floor did or did not reach the pedal.

On the 2026-11-19 drives the launch floor (stop_accel_boost.py) never showed up
in a single one of the 10 automatic launches: the command climbed from 0 on the
PID alone and the car took ~2 s to break away. The latch was on, so the floor
was refused by the context gate -- but the rlog does not carry the floor or the
gate, so it could not say which term. This records it.

An episode starts when the launch latch sets and ends when it clears, the car
reaches END_KPH, or MAX_S passes. One JSON line per episode goes to
/data/log/launch_floor.jsonl: the lead at the start, how long the latch was
active, how long the floor was allowed / refused / on, the seconds each gate
term refused it, the plan sources seen, and how quickly the car moved.

Cost: a few comparisons a frame while a latch is set (seconds per launch),
nothing otherwise; the file is written from a thread.
"""
import json
import math
import os
import threading
import time

LOG_PATH = os.getenv("LAUNCH_FLOOR_LOG_PATH", "/data/log/launch_floor.jsonl")
LOG_MAX_BYTES = 1024 * 1024

MOVE_MS = 0.3
END_KPH = 25.0
MAX_S = 15.0


def _f(x, nd=2):
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return round(x, nd) if math.isfinite(x) else None


def _write_async(line):
  def write():
    try:
      if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
        os.replace(LOG_PATH, LOG_PATH + ".1")
      with open(LOG_PATH, 'a') as f:
        f.write(line + "\n")
    except Exception:
      pass
  threading.Thread(target=write, daemon=True).start()


class LaunchFloorDiag:
  def __init__(self, dt=0.01, write_fn=None):
    self.dt = dt
    self.write_fn = write_fn or _write_async
    self.ep = None

  def update(self, latched, active, floor_allowed, block_reasons, floor_accel,
             v_ego, a_ego, gas_pressed, brake_pressed, lead_d, lead_v, plan_source,
             lead_accel=0.0, match_floor=0.0):
    ep = self.ep
    if ep is None:
      if not latched:
        return
      ep = self.ep = {
        'wall': round(time.time(), 1), 't': 0.0,
        'v_kph': _f(v_ego * 3.6, 1), 'standstill': bool(v_ego < MOVE_MS),
        'lead_d': _f(lead_d, 1), 'lead_v': _f(lead_v),
        'active_s': 0.0, 'allowed_s': 0.0, 'blocked_s': 0.0, 'floor_on_s': 0.0,
        'floor_max': 0.0, 'first_block': None, 'reasons': {}, 'sources': {},
        't_active': None, 't_floor': None, 't_move': None, 't_10kph': None,
        'a_max': 0.0, 'gas': False, 'brake': False,
        'a_lead_max': 0.0, 'match_max': 0.0, 'drop_s': 0.0,
        'gap_10kph': None, 'lead_v_10kph': None,
      }
    else:
      ep['t'] += self.dt
    t = ep['t']

    if active:
      ep['active_s'] += self.dt
      if ep['t_active'] is None:
        ep['t_active'] = round(t, 2)
    if floor_allowed:
      ep['allowed_s'] += self.dt
      ep['sources'][plan_source] = ep['sources'].get(plan_source, 0.0) + self.dt
      if block_reasons:
        ep['blocked_s'] += self.dt
        if ep['first_block'] is None:
          ep['first_block'] = list(block_reasons)
        for r in block_reasons:
          ep['reasons'][r] = ep['reasons'].get(r, 0.0) + self.dt
    if floor_accel > 0.0:
      ep['floor_on_s'] += self.dt
      ep['floor_max'] = max(ep['floor_max'], float(floor_accel))
      if ep['t_floor'] is None:
        ep['t_floor'] = round(t, 2)
    if ep['standstill'] and ep['t_move'] is None and v_ego >= MOVE_MS:
      ep['t_move'] = round(t, 2)
    if ep['t_10kph'] is None and v_ego * 3.6 >= 10.0:
      ep['t_10kph'] = round(t, 2)
      ep['gap_10kph'] = _f(lead_d, 1)
      ep['lead_v_10kph'] = _f(lead_v)
    ep['a_lead_max'] = max(ep['a_lead_max'], float(lead_accel))
    ep['match_max'] = max(ep['match_max'], float(match_floor))
    # Launch band with the latch emitting but no floor: the 0.3-0.5 s pedal
    # holes the hold hysteresis is meant to remove.
    if active and floor_accel <= 0.0 and MOVE_MS <= v_ego < 10.0 / 3.6:
      ep['drop_s'] += self.dt
    ep['a_max'] = max(ep['a_max'], float(a_ego))
    ep['gas'] = ep['gas'] or bool(gas_pressed)
    ep['brake'] = ep['brake'] or bool(brake_pressed)

    end = None
    if not latched:
      end = 'unlatched'
    elif v_ego * 3.6 >= END_KPH:
      end = 'speed'
    elif t >= MAX_S:
      end = 'timeout'
    if end is None:
      return

    ep['end'] = end
    ep['v_end_kph'] = _f(v_ego * 3.6, 1)
    for k in ('t', 'active_s', 'allowed_s', 'blocked_s', 'floor_on_s', 'floor_max', 'a_max',
              'a_lead_max', 'match_max', 'drop_s'):
      ep[k] = _f(ep[k])
    ep['reasons'] = {k: _f(v) for k, v in ep['reasons'].items()}
    ep['sources'] = {k: _f(v) for k, v in ep['sources'].items()}
    self.ep = None
    # A latch that never went active (driver held the brake, lead re-stopped)
    # says nothing about the floor; keep the file to real launches.
    if ep['active_s'] > 0.0 or ep['allowed_s'] > 0.0:
      self.write_fn(json.dumps(ep, separators=(',', ':')))

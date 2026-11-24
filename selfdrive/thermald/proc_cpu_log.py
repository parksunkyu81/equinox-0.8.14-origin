"""Who uses the CPU while driving: one JSON line every WINDOW_S onroad.

The rlog carries no procLog or deviceState, and process_diagnostics.jsonl has
only controlsd's own timing. From 11-22 controlsd's preemptions rose ~30%
(310 -> 410 /s) and its lagging windows went from ~10 to ~70 an hour, with no
way to say which process took the time. This records it:

  cores   busy % of each core over the window (/proc/stat)
  top     the TOP_N processes by CPU % (user+system, 100 = one core), with the
          core they last ran on, their RT priority and nice
          as [name, pid, pct, last_cpu, rt_prio, nice]
  rt      the TOP_N real-time threads by CPU % (threads of processes that run
          SCHED_FIFO/RR, kernel IRQ threads included): only these can take
          core 3 from controlsd (FIFO 53); [name/thread, tid, pct, last_cpu, rt_prio]

Written by thermald (not real-time, its own 2 Hz loop) to
/data/log/proc_cpu.jsonl, rotated at 2 MB. Cost, measured on the EON: 70-95 ms
per snapshot (every process's stat, plus the threads of openpilot processes),
once per 30 s window = ~0.3% of one core, on whichever core thermald runs.
"""
import json
import os
import time

LOG_PATH = os.getenv("PROC_CPU_LOG_PATH", "/data/log/proc_cpu.jsonl")
LOG_MAX_BYTES = 2 * 1024 * 1024
WINDOW_S = 30.
TOP_N = 12
MIN_PCT = 0.5
# Processes whose threads are walked for RT ones (plus any RT process).
OP_PREFIXES = ('_', 'selfdrive', 'manager')
OP_NATIVE = {'camerad', 'boardd', 'loggerd', 'modeld', 'dmonitoringmodeld', 'encoderd', 'proclogd'}


def _read(path):
  try:
    with open(path, 'rb') as f:
      return f.read()
  except OSError:
    return None


def _stat(path):
  """(comm, utime+stime ticks, last cpu, rt_priority, nice, num_threads, starttime) or None."""
  raw = _read(path)
  if raw is None:
    return None
  try:
    s = raw.decode('utf-8', 'replace')
    lp, rp = s.index('('), s.rindex(')')
    f = s[rp + 2:].split()
    # f[0] is field 3 (state): utime 14 -> f[11], stime f[12], nice 19 -> f[16],
    # num_threads f[17], starttime 22 -> f[19], processor 39 -> f[36], rt_priority f[37]
    return (s[lp + 1:rp], int(f[11]) + int(f[12]), int(f[36]), int(f[37]), int(f[16]), int(f[17]), int(f[19]))
  except (ValueError, IndexError):
    return None


def _cores():
  raw = _read('/proc/stat')
  out = {}
  if raw is None:
    return out
  for line in raw.decode().splitlines():
    if line.startswith('cpu') and line[3:4].isdigit():
      p = line.split()
      v = [int(x) for x in p[1:8]]
      idle = v[3] + v[4]
      out[p[0]] = (sum(v) - idle, idle)
  return out


class ProcCpuLog:
  def __init__(self, path=LOG_PATH, window_s=WINDOW_S):
    self.path = path
    self.window_s = window_s
    self.hz = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100
    self.names = {}   # (pid, starttime) -> name
    self.reset()

  def reset(self):
    self.t0 = None
    self.procs0 = None
    self.rt0 = None
    self.cores0 = None

  def _name(self, pid, comm, start):
    key = (pid, start)
    n = self.names.get(key)
    if n is None:
      raw = _read('/proc/%d/cmdline' % pid)
      args = [a for a in raw.decode('utf-8', 'replace').split('\0') if a] if raw else []
      n = comm
      if args:
        n = os.path.basename(args[0])
        if n.startswith('python') and len(args) > 1:
          n = args[-1] if args[1] == '-m' else os.path.basename(args[1])
      self.names[key] = n
    return n

  def _snapshot(self):
    procs, rt = {}, {}
    for d in os.listdir('/proc'):
      if not d.isdigit():
        continue
      pid = int(d)
      st = _stat('/proc/%d/stat' % pid)
      if st is None:
        continue
      comm, ticks, cpu, rtp, nice, nthr, start = st
      name = self._name(pid, comm, start)
      procs[(pid, start)] = (name, ticks, cpu, rtp, nice)
      if rtp > 0 and nthr == 1:
        rt[(pid, pid, start)] = (name, ticks, cpu, rtp)
      elif nthr > 1 and (rtp > 0 or name.startswith(OP_PREFIXES) or name in OP_NATIVE):
        # threads of an openpilot (or RT) process: only RT ones matter for core 3.
        # Android apps' threads are skipped: walking them all cost ~150 ms.
        try:
          tids = os.listdir('/proc/%d/task' % pid)
        except OSError:
          continue
        for t in tids:
          tst = _stat('/proc/%d/task/%s/stat' % (pid, t))
          if tst is not None and tst[3] > 0:
            rt[(pid, int(t), start)] = ('%s/%s' % (name, tst[0]), tst[1], tst[2], tst[3])
    return procs, rt

  def update(self, now, onroad):
    if not onroad:
      self.reset()
      return
    if self.t0 is not None and now - self.t0 < self.window_s:
      return
    c0 = time.monotonic()
    procs, rt = self._snapshot()
    cores = _cores()
    cost_ms = (time.monotonic() - c0) * 1000.
    if self.t0 is not None:
      self._emit(now - self.t0, procs, rt, cores, cost_ms)
    # drop names of processes that are gone
    live = set(procs)
    self.names = {k: v for k, v in self.names.items() if k in live}
    self.t0, self.procs0, self.rt0, self.cores0 = now, procs, rt, cores

  def _pct(self, ticks, dt):
    return round(100. * ticks / self.hz / dt, 1)

  def _emit(self, dt, procs, rt, cores, cost_ms):
    top = []
    for k, (name, ticks, cpu, rtp, nice) in procs.items():
      p0 = self.procs0.get(k)
      if p0 is None:
        continue
      pct = self._pct(ticks - p0[1], dt)
      if pct >= MIN_PCT:
        top.append([name, k[0], pct, cpu, rtp, nice])
    top.sort(key=lambda x: -x[2])
    rts = []
    for k, (name, ticks, cpu, rtp) in rt.items():
      r0 = self.rt0.get(k)
      if r0 is None:
        continue
      pct = self._pct(ticks - r0[1], dt)
      if pct >= MIN_PCT:
        rts.append([name, k[1], pct, cpu, rtp])
    rts.sort(key=lambda x: -x[2])
    busy = {}
    for c, (b, i) in cores.items():
      if c in self.cores0:
        db, di = b - self.cores0[c][0], i - self.cores0[c][1]
        busy[c] = round(100. * db / (db + di), 1) if db + di > 0 else 0.
    line = json.dumps({
      'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime()),
      'window_s': round(dt, 2), 'cost_ms': round(cost_ms, 1),
      'cores': [busy.get('cpu%d' % i) for i in range(4)],
      'top': top[:TOP_N], 'rt': rts[:TOP_N],
    }, separators=(',', ':'))
    try:
      if os.path.exists(self.path) and os.path.getsize(self.path) > LOG_MAX_BYTES:
        os.replace(self.path, self.path + '.1')
      with open(self.path, 'a') as f:
        f.write(line + '\n')
    except OSError:
      pass

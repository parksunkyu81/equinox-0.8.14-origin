#!/usr/bin/env python3
import os
import signal
import time
import datetime

from common.params import Params
from selfdrive.hardware.eon.hardware import getprop
from selfdrive.swaglog import cloudlog

# How long loggerd gets to close the segment it has open before it is killed.
# Android kills everything holding /data/media moments after the shutdown
# property is set, so this is the only window it gets.
LOGGERD_CLOSE_TIMEOUT_S = 3.0


def loggerd_pids():
  """PIDs whose kernel name is exactly loggerd.

  This replaced `pkill -9 loggerd`, which was killing two processes. BusyBox
  pgrep/pkill match the pattern against the command line, not the process name,
  so "loggerd" also matched `selfdrive.loggerd.deleter` -- which is why every
  shutdown in this device's logs shows loggerd and deleter dying together on
  signal 9. -x does not help: it wants the whole command line, and loggerd's is
  "./loggerd". /proc/<pid>/comm is the kernel's own name, capped at 15
  characters, so it reads "loggerd" for the native binary and
  "selfdrive.logge" for the Python one -- an exact comparison tells them apart.
  """
  pids = []
  for entry in os.listdir("/proc"):
    if not entry.isdigit():
      continue
    try:
      with open(os.path.join("/proc", entry, "comm")) as f:
        if f.read().strip() == "loggerd":
          pids.append(int(entry))
    except (OSError, ValueError):
      continue
  return pids


def stop_loggerd():
  """Ask loggerd to finish, then insist.

  SIGKILL on its own leaves the open rlog.bz2 truncated: the last segment of a
  drive cannot be decompressed past the point it was cut, which is every
  drive's last minute. loggerd closes the segment on SIGINT.
  """
  pids = loggerd_pids()
  for pid in pids:
    try:
      os.kill(pid, signal.SIGINT)
    except OSError:
      pass
  deadline = time.monotonic() + LOGGERD_CLOSE_TIMEOUT_S
  while time.monotonic() < deadline and loggerd_pids():
    time.sleep(0.05)
  remaining = loggerd_pids()
  for pid in remaining:
    cloudlog.warning("loggerd %d did not close its segment in %.1fs; SIGKILL"
                     % (pid, LOGGERD_CLOSE_TIMEOUT_S))
    try:
      os.kill(pid, signal.SIGKILL)
    except OSError:
      pass


def main():
  prev = b""
  params = Params()
  while True:
    with open("/dev/__properties__", 'rb') as f:
      cur = f.read()

    if cur != prev:
      prev = cur

      # 0 for shutdown, 1 for reboot
      prop = getprop("sys.shutdown.requested")
      if prop is not None and len(prop) > 0:
        stop_loggerd()
        params.put("LastSystemShutdown", f"'{prop}' {datetime.datetime.now()}")
        os.sync()

        time.sleep(120)
        cloudlog.error('shutdown false positive')
        break

    time.sleep(0.1)

if __name__ == "__main__":
  main()

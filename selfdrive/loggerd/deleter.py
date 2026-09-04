#!/usr/bin/env python3
"""Drive-log retention.

Two independent policies run here:

  * The original emergency policy: once the disk is nearly full, delete the
    oldest directory, one per pass, until it is not.
  * An age policy: keep the last PRESERVE_DAYS days of drive segments and
    delete the rest, regardless of how much room is left.

The age policy exists because the emergency one only reacts at 5 GB / 10 %
free, so /data/media/0/realdata is allowed to grow to tens of gigabytes first
and every drive is written against an almost-full disk. On this device the
uploader is disabled (see process_config.py), so nothing is ever offloaded and
nothing here is deleting data that is waiting to be uploaded.

Only directories named like a logger segment -- 2026-09-04--00-09-20--7 -- are
ever considered. Anything else living under ROOT, such as this fork's
lateral_bias_csv, is not a drive log and is left alone.
"""
import os
import re
import shutil
import threading
from datetime import datetime, timedelta

from selfdrive.swaglog import cloudlog
from selfdrive.loggerd.config import ROOT, get_available_bytes, get_available_percent
from selfdrive.loggerd.uploader import listdir_by_creation

MIN_BYTES = 5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

# How many days of drive segments to keep. Change this one number to change
# the retention window.
PRESERVE_DAYS = 3

# rmtree on a segment is tens of megabytes of I/O. Cap how many go per pass so
# a large backlog is cleared over several seconds of wall clock instead of
# blocking this thread -- and the disk -- in one burst mid-drive.
MAX_DELETES_PER_PASS = 8

# The device clock can come back wrong after the RTC loses power, and "every
# log is older than three days" is exactly what a reset clock looks like. Two
# guards, both derived from data already on disk:
#   * a hard floor no plausible present is below
#   * the newest segment on disk, which the present can never predate, and
#     which the present is never absurdly far ahead of
# Failing either, the age policy sits out this pass. The emergency policy is
# unaffected and still protects the disk.
CLOCK_SANE_AFTER = datetime(2026, 1, 1)
CLOCK_SANE_WITHIN = timedelta(days=90)

SEGMENT_RE = re.compile(r'^(?P<route>\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2})--\d+$')


def segment_started_at(name):
  """Return when a segment directory's route began, or None if it is not one.

  The name is the logger's own record of the route start, so it is used in
  preference to any filesystem timestamp, which a copy or a touch would move.
  """
  match = SEGMENT_RE.match(name)
  if match is None:
    return None
  try:
    return datetime.strptime(match.group('route'), '%Y-%m-%d--%H-%M-%S')
  except ValueError:
    return None


def expired_segments(names, now, preserve_days=PRESERVE_DAYS):
  """Return the segment directories that have aged out.

  Pure: it takes a listing and a time and returns names, so the policy can be
  reasoned about and tested without a filesystem.

  The most recent route is always kept, whatever its age. Otherwise a device
  parked for longer than the window would come back with no logs at all,
  including the drive the owner most likely wants to look at.
  """
  dated = []
  for name in names:
    started = segment_started_at(name)
    if started is not None:
      dated.append((started, name))

  if not dated:
    return []

  newest = max(started for started, _ in dated)
  if now < CLOCK_SANE_AFTER or now < newest or now - newest > CLOCK_SANE_WITHIN:
    cloudlog.warning(f"deleter: clock at {now} not usable for age deletion, "
                     f"newest segment is {newest}")
    return []

  newest_route = max(name for started, name in dated if started == newest)
  keep_prefix = SEGMENT_RE.match(newest_route).group('route') + '--'

  cutoff = now - timedelta(days=preserve_days)
  return sorted(name for started, name in dated
                if started < cutoff and not name.startswith(keep_prefix))


def delete_segment(name):
  """Delete one segment directory. Returns True if it is now gone."""
  path = os.path.join(ROOT, name)
  try:
    if any(fname.endswith(".lock") for fname in os.listdir(path)):
      # Still being written. It cannot be older than the window in practice,
      # but the check costs nothing and the consequence of being wrong is a
      # half-deleted active segment.
      return False
    shutil.rmtree(path)
    return True
  except OSError:
    cloudlog.exception(f"issue deleting {path}")
    return False


def delete_expired(now=None):
  """Delete aged-out segments, up to MAX_DELETES_PER_PASS. Returns the count."""
  now = datetime.now() if now is None else now
  expired = expired_segments(listdir_by_creation(ROOT), now)
  if not expired:
    return 0

  deleted = 0
  for name in expired[:MAX_DELETES_PER_PASS]:
    if delete_segment(name):
      deleted += 1
  if deleted:
    cloudlog.info(f"deleter: removed {deleted} segment(s) older than "
                  f"{PRESERVE_DAYS} days, {len(expired) - deleted} still queued")
  return deleted


def deleter_thread(exit_event):
  while not exit_event.is_set():
    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      # remove the earliest directory we can
      dirs = sorted(listdir_by_creation(ROOT), key=lambda x: x in DELETE_LAST)
      for delete_dir in dirs:
        delete_path = os.path.join(ROOT, delete_dir)

        if any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        try:
          cloudlog.info(f"deleting {delete_path}")
          if os.path.isfile(delete_path):
            os.remove(delete_path)
          else:
            shutil.rmtree(delete_path)
          break
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
      exit_event.wait(.1)
    elif delete_expired():
      # A backlog is worked off a few segments at a time. Come back sooner
      # than the idle poll, but leave the disk alone in between.
      exit_event.wait(1.0)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()

import time
import traceback

from selfdrive.process_diagnostics import append_process_diagnostic
from selfdrive.swaglog import cloudlog


PLANNER_EXCEPTION_LOG_INTERVAL = 5.0


def run_planner_cycle(planner_name, planner, sm, pm, last_exception_log):
  """Keep a transient fault in one planner from terminating both planners."""
  try:
    planner.update(sm)
    planner.publish(sm, pm)
  except Exception as exc:
    now = time.monotonic()
    if now - last_exception_log >= PLANNER_EXCEPTION_LOG_INTERVAL:
      traceback_text = traceback.format_exc()
      cloudlog.error("%s planner cycle failed\n%s", planner_name, traceback_text)
      append_process_diagnostic(
        "planner_cycle_exception",
        planner=planner_name,
        exception_type=type(exc).__name__,
        exception_message=str(exc),
        traceback=traceback_text,
      )
      return now
  return last_exception_log

def expected_not_running_processes(process_states, ignored_processes):
  """Return only processes that manager expects to run but currently are not running."""
  ignored_processes = set(ignored_processes)
  return {
    process.name for process in process_states
    if process.shouldBeRunning and not process.running and process.name not in ignored_processes
  }


def update_process_not_running_state(counter, candidates, not_running, required_updates):
  """Track processes that remain expected-but-not-running across managerState updates."""
  not_running = set(not_running)
  candidates = set(candidates)

  if not not_running:
    return 0, set(), False

  if counter <= 0:
    counter = 1
    candidates = not_running
  else:
    persistent = candidates & not_running
    if persistent:
      counter += 1
      candidates = persistent
    else:
      counter = 1
      candidates = not_running

  return counter, candidates, counter >= required_updates

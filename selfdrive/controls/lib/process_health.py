def expected_not_running_processes(process_states, ignored_processes):
  """Return only processes that manager expects to run but currently are not running."""
  ignored_processes = set(ignored_processes)
  return {
    process.name for process in process_states
    if process.shouldBeRunning and not process.running and process.name not in ignored_processes
  }


def controlsd_communication_ok(submaster, optional_validity_services=()):
  """Check control inputs while allowing derived data to fall back safely.

  Optional-validity services must still be alive. Their payload validity is not
  allowed to raise commIssue because consumers retain the last valid value.
  """
  optional_validity_services = set(optional_validity_services)
  required_services = [
    service for service in submaster.alive
    if service not in optional_validity_services
  ]
  optional_services_alive = all(
    submaster.alive.get(service, False)
    for service in optional_validity_services
  )
  return submaster.all_checks(service_list=required_services) and optional_services_alive


def panda_power_down_in_progress(charging_disabled, controls_enabled):
  """True only for an intentional, disengaged EON Panda power transition.

  Switching a Black Panda from CDP to CLIENT briefly removes the CAN USB
  device. That expected offroad transition must not be surfaced as a process
  communication failure, but a disconnect while controls are enabled remains
  an immediate fault.
  """
  return bool(charging_disabled and not controls_enabled)


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

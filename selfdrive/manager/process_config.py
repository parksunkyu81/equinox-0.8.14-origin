import os

from selfdrive.hardware import EON, TICI, PC
from selfdrive.manager.process import PythonProcess, NativeProcess, DaemonProcess

WEBCAM = os.getenv("USE_WEBCAM") is not None
EQUINOX_SIMULATOR = os.getenv("EQUINOX_SIMULATOR") == "1"

procs = [
  #DaemonProcess("manage_athenad", "selfdrive.athena.manage_athenad", "AthenadPid"),
  # due to qualcomm kernel bugs SIGKILLing camerad sometimes causes page table corruption
  NativeProcess("camerad", "selfdrive/camerad", ["./camerad"], unkillable=True, driverview=True),
  NativeProcess("clocksd", "selfdrive/clocksd", ["./clocksd"]),
  NativeProcess("dmonitoringmodeld", "selfdrive/modeld", ["./dmonitoringmodeld"],
                enabled=(not EQUINOX_SIMULATOR and (not PC or WEBCAM)), driverview=True),
  #NativeProcess("logcatd", "selfdrive/logcatd", ["./logcatd"]),
  NativeProcess("loggerd", "selfdrive/loggerd", ["./loggerd"]),
  NativeProcess("modeld", "selfdrive/modeld", ["./modeld"], enabled=not EQUINOX_SIMULATOR),
  NativeProcess("navd", "selfdrive/ui/navd", ["./navd"], enabled=(PC or TICI), persistent=True),
  NativeProcess("proclogd", "selfdrive/proclogd", ["./proclogd"]),
  NativeProcess("sensord", "selfdrive/sensord", ["./sensord"],
                enabled=(not EQUINOX_SIMULATOR and not PC), persistent=EON, sigkill=EON),
  NativeProcess("ubloxd", "selfdrive/locationd", ["./ubloxd"],
                enabled=(not EQUINOX_SIMULATOR and (not PC or WEBCAM))),
  NativeProcess("ui", "selfdrive/ui", ["./ui"], persistent=True, watchdog_max_dt=(5 if TICI else None)),
  NativeProcess("soundd", "selfdrive/ui/soundd", ["./soundd"], persistent=True),
  NativeProcess("locationd", "selfdrive/locationd", ["./locationd"], enabled=not EQUINOX_SIMULATOR),
  NativeProcess("boardd", "selfdrive/boardd", ["./boardd"], enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", enabled=not EQUINOX_SIMULATOR),
  # Disabled: its live-learned torque params are never consumed. Actual torque
  # control reads fixed latAccelFactor/friction from ntune
  # (selfdrive/ntune.py) via controlsd's update_ntune_torque_params, and
  # LatControlTorque.update_live_torque_params() explicitly discards whatever
  # this would have fed it. Measured ~16% of a CPU core wasted on a real
  # drive for zero effect on steering. See controlsd.py's SubMaster/
  # communication_ok changes removing liveTorqueParameters accordingly.
  PythonProcess("torqued", "selfdrive.locationd.torqued", enabled=False),
  PythonProcess("controlsd", "selfdrive.controls.controlsd"),
  PythonProcess("deleter", "selfdrive.loggerd.deleter", persistent=True),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd",
                enabled=(not EQUINOX_SIMULATOR and (not PC or WEBCAM)), driverview=True),
  PythonProcess("logmessaged", "selfdrive.logmessaged", persistent=True),
  PythonProcess("pandad", "selfdrive.boardd.pandad", persistent=True),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", enabled=not EQUINOX_SIMULATOR),
  PythonProcess("plannerd", "selfdrive.controls.plannerd", enabled=not EQUINOX_SIMULATOR),
  PythonProcess("radard", "selfdrive.controls.radard", enabled=not EQUINOX_SIMULATOR),
  PythonProcess("equinoxcan", "tools.equinox_sim.virtual_panda", enabled=EQUINOX_SIMULATOR, persistent=True),
  PythonProcess("equinoxservices", "tools.equinox_sim.services", enabled=EQUINOX_SIMULATOR, persistent=True),
  PythonProcess("thermald", "selfdrive.thermald.thermald", persistent=True),
  PythonProcess("timezoned", "selfdrive.timezoned", enabled=TICI, persistent=True),
  #PythonProcess("tombstoned", "selfdrive.tombstoned", enabled=not PC, persistent=True),
  #PythonProcess("updated", "selfdrive.updated", enabled=not PC, persistent=True),
  #PythonProcess("uploader", "selfdrive.loggerd.uploader", persistent=True),
  #PythonProcess("statsd", "selfdrive.statsd", persistent=True),

  # EON only
  PythonProcess("rtshield", "selfdrive.rtshield", enabled=EON),
  PythonProcess("shutdownd", "selfdrive.hardware.eon.shutdownd", enabled=EON),
  PythonProcess("androidd", "selfdrive.hardware.eon.androidd", enabled=EON, persistent=True),

  # Experimental
  PythonProcess("rawgpsd", "selfdrive.sensord.rawgps.rawgpsd", enabled=os.path.isfile("/persist/comma/use-quectel-rawgps")),
]

managed_processes = {p.name: p for p in procs}

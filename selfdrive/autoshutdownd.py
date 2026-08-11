#!/usr/bin/env python3

import os
import time
import cereal.messaging as messaging
from common.params import Params


AUTO_SHUTDOWN_SECONDS = 180
AUTO_SHUTDOWN_CHECK_INTERVAL_SECONDS = 5


def update_shutdown_count(shutdown_count, auto_shutdown_enabled, usb_online, started):
  if auto_shutdown_enabled and not usb_online and not started:
    return shutdown_count + AUTO_SHUTDOWN_CHECK_INTERVAL_SECONDS
  return 0


def main():

  shutdown_count = 0
  device_state_sock = messaging.sub_sock('deviceState')
  params = Params()

  while 1:
    msg = messaging.recv_sock(device_state_sock, wait=True)
    if msg is not None:
      # manager initializes AutoShutdown to "1". Treat a missing value as
      # enabled as a fail-safe so upgrading preserves the existing behavior.
      auto_shutdown_param = params.get("AutoShutdown")
      auto_shutdown_enabled = auto_shutdown_param is None or auto_shutdown_param == b"1"
      shutdown_count = update_shutdown_count(
        shutdown_count,
        auto_shutdown_enabled,
        msg.deviceState.usbOnline,
        msg.deviceState.started)
    else:
      shutdown_count = 0

    #print('current', shutdown_count, 'shutdown_at', AUTO_SHUTDOWN_SECONDS)

    if shutdown_count >= AUTO_SHUTDOWN_SECONDS:
      os.system('LD_LIBRARY_PATH="" svc power shutdown')

    time.sleep(AUTO_SHUTDOWN_CHECK_INTERVAL_SECONDS)


if __name__ == "__main__":
  main()

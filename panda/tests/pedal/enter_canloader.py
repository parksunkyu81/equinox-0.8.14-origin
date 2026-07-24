#!/usr/bin/env python3
import argparse
import time

from panda import Panda
from panda.tests.pedal.canhandle import CanHandle


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Flash comma pedal over CAN")
  parser.add_argument("--recover", action="store_true")
  parser.add_argument("firmware", type=str, nargs="?", help="signed pedal firmware")
  args = parser.parse_args()

  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while panda.can_recv():
    pass

  if args.recover:
    panda.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", 0)
    raise SystemExit(0)

  panda.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
  if args.firmware:
    time.sleep(0.1)
    print("flashing", args.firmware)
    with open(args.firmware, "rb") as stream:
      Panda.flash_static(CanHandle(panda, 0), stream.read())

  print("CAN flash complete")

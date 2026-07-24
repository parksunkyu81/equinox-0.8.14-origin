#!/usr/bin/env python3
import argparse
import signal
import struct
import time

from panda import Panda


class CanHandle:
  def __init__(self, panda, bus):
    self.panda = panda
    self.bus = bus

  def transact(self, data):
    self.panda.isotp_send(1, data, self.bus, recvaddr=2)

    def handle_timeout(signum, frame):
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, handle_timeout)
    signal.alarm(1)
    try:
      return self.panda.isotp_recv(2, self.bus, sendaddr=1)
    finally:
      signal.alarm(0)

  def controlWrite(self, request_type, request, value, index, data, timeout=0):
    return self.controlRead(request_type, request, value, index, 0, timeout)

  def controlRead(self, request_type, request, value, index, length, timeout=0):
    data = struct.pack("HHBBHHH", 0, 0, request_type, request, value, index, length)
    return self.transact(data)

  def bulkWrite(self, endpoint, data, timeout=0):
    if len(data) > 0x10:
      raise ValueError("Data must not be longer than 0x10")
    return self.transact(struct.pack("HH", endpoint, len(data)) + data)

  def bulkRead(self, endpoint, length, timeout=0):
    return self.transact(struct.pack("HH", endpoint, 0))


def require_offroad():
  try:
    with open("/data/params/d/IsOffroad", "rb") as stream:
      offroad = stream.read().strip()
  except OSError as error:
    raise RuntimeError("cannot read IsOffroad") from error
  if offroad != b"1":
    raise RuntimeError("refusing pedal flash while IsOffroad is not 1")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Flash a prebuilt comma pedal image over Panda CAN")
  parser.add_argument("firmware", help="signed pedal firmware")
  args = parser.parse_args()

  require_offroad()
  serials = Panda.list()
  if len(serials) != 1:
    raise RuntimeError("expected exactly one Panda, found %d" % len(serials))

  with open(args.firmware, "rb") as stream:
    firmware = stream.read()

  panda = Panda(serials[0])
  try:
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    while panda.can_recv():
      pass

    panda.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", 0)
    time.sleep(0.1)
    Panda.flash_static(CanHandle(panda, 0), firmware)
    print("PEDAL_FLASH_OK")
  finally:
    panda.close()

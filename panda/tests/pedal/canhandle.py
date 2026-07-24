import signal
import struct


class CanHandle:
  def __init__(self, panda, bus):
    self.panda = panda
    self.bus = bus

  def transact(self, data):
    self.panda.isotp_send(1, data, self.bus, recvaddr=2)

    def handle_timeout(signum, frame):
      # The pedal resets at the end of a successful flash.
      raise Exception("timeout")

    signal.signal(signal.SIGALRM, handle_timeout)
    signal.alarm(1)
    try:
      response = self.panda.isotp_recv(2, self.bus, sendaddr=1)
    finally:
      signal.alarm(0)
    return response

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

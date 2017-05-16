import struct
import sys

if sys.version_info[0] >= 3:
    def iter_ints(b):
        return b
else:
    def iter_ints(b):
        for c in b:
            yield ord(c)

class UBXMessage(object):
    def __init__(self, msg_id, payload):
        self.msg_id = msg_id
        self.payload = payload

    def serialise(self):
        msg_body = self.msg_id + struct.pack('<H', len(self.payload)) + self.payload
        checksum = ubx_checksum(msg_body)
        return b'\xB5\x62' + msg_body + checksum + b'\x10\x13'


def ubx_checksum(msg):
    a = b = 0
    for c in iter_ints(msg):
        a = (a + c) & 0xff
        b = (b + a) & 0xff
    return struct.pack('BB', a, b)

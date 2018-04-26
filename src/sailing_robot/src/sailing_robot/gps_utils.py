import os
import struct
import sys
from pynmea2 import NMEASentence

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

    def i2cise(self):
        msg_body = self.msg_id + struct.pack('<H', len(self.payload)) + self.payload
        checksum = ubx_checksum(msg_body)
        msg_string = b'\x62' + msg_body + checksum + b'\x10\x13'
        first_byte = int(b'\xB5'.encode('hex'), 16)

        msg_list = []
        for byte in msg_string:
            msg_list.append(int(byte.encode('hex'), 16))
        return first_byte, msg_list

    def i2cise_serial(self, serial_msg):
        # Convert a serial message to i2c format
        msg_list = []
        first_byte = ''
        for char in serial_msg:
            if not first_byte:
                first_byte = int(char.encode('hex'), 16)
            else:
                msg_list.append(int(char.encode('hex'), 16))
        return first_byte, msg_list


def ubx_checksum(msg):
    a = b = 0
    for c in iter_ints(msg):
        a = (a + c) & 0xff
        b = (b + a) & 0xff
    return struct.pack('BB', a, b)

def get_port():
    """The serial port for the GPS has different names on raspi 2 and 3.
    """
    if os.path.exists('/its_raspi3'):
        return "/dev/serial0"  # Raspi 3
    return "/dev/ttyAMA0"  # Raspi 2

class UbxNmeaParser(object):
    """Parse a mixed stream of UBX and NMEA messages
    
    Our u-blox GPS unit emits a mixture of messages in two formats:
    - Binary UBX messages (Start \\xb5b, length in message)
    - ASCII NMEA 0183 messages (Start $, end \\r\\n)
    
    This parser reads the two types of messages, along with any other chunks
    (e.g. if we start reading half-way through a message). Use .feed(data) to
    supply newly read data, and .get_msgs() to get an iterator of bytes and
    pynmea2.NMEASentence objects representing messages already received.
    
    At present, we're only interested in the content of the NMEA messages, so
    we leave the UBX messages as raw bytes rather than attempting to parse them.
    This may change in the future.
    """
    def __init__(self):
        self.buf = b''

    def feed(self, data):
        self.buf += data

    def _take_chunk(self, n):
        c, self.buf = self.buf[:n], self.buf[n:]
        return c

    def _take_nmea(self):
        crlf_ix = self.buf.find(b'\r\n')
        if crlf_ix == -1:
            return None   # Don't have a complete message yet
        data = self._take_chunk(crlf_ix + 2)
        return NMEASentence.parse(data)

    def _take_ubx(self):
        if len(self.buf) < 6:
            return None    # Don't have the length yet
        payload_length = struct.unpack('<H', self.buf[4:6])[0]
        msg_length = payload_length + 8
        if len(self.buf) >= msg_length:
            return self._take_chunk(msg_length)

    def _next_msg(self):
        nmea_start = self.buf.find(b'$')
        ubx_start = self.buf.find(b'\xb5b')
        if nmea_start == 0:
            return self._take_nmea()
        elif ubx_start == 0:
            return self._take_ubx()
        elif (nmea_start > 0) and (ubx_start > 0):
            return self._take_chunk(min(nmea_start, ubx_start))
        elif nmea_start > 0:
            return self._take_chunk(nmea_start)
        elif ubx_start > 0:
            return self._take_chunk(ubx_start)

    def get_msgs(self):
        return iter(self._next_msg, None)

def test_stream_parser(filename):
    import random
    parser = UbxNmeaParser()
    with open(filename, 'rb') as f:
        while True:
            b = f.read(random.randint(1, 100))
            if b == b'':
                return
            parser.feed(b)
            for msg in parser.get_msgs():
                print(repr(msg))

if __name__ == '__main__':
    test_stream_parser(sys.argv[1])

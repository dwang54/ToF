
"""
Decoder for the VL53L7CX binary wire protocol.
 
Frame layout (before COBS encoding, little-endian throughout):
 
    offset  size  field
    0       1     version        0x01
    1       1     frame type     0x01 = ranging
    2       1     grid width     4 or 8
    3       4     sequence       uint32
    7       4     timestamp_ms   uint32
    11      N     zone records   N = grid_width^2 * 5
    11+N    2     CRC-16/CCITT   over bytes 0 .. 11+N-1
 
Each zone record is 5 bytes: int16 distance_mm, uint16 range_sigma_mm,
uint8 target_status.
 
On the wire: COBS-encode the whole frame above, then append a 0x00
delimiter. Decoding reverses that -- split on 0x00, COBS-decode, verify
CRC, then unpack.
 
CRC IS COMPUTED OVER THE RAW FRAME, BEFORE COBS ENCODING. The firmware
must do the same or every frame will fail.
"""
 
import struct
 
HEADER_FMT = '<BBBII'
HEADER_LEN = struct.calcsize(HEADER_FMT)      # 11
RECORD_LEN = 5
RECORD_LEN_EXT = 9        # base 5 + uint16 signal_kcps + uint16 ambient_kcps
CRC_LEN = 2

PROTO_VERSION = 0x01
TYPE_RANGING = 0x01
TYPE_TEXT = 0x02
TYPE_RANGING_EXT = 0x03

MAX_FRAME = 4096          # buffer guard: bigger than any legal frame 
# --------------------------------------------------------------------------
# CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
# Check value: crc16_ccitt(b"123456789") == 0x29B1
# --------------------------------------------------------------------------
def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
 
 
# --------------------------------------------------------------------------
# COBS
# --------------------------------------------------------------------------
def cobs_encode(data):
    """Encode so the result contains no 0x00 bytes."""
    out = bytearray()
    pos = 0
    n = len(data)
    while True:
        z = data.find(0, pos)
        if z < 0:
            z = n
        chunk = data[pos:z]
 
        # Runs longer than 254 need splitting: a code byte of 0xFF means
        # "254 data bytes follow and NO implicit zero".
        while len(chunk) >= 254:
            out.append(0xFF)
            out += chunk[:254]
            chunk = chunk[254:]
            if not chunk and z == n:
                return bytes(out)
 
        out.append(len(chunk) + 1)
        out += chunk
        if z == n:
            return bytes(out)
        pos = z + 1
 
 
def cobs_decode(data):
    """Inverse of cobs_encode. Raises ValueError on malformed input."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError('zero byte inside COBS frame')
        i += 1
        end = i + code - 1
        if end > n:
            raise ValueError('COBS group overruns frame')
        out += data[i:end]
        i = end
        # A full 254-byte group carries no implicit zero. Nor does the
        # final group.
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)
 
 
# --------------------------------------------------------------------------
# Frame build / parse
# --------------------------------------------------------------------------
def build_frame(seq, timestamp_ms, grid_width, distances, sigmas, statuses,
                frame_type=TYPE_RANGING):
    """Reference encoder. Mirrors what the firmware must produce -- used for
    round-trip tests and for generating fixtures without hardware."""
    n_zones = grid_width * grid_width
    body = bytearray(struct.pack(HEADER_FMT, PROTO_VERSION, frame_type,
                                 grid_width, seq, timestamp_ms))
    for i in range(n_zones):
        body += struct.pack('<hHB', int(distances[i]), int(sigmas[i]),
                            int(statuses[i]))
    body += struct.pack('<H', crc16_ccitt(body))
    return cobs_encode(bytes(body)) + b'\x00'
 
 
def decode_frame(raw):
    """Parse one COBS-decoded frame body.

    Returns a dict, or None if the frame is malformed. Never raises --
    corrupt input is expected on a serial link, not exceptional.
    """
    if len(raw) < HEADER_LEN + CRC_LEN:
        return None

    version, ftype, grid_width, seq, ts = struct.unpack_from(HEADER_FMT, raw, 0)

    if version != PROTO_VERSION:
        return None
    if grid_width not in (4, 8):
        return None
    if ftype not in (TYPE_RANGING, TYPE_RANGING_EXT):
        return None

    n_zones = grid_width * grid_width
    record_len = RECORD_LEN_EXT if ftype == TYPE_RANGING_EXT else RECORD_LEN
    expected = HEADER_LEN + n_zones * record_len + CRC_LEN
    if len(raw) != expected:
        return None

    got = struct.unpack_from('<H', raw, expected - CRC_LEN)[0]
    if crc16_ccitt(raw[:expected - CRC_LEN]) != got:
        return None

    result = {
        'type': ftype,
        'seq': seq,
        'timestamp_ms': ts,
        'grid_width': grid_width,
    }

    if ftype == TYPE_RANGING_EXT:
        # One unpack for all zones: distance, sigma, status, signal, ambient.
        flat = struct.unpack_from('<' + 'hHBHH' * n_zones, raw, HEADER_LEN)
        result['distances'] = flat[0::5]
        result['sigmas'] = flat[1::5]
        result['statuses'] = flat[2::5]
        result['signals'] = flat[3::5]
        result['ambients'] = flat[4::5]
    else:
        # One unpack for all zones; far faster and less error-prone than
        # slicing each record individually.
        flat = struct.unpack_from('<' + 'hHB' * n_zones, raw, HEADER_LEN)
        result['distances'] = flat[0::3]
        result['sigmas'] = flat[1::3]
        result['statuses'] = flat[2::3]

    return result 
# --------------------------------------------------------------------------
# Stream reassembly
# --------------------------------------------------------------------------
class FrameExtractor:
    """Accumulates arbitrary byte chunks and yields complete frames.
 
    Serial reads return whatever happens to be buffered: half a frame,
    three frames, one byte. This holds the leftovers between calls.
    """
 
    def __init__(self):
        self.buf = bytearray()
        self.n_ok = 0
        self.n_bad = 0
        self.synced = False       # discard bytes until the first delimiter
 
    def feed(self, chunk):
        """Append bytes; return a list of successfully decoded frames."""
        self.buf += chunk
        frames = []
 
        while True:
            z = self.buf.find(0)
            if z < 0:
                break
 
            packet = bytes(self.buf[:z])
            del self.buf[:z + 1]
 
            # Anything before the very first delimiter is the tail of a
            # frame that was already in flight when we connected.
            if not self.synced:
                self.synced = True
                continue
 
            if not packet:
                continue
 
            try:
                body = cobs_decode(packet)
            except ValueError:
                self.n_bad += 1
                continue
 
            frame = decode_frame(body)
            if frame is None:
                self.n_bad += 1
                continue
 
            self.n_ok += 1
            frames.append(frame)
 
        # No delimiter in a buffer this large means we are desynchronised
        # or at the wrong baud rate. Drop it rather than growing forever.
        if len(self.buf) > MAX_FRAME:
            self.buf.clear()
            self.n_bad += 1
 
        return frames
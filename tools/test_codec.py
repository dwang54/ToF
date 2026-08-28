import os, random, struct
from frame_codec import *

fail = 0
def check(name, cond):
    global fail
    print(("  PASS  " if cond else "  FAIL  ") + name)
    if not cond: fail += 1

print("CRC")
check("published check vector 0x29B1", crc16_ccitt(b"123456789") == 0x29B1)

print("\nCOBS edge cases")
for name, data in [
    ("empty",                b""),
    ("single zero",          b"\x00"),
    ("all zeros",            b"\x00" * 10),
    ("no zeros",             bytes(range(1, 100))),
    ("exactly 254 non-zero", bytes([0xAB]) * 254),
    ("254 non-zero + zero",  bytes([0xAB]) * 254 + b"\x00"),
    ("255 non-zero",         bytes([0xAB]) * 255),
    ("508 non-zero",         bytes([0xAB]) * 508),
    ("zeros around run",     b"\x00" + bytes([0xCD]) * 300 + b"\x00\x00"),
]:
    enc = cobs_encode(data)
    check(f"{name}: round-trip", cobs_decode(enc) == data)
    check(f"{name}: no 0x00 in output", 0 not in enc)

print("\nCOBS fuzz (2000 random buffers)")
random.seed(1)
bad = 0
for _ in range(2000):
    n = random.randint(0, 700)
    d = bytes(random.choice([0, 0, random.randint(0, 255)]) for _ in range(n))
    if cobs_decode(cobs_encode(d)) != d or 0 in cobs_encode(d):
        bad += 1
check("all 2000 round-trip with no embedded zeros", bad == 0)

print("\nFrame round-trip")
n = 64
dist = [1000 + i for i in range(n)]          # distinct per zone: catches stride bugs
sig  = [3 + (i % 7) for i in range(n)]
stat = [5 if i % 3 else 9 for i in range(n)]
dist[0] = -1        # sign extension
dist[1] = 0x7FFF    # boundary
dist[2] = 0x0102    # byte order

wire = build_frame(42, 123456, 8, dist, sig, stat)
print(f"  on-wire size: {len(wire)} bytes (333 body + COBS + delimiter)")

ex = FrameExtractor()
ex.feed(b"\x00")                              # establish sync
f = ex.feed(wire)
check("one frame decoded", len(f) == 1)
if f:
    g = f[0]
    check("seq",        g['seq'] == 42)
    check("timestamp",  g['timestamp_ms'] == 123456)
    check("grid width", g['grid_width'] == 8)
    check("distances",  list(g['distances']) == dist)
    check("sigmas",     list(g['sigmas']) == sig)
    check("statuses",   list(g['statuses']) == stat)
    check("negative distance survives", g['distances'][0] == -1)
    check("0x0102 not byte-swapped",    g['distances'][2] == 0x0102)

print("\n4x4 works unchanged")
w4 = build_frame(1, 99, 4, [500+i for i in range(16)], [4]*16, [5]*16)
ex2 = FrameExtractor(); ex2.feed(b"\x00")
f4 = ex2.feed(w4)
check("4x4 decodes", len(f4) == 1 and f4[0]['grid_width'] == 4)
check("4x4 zone count", len(f4[0]['distances']) == 16)

print("\nStream robustness")
ex3 = FrameExtractor(); ex3.feed(b"\x00")
# frames split across arbitrary read boundaries
blob = b"".join(build_frame(i, i*10, 8, dist, sig, stat) for i in range(5))
got = []
i = 0
random.seed(7)
while i < len(blob):
    k = random.randint(1, 97)
    got += ex3.feed(blob[i:i+k])
    i += k
check("5 frames survive random chunking", len(got) == 5)
check("sequence numbers intact", [g['seq'] for g in got] == list(range(5)))

ex4 = FrameExtractor(); ex4.feed(b"\x00")
good = build_frame(1, 1, 8, dist, sig, stat)
corrupt = bytearray(build_frame(2, 2, 8, dist, sig, stat))
corrupt[40] ^= 0xFF                            # flip a bit in the payload
out = ex4.feed(good + bytes(corrupt) + good)
check("corrupt frame rejected, neighbours survive", len(out) == 2)
check("bad counter incremented", ex4.n_bad == 1)

ex5 = FrameExtractor(); ex5.feed(b"\x00")
out = ex5.feed(b"\x11\x22\x33\x00" + good)     # garbage before a good frame
check("leading garbage discarded, next frame ok", len(out) == 1)

ex6 = FrameExtractor(); ex6.feed(b"\x00")
truncated = good[:100]
out = ex6.feed(truncated + good)
check("truncated frame costs 2 frames then recovers", len(out) == 0)

ex7 = FrameExtractor(); ex7.feed(b"\x00")
out = ex7.feed(bytes([0xAA]) * 9000)           # wrong baud: never a delimiter
check("buffer guard fires, no unbounded growth", len(ex7.buf) <= MAX_FRAME)

print("\n" + ("ALL PASS" if fail == 0 else f"{fail} FAILURES"))
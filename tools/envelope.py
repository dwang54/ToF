"""
Machine-envelope calibration + intrusion watch for the VL53L7CX stream.

Two modes:

  CALIBRATE -- run while the machine exercises its FULL range of motion
  (all rotations, all in/out excursions, fault-retract moves, everything)
  with NO person in the scene. Records, per zone, the band of distances
  the machine (plus static background) can legitimately produce:

      py -3.11 tools/envelope.py calibrate COM3 --seconds 120 --out tools/envelope.json

  WATCH -- live intrusion detector. A zone is "suspicious" when its
  reading falls OUTSIDE its calibrated band (closer than the machine can
  ever reach, or at a depth the machine never produces). An alarm fires
  only when suspicious zones form a blob of adjacent zones AND persist
  across several frames -- one flickering pixel is noise, an arm is not:

      py -3.11 tools/envelope.py watch COM3 --envelope tools/envelope.json

Shared flags: --baud (default 115200), --mask (zone_mask.json, loaded
automatically if present; masked zones are ignored in both modes).

The envelope JSON stores min/max per zone plus sample counts, so you can
re-run calibrate with --merge to extend an existing envelope after the
machine gains new moves, instead of starting over.
"""

import argparse
import json
import os
import sys
import time
from collections import deque

import serial

from frame_codec import FrameExtractor

# Statuses we trust enough to learn from / act on.
VALID_STATUS = (5, 9)
SIGMA_MAX_MM = 15

# --- watch-mode tuning ------------------------------------------------------
MARGIN_MM = 60        # band is widened by this much before comparing
BLOB_MIN_ZONES = 2    # intrusion must span at least this many adjacent zones
PERSIST_M = 3         # ... in at least M of the last N frames
PERSIST_N = 5


def load_mask(path):
    """Returns the set of excluded zone indices from zone_map.py's JSON."""
    if path and os.path.exists(path):
        with open(path) as f:
            excluded = set(json.load(f).get('excluded', []))
        print(f'mask: ignoring {len(excluded)} zones from {path}')
        return excluded
    return set()


def frames_from_serial(port, baud, seconds=None):
    """Generator yielding decoded frames until Ctrl+C or the time limit."""
    ser = serial.Serial(port, baud, timeout=0.1)
    extractor = FrameExtractor()
    t0 = time.monotonic()
    try:
        while True:
            if seconds is not None and time.monotonic() - t0 >= seconds:
                return
            chunk = ser.read(4096)
            if not chunk:
                continue
            for frame in extractor.feed(chunk):
                yield frame
    except KeyboardInterrupt:
        return
    finally:
        ser.close()


# -----------------------------------------------------------------------------
# calibrate
# -----------------------------------------------------------------------------

def run_calibrate(args):
    excluded = load_mask(args.mask)

    # Start from an existing envelope when --merge is given.
    env = None
    if args.merge and os.path.exists(args.out):
        with open(args.out) as f:
            env = json.load(f)
        print(f'merging into existing envelope {args.out}')

    n_frames = 0
    print(f'calibrating from {args.port} @ {args.baud}'
          + (f' for {args.seconds}s' if args.seconds else ' (Ctrl+C to stop)'))
    print('>>> exercise the machine through its FULL range of motion now <<<')

    for frame in frames_from_serial(args.port, args.baud, args.seconds):
        gw = frame['grid_width']
        n_zones = gw * gw

        if env is None:
            env = {
                'grid_width': gw,
                'min_mm': [None] * n_zones,
                'max_mm': [None] * n_zones,
                'samples': [0] * n_zones,
            }

        for i in range(n_zones):
            if i in excluded:
                continue
            if frame['statuses'][i] not in VALID_STATUS:
                continue
            if frame['sigmas'][i] > SIGMA_MAX_MM:
                continue
            d = frame['distances'][i]
            if env['min_mm'][i] is None or d < env['min_mm'][i]:
                env['min_mm'][i] = d
            if env['max_mm'][i] is None or d > env['max_mm'][i]:
                env['max_mm'][i] = d
            env['samples'][i] += 1

        n_frames += 1
        if n_frames % 15 == 0:
            learned = sum(1 for s in env['samples'] if s > 0)
            print(f'\rframes={n_frames}  zones learned={learned}/{n_zones}', end='')

    if env is None:
        sys.exit('no frames received -- check port/baud')

    env['excluded'] = sorted(excluded)
    with open(args.out, 'w') as f:
        json.dump(env, f, indent=2)

    learned = sum(1 for s in env['samples'] if s > 0)
    thin = [i for i, s in enumerate(env['samples'])
            if 0 < s < 30 and i not in excluded]
    print(f'\nsaved {args.out}: {learned}/{len(env["samples"])} zones learned, '
          f'{n_frames} frames')
    if thin:
        print(f'warning: zones {thin} have <30 samples -- '
              'their bands may be unreliable; run longer or use --merge')


# -----------------------------------------------------------------------------
# watch
# -----------------------------------------------------------------------------

def blobs(suspicious, gw):
    """Group suspicious zone indices into 4-connected blobs (flood fill)."""
    remaining = set(suspicious)
    out = []
    while remaining:
        stack = [remaining.pop()]
        blob = set(stack)
        while stack:
            z = stack.pop()
            r, c = divmod(z, gw)
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= nr < gw and 0 <= nc < gw:
                    nz = nr * gw + nc
                    if nz in remaining:
                        remaining.remove(nz)
                        blob.add(nz)
                        stack.append(nz)
        out.append(blob)
    return out


def run_watch(args):
    with open(args.envelope) as f:
        env = json.load(f)
    gw = env['grid_width']
    excluded = set(env.get('excluded', [])) | load_mask(args.mask)

    history = deque(maxlen=PERSIST_N)   # one set of suspicious zones per frame
    alarmed = False

    print(f'watching {args.port} @ {args.baud} against {args.envelope} '
          f'(margin {MARGIN_MM} mm, blob>={BLOB_MIN_ZONES}, '
          f'{PERSIST_M}/{PERSIST_N} frames). Ctrl+C to stop.')

    for frame in frames_from_serial(args.port, args.baud):
        suspicious = set()
        for i in range(gw * gw):
            if i in excluded or env['samples'][i] == 0:
                continue
            if frame['statuses'][i] not in VALID_STATUS:
                continue
            if frame['sigmas'][i] > SIGMA_MAX_MM:
                continue
            d = frame['distances'][i]
            lo = env['min_mm'][i] - MARGIN_MM
            hi = env['max_mm'][i] + MARGIN_MM
            if d < lo or d > hi:        # a depth the machine never produces
                suspicious.add(i)

        history.append(suspicious)

        # Persistence: keep only zones suspicious in >= M of the last N frames.
        persistent = {z for z in suspicious
                      if sum(z in past for past in history) >= PERSIST_M}

        # Spatial coherence: require a blob of adjacent persistent zones.
        hits = [b for b in blobs(persistent, gw) if len(b) >= BLOB_MIN_ZONES]

        if hits:
            if not alarmed:
                alarmed = True
                zones = sorted(z for b in hits for z in b)
                dists = {z: frame['distances'][z] for z in zones}
                print(f'\nALARM seq {frame["seq"]}: intrusion in zones {zones} '
                      f'(mm: {dists})')
        else:
            if alarmed:
                print(f'clear at seq {frame["seq"]}')
            alarmed = False

        print(f'\rseq {frame["seq"]}  suspicious={len(suspicious)} '
              f'persistent={len(persistent)} '
              f'{"*** ALARM ***" if alarmed else "clear":>14}', end='')


# -----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    default_mask = os.path.join(os.path.dirname(__file__), 'zone_mask.json')

    cal = sub.add_parser('calibrate', help='record the machine envelope')
    cal.add_argument('port')
    cal.add_argument('--baud', type=int, default=115200)
    cal.add_argument('--seconds', type=float, default=None,
                     help='stop after this long (default: run until Ctrl+C)')
    cal.add_argument('--out', default=os.path.join(os.path.dirname(__file__),
                                                   'envelope.json'))
    cal.add_argument('--merge', action='store_true',
                     help='extend an existing envelope instead of replacing it')
    cal.add_argument('--mask', default=default_mask)
    cal.set_defaults(fn=run_calibrate)

    wat = sub.add_parser('watch', help='live intrusion detection')
    wat.add_argument('port')
    wat.add_argument('--baud', type=int, default=115200)
    wat.add_argument('--envelope', default=os.path.join(os.path.dirname(__file__),
                                                        'envelope.json'))
    wat.add_argument('--mask', default=default_mask)
    wat.set_defaults(fn=run_watch)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()

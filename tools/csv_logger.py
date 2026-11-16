"""
Logs decoded VL53L7CX ranging frames straight to a CSV file for analysis
in Excel / pandas.

One row per zone per frame:
    seq, timestamp_ms, zone, row, col,
    distance_mm, sigma_mm, status, signal_kcps, ambient_kcps

signal_kcps / ambient_kcps are only populated if the firmware is sending
FRAME_TYPE_RANGING_EXT frames (frame_send_ranging_ext in frame_encoder.c).
If it's sending plain FRAME_TYPE_RANGING frames instead, those two columns
are left blank.

Usage:
    py -3.11 tools/csv_logger.py COM3 --baud 115200 --out run1.csv
    py -3.11 tools/csv_logger.py COM3 --out run1.csv --seconds 30
    py -3.11 tools/csv_logger.py COM3 --out run1.csv --center-only

Stop with Ctrl+C -- the CSV is flushed after every frame so you won't
lose data if you kill it.
"""

import argparse
import csv
import sys
import time

import serial

from frame_codec import FrameExtractor, TYPE_RANGING_EXT

FIELDS = [
    'seq', 'timestamp_ms', 'zone', 'row', 'col',
    'distance_mm', 'sigma_mm', 'status', 'signal_kcps', 'ambient_kcps',
]


def center_zones(grid_width, half_width=2):
    """Zone indices within `half_width` of the grid center, for --center-only."""
    lo = grid_width // 2 - half_width
    hi = grid_width // 2 + half_width
    zones = set()
    for r in range(grid_width):
        for c in range(grid_width):
            if lo <= r < hi and lo <= c < hi:
                zones.add(r * grid_width + c)
    return zones


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', help='Serial port, e.g. COM3')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--out', default='tof_log.csv', help='Output CSV path')
    ap.add_argument('--seconds', type=float, default=None,
                    help='Stop automatically after this many seconds')
    ap.add_argument('--center-only', action='store_true',
                    help='Only log zones near the grid center (still detects '
                         'grid_width from the stream; use for quick range tests)')
    ap.add_argument('--half-width', type=int, default=2,
                    help='Half-width of the center region for --center-only '
                         '(default 2 -> 4x4 center of an 8x8 grid)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    extractor = FrameExtractor()

    t_start = time.monotonic()
    n_rows = 0
    n_frames = 0

    print(f"Logging {args.port} @ {args.baud} -> {args.out}  (Ctrl+C to stop)")

    with open(args.out, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)

        try:
            while True:
                if args.seconds is not None and (time.monotonic() - t_start) >= args.seconds:
                    break

                chunk = ser.read(4096)
                if not chunk:
                    continue

                for frame in extractor.feed(chunk):
                    n_frames += 1
                    gw = frame['grid_width']
                    zones = center_zones(gw, args.half_width) if args.center_only else None

                    has_ext = frame['type'] == TYPE_RANGING_EXT
                    signals = frame.get('signals')
                    ambients = frame.get('ambients')

                    for i in range(gw * gw):
                        if zones is not None and i not in zones:
                            continue

                        row = i // gw
                        col = i % gw
                        writer.writerow([
                            frame['seq'],
                            frame['timestamp_ms'],
                            i, row, col,
                            frame['distances'][i],
                            frame['sigmas'][i],
                            frame['statuses'][i],
                            signals[i] if has_ext else '',
                            ambients[i] if has_ext else '',
                        ])
                        n_rows += 1

                    f.flush()

                    if n_frames % 15 == 0:
                        print(f"\rframes={n_frames} rows={n_rows} "
                             f"ok={extractor.n_ok} bad={extractor.n_bad}", end='')

        except KeyboardInterrupt:
            pass

    print(f"\nDone. {n_frames} frames, {n_rows} rows -> {args.out}")


if __name__ == '__main__':
    sys.exit(main())

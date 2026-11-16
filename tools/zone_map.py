"""
Live VL53L7CX zone-mapping tool.

Shows the 8x8 (or 4x4) zone grid as a live heatmap, labeled with each
zone's index so you can physically map which zone sees which part of
your scene: wave a hand / hold a target around the FOV and watch which
cell responds.

Orientation convention (matches cloud.py, verified empirically):
    row 0 = TOP of the sensor's field of view
    col 0 = sensor's LEFT -- which appears on YOUR RIGHT when you are
            standing in front of and facing the sensor.

Exclusion mask workflow:
    LEFT-CLICK any cell to toggle it excluded/included. Excluded cells
    show a red X. Press S to save the current exclusion list to
    zone_mask.json (next to this script). Press L to reload it.
    The JSON is just {"grid_width": 8, "excluded": [0, 1, 8, ...]} --
    easy for cloud.py or the firmware to consume later.

Usage:
    py -3.11 tools/zone_map.py COM3 --baud 115200
    py -3.11 tools/zone_map.py COM3 --mask my_mask.json

Keys / mouse:
    left-click cell  toggle zone excluded
    S                save exclusion mask to JSON
    L                load exclusion mask from JSON
    C                clear all exclusions
    Q / close window quit
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import serial
import matplotlib
import matplotlib.patheffects
import matplotlib.pyplot as plt

from frame_codec import FrameExtractor

VALID_STATUS = (5, 9)

# Colormap range for the distance heatmap. Tune to your scene; readings
# outside this just saturate the color scale, they are not dropped.
CMAP_MIN_MM = 0
CMAP_MAX_MM = 3000


class ZoneMapView:
    def __init__(self, mask_path):
        self.mask_path = mask_path
        self.grid_width = 8                  # corrected from stream on first frame
        self.excluded = set()
        self.quit = False

        self.fig, self.ax = plt.subplots(figsize=(9, 8))
        self.fig.canvas.manager.set_window_title('VL53L7CX zone map')

        self.img = self.ax.imshow(
            np.zeros((8, 8)), cmap='turbo',
            vmin=CMAP_MIN_MM, vmax=CMAP_MAX_MM, origin='upper')
        self.cbar = self.fig.colorbar(self.img, ax=self.ax, label='distance_mm')

        self.ax.set_xlabel("col  (0 = sensor's left = YOUR right when facing it)")
        self.ax.set_ylabel('row  (0 = top of FOV)')
        self.title = self.ax.set_title('waiting for frames...')

        # One text artist per cell, created lazily once grid width is known.
        self.cell_texts = []
        self.x_marks = []

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('close_event', lambda e: self._set_quit())

        plt.ion()
        plt.show()

    def _set_quit(self):
        self.quit = True

    # -- mask persistence ---------------------------------------------------
    def save_mask(self):
        data = {'grid_width': self.grid_width,
                'excluded': sorted(self.excluded)}
        with open(self.mask_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f'saved {len(self.excluded)} excluded zones -> {self.mask_path}')

    def load_mask(self):
        if not os.path.exists(self.mask_path):
            print(f'no mask file at {self.mask_path}')
            return
        with open(self.mask_path) as f:
            data = json.load(f)
        self.excluded = set(data.get('excluded', []))
        print(f'loaded {len(self.excluded)} excluded zones from {self.mask_path}')

    # -- interaction --------------------------------------------------------
    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        col = int(round(event.xdata))
        row = int(round(event.ydata))
        g = self.grid_width
        if not (0 <= row < g and 0 <= col < g):
            return
        zone = row * g + col
        if zone in self.excluded:
            self.excluded.discard(zone)
            print(f'zone {zone} (r{row},c{col}) INCLUDED')
        else:
            self.excluded.add(zone)
            print(f'zone {zone} (r{row},c{col}) EXCLUDED')

    def _on_key(self, event):
        k = (event.key or '').lower()
        if k == 's':
            self.save_mask()
        elif k == 'l':
            self.load_mask()
        elif k == 'c':
            self.excluded.clear()
            print('cleared all exclusions')
        elif k == 'q':
            self._set_quit()

    # -- per-frame draw -------------------------------------------------------
    def _ensure_cells(self, g):
        """(Re)create the per-cell text labels when grid width changes."""
        if self.cell_texts and self.grid_width == g:
            return
        self.grid_width = g
        for t in self.cell_texts:
            t.remove()
        self.cell_texts = []
        self.img.set_extent((-0.5, g - 0.5, g - 0.5, -0.5))
        self.ax.set_xticks(range(g))
        self.ax.set_yticks(range(g))
        for r in range(g):
            for c in range(g):
                zone = r * g + c
                # Zone index on top, live distance below (updated each frame)
                t = self.ax.text(c, r, f'{zone}\n--', ha='center', va='center',
                                 fontsize=8, color='white',
                                 path_effects=[
                                     matplotlib.patheffects.withStroke(
                                         linewidth=2, foreground='black')])
                self.cell_texts.append(t)

    def update(self, frame):
        g = frame['grid_width']
        self._ensure_cells(g)

        dist = np.asarray(frame['distances'], dtype=float).reshape(g, g)
        status = np.asarray(frame['statuses']).reshape(g, g)

        # Gray out zones with invalid status by masking them out of the
        # heatmap (NaN renders as the "bad" color).
        shown = dist.copy()
        shown[~np.isin(status, VALID_STATUS)] = np.nan

        self.img.set_data(shown)

        for r in range(g):
            for c in range(g):
                zone = r * g + c
                t = self.cell_texts[zone]
                if zone in self.excluded:
                    t.set_text(f'{zone}\nX')
                    t.set_color('red')
                elif np.isnan(shown[r, c]):
                    t.set_text(f'{zone}\nst{status[r, c]}')
                    t.set_color('gray')
                else:
                    t.set_text(f'{zone}\n{int(dist[r, c])}')
                    t.set_color('white')

        self.title.set_text(
            f"seq {frame['seq']}   {g}x{g}   excluded: {len(self.excluded)}"
            f"   (click=toggle, S=save, L=load, C=clear, Q=quit)")

    def pump(self):
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', help='Serial port, e.g. COM3')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--mask', default=os.path.join(os.path.dirname(__file__),
                                                   'zone_mask.json'),
                    help='Path of the exclusion-mask JSON (default: tools/zone_mask.json)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ex = FrameExtractor()
    view = ZoneMapView(args.mask)

    print(f'{args.port} @ {args.baud}. Wave a target around the FOV and watch '
          f'which cells respond.')

    last_note = time.time()
    try:
        while not view.quit:
            data = ser.read(8192)
            if data:
                frames = ex.feed(data)
                if frames:
                    view.update(frames[-1])   # newest frame only; skip backlog
            else:
                if time.time() - last_note > 3.0 and ex.n_ok == 0:
                    print(f'no frames yet (bad: {ex.n_bad}) -- wrong baud if '
                          f'bad is climbing, dead link if both are 0')
                    last_note = time.time()
            view.pump()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f'{ex.n_ok} frames decoded, {ex.n_bad} rejected')


if __name__ == '__main__':
    sys.exit(main())

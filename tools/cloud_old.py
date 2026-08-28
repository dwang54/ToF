#!/usr/bin/env python3
"""
VL53L7CX live point cloud viewer (Open3D).

Converts each frame's 64 radial distances into 3D points and renders them
live. Optionally fits the dominant plane (wall/floor) with RANSAC, removes
it, and clusters whatever is left into objects with bounding boxes.

Coordinate convention:
    x = lateral   (right positive)
    y = vertical  (up positive)
    z = depth     (away from sensor)
Units are METRES -- Open3D's default camera and point sizes assume metres,
so millimetres from the sensor are divided by 1000 on the way in.

Wire format, one line per frame:
    F,<seq>,<n>,<temp_c>,<d>:<status>:<sigma>,...

Keys (in the render window):
    P   toggle plane removal
    C   toggle clustering
    Q   quit

Usage:
    python cloud.py COM7
    python cloud.py COM7 --baud 115200
    python cloud.py --replay run.csv
"""

import argparse
import math
import sys
import time

import numpy as np
import open3d as o3d

# --- sensor / filtering constants -------------------------------------------

FOV_DEG = 60.0
VALID_STATUS = (5, 9)
SIGMA_MAX_MM = 15

# RANSAC: how far a point can sit from the plane and still count as on it.
# Generous, because 64 points is a sparse fit and the sensor has real noise.
PLANE_TOL_M = 0.040

# DBSCAN: at 1 m, adjacent zones are ~130 mm apart, so eps must exceed that
# or every point is its own cluster. min_points 3 kills single-zone noise.
CLUSTER_EPS_M = 0.25
CLUSTER_MIN_PTS = 3


def zone_angles(n):
    """Centre angle of each zone along one axis, in radians."""
    step = FOV_DEG / n
    return np.array([math.radians(-FOV_DEG / 2 + (i + 0.5) * step)
                     for i in range(n)])


def to_points(dist_mm, valid_mask, n):
    """Radial distances -> (M,3) array of XYZ points in metres.

    The sensor measures along each zone's line of sight, so a flat wall reads
    farther at the corners. Dividing by sqrt(1 + tx^2 + ty^2) recovers true
    perpendicular depth and the wall becomes flat.
    """
    ang = zone_angles(n)
    tx = np.tan(ang)[None, :]
    ty = np.tan(ang)[:, None]

    d = dist_mm / 1000.0
    z = d / np.sqrt(1.0 + tx ** 2 + ty ** 2)
    x = z * tx
    y = -z * ty          # row 0 is the top of the view, so flip to y-up

    pts = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)
    return pts[valid_mask.ravel()]


def depth_colors(pts, near=0.1, far=3.5):
    """Blue (far) to red (near). Reads instantly without rotating."""
    if len(pts) == 0:
        return np.zeros((0, 3))
    t = np.clip((pts[:, 2] - near) / (far - near), 0.0, 1.0)
    return np.stack([1.0 - t, np.zeros_like(t) + 0.25, t], axis=1)


def parse_frame(line):
    """Return (seq, n, temp, dist, status, sigma) or None."""
    parts = line.strip().split(',')
    if len(parts) < 5 or parts[0] != 'F':
        return None
    try:
        seq, n, temp = int(parts[1]), int(parts[2]), int(parts[3])
        zones = parts[4:]
        if len(zones) != n * n:
            return None

        d = np.zeros(n * n)
        s = np.zeros(n * n, dtype=int)
        g = np.zeros(n * n, dtype=int)
        for i, z in enumerate(zones):
            a, b, c = z.split(':')
            d[i], s[i], g[i] = float(a), int(b), int(c)

        return seq, n, temp, d.reshape(n, n), s.reshape(n, n), g.reshape(n, n)
    except (ValueError, IndexError):
        return None


class CloudViewer:
    def __init__(self, do_plane=False, do_cluster=False):
        self.do_plane = do_plane
        self.do_cluster = do_cluster
        self.boxes = []
        self.first = True

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window('VL53L7CX point cloud', width=1100, height=800)

        opt = self.vis.get_render_option()
        opt.point_size = 9.0
        opt.background_color = np.array([0.08, 0.08, 0.09])

        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        # Sensor origin marker, 20 cm axes. Without a reference, floating
        # dots are very hard to interpret.
        self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.vis.add_geometry(self.frame)

        self.vis.register_key_callback(ord('P'), self._toggle_plane)
        self.vis.register_key_callback(ord('C'), self._toggle_cluster)
        self.vis.register_key_callback(ord('Q'), lambda v: v.close() or False)

    def _toggle_plane(self, _vis):
        self.do_plane = not self.do_plane
        print(f'plane removal: {self.do_plane}')
        return False

    def _toggle_cluster(self, _vis):
        self.do_cluster = not self.do_cluster
        print(f'clustering: {self.do_cluster}')
        return False

    def _clear_boxes(self):
        for b in self.boxes:
            self.vis.remove_geometry(b, reset_bounding_box=False)
        self.boxes.clear()

    def _add_box(self, cloud, color):
        box = cloud.get_axis_aligned_bounding_box()
        box.color = color
        self.boxes.append(box)
        self.vis.add_geometry(box, reset_bounding_box=False)

    def update(self, frame):
        seq, n, temp, dist, status, sigma = frame

        ok = np.isin(status, VALID_STATUS) & (sigma <= SIGMA_MAX_MM)
        pts = to_points(dist, ok, n)

        self._clear_boxes()

        if len(pts) == 0:
            self.pcd.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self._render()
            return

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)

        plane_pts = np.zeros((0, 3))

        # --- dominant plane (wall, floor, table) -------------------------
        if self.do_plane and len(pts) >= 8:
            model, inliers = cloud.segment_plane(
                distance_threshold=PLANE_TOL_M,
                ransac_n=3,
                num_iterations=200)
            if len(inliers) >= 6:
                plane_pts = pts[inliers]
                keep = np.setdiff1d(np.arange(len(pts)), inliers)
                pts = pts[keep]
                a, b, c, d = model
                if seq % 20 == 0:
                    print(f'plane: {len(inliers)} pts, '
                          f'normal ({a:.2f},{b:.2f},{c:.2f}), '
                          f'offset {-d:.2f} m')

        cols = depth_colors(pts)

        # --- cluster whatever is left into objects -----------------------
        if self.do_cluster and len(pts) >= CLUSTER_MIN_PTS:
            rest = o3d.geometry.PointCloud()
            rest.points = o3d.utility.Vector3dVector(pts)
            labels = np.array(rest.cluster_dbscan(
                eps=CLUSTER_EPS_M,
                min_points=CLUSTER_MIN_PTS,
                print_progress=False))

            palette = np.array([[1.0, 0.85, 0.2], [0.3, 0.9, 0.5],
                                [0.9, 0.4, 0.9], [0.4, 0.7, 1.0]])
            for k in range(labels.max() + 1):
                idx = np.where(labels == k)[0]
                colour = palette[k % len(palette)]
                cols[idx] = colour

                sub = o3d.geometry.PointCloud()
                sub.points = o3d.utility.Vector3dVector(pts[idx])
                self._add_box(sub, colour)

            cols[labels < 0] = [0.35, 0.35, 0.35]      # DBSCAN noise
            if seq % 20 == 0 and labels.max() >= 0:
                near = pts[:, 2].min()
                print(f'{labels.max() + 1} object(s), nearest {near:.2f} m')

        # Plane points render dim so they recede behind the objects.
        if len(plane_pts):
            pts = np.vstack([pts, plane_pts])
            cols = np.vstack([cols, np.tile([0.22, 0.22, 0.25],
                                            (len(plane_pts), 1))])

        self.pcd.points = o3d.utility.Vector3dVector(pts)
        self.pcd.colors = o3d.utility.Vector3dVector(cols)
        self._render()

    def _render(self):
        self.vis.update_geometry(self.pcd)
        if self.first:
            ctr = self.vis.get_view_control()
            ctr.set_front([0.0, -0.3, -1.0])
            ctr.set_up([0.0, 1.0, 0.0])
            ctr.set_lookat([0.0, 0.0, 1.2])
            ctr.set_zoom(0.6)
            self.first = False
        self.vis.poll_events()
        self.vis.update_renderer()

    def alive(self):
        return self.vis.poll_events()

    def close(self):
        self.vis.destroy_window()


def run_serial(port, baud, plane, cluster, logfile):
    import serial
    ser = serial.Serial(port, baud, timeout=0)     # non-blocking
    view = CloudViewer(plane, cluster)
    log = open(logfile, 'w') if logfile else None
    buf = b''
    n_frames = 0
    last_report = time.time()
    print('connected. P = plane removal, C = clustering, Q = quit')

    try:
        while view.alive():
            view.vis.update_renderer()             # keep the window alive

            data = ser.read(8192)

            # if data:
            #     print('raw:', repr(data[:120]))

            if not data:
                time.sleep(0.005)
                if time.time() - last_report > 3.0:
                    print(f'{n_frames} frames so far')
                    last_report = time.time()
                continue

            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='ignore')
                frame = parse_frame(text)
                if frame is None:
                    continue
                if log:
                    log.write(text + '\n')
                n_frames += 1
                view.update(frame)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log:
            log.close()
        view.close()


def run_replay(path, plane, cluster, delay):
    view = CloudViewer(plane, cluster)
    with open(path) as f:
        lines = f.readlines()
    try:
        while True:                       # loop the recording
            for raw in lines:
                if not view.alive():
                    return
                frame = parse_frame(raw)
                if frame:
                    view.update(frame)
                    end = time.time() + delay
                    while time.time() < end:
                        if not view.alive():
                            return
                        time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        view.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', help='serial port, e.g. COM7')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--plane', action='store_true', help='remove dominant plane')
    ap.add_argument('--cluster', action='store_true', help='cluster objects')
    ap.add_argument('--log', help='record frames to this file')
    ap.add_argument('--replay', help='play back a recording instead')
    ap.add_argument('--delay', type=float, default=0.1)
    args = ap.parse_args()

    if args.replay:
        run_replay(args.replay, args.plane, args.cluster, args.delay)
    elif args.port:
        run_serial(args.port, args.baud, args.plane, args.cluster, args.log)
    else:
        sys.exit('need a port, or --replay <file>')
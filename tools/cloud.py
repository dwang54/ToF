"""
VL53L7CX live point cloud viewer.

Reads COBS-framed binary frames from the Nucleo, converts each zone's radial
distance into a 3D point, and renders the result live in Open3D.

Usage:
    py -3.11 tools/cloud.py COM3 --baud 115200
    py -3.11 tools/cloud.py COM3 --log run.bin --plane --cluster
    py -3.11 tools/cloud.py --replay run.bin --loop
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import open3d as o3d

from frame_codec import FrameExtractor

# -----------------------------------------------------------------------------
# SECTION 1 -- constants
# -----------------------------------------------------------------------------

SENSOR_FOV = 60.0          # degrees, per axis (the 90 deg figure is diagonal)

VALID_STATUS = (5, 9)      # 5 = valid, 9 = valid but possibly merged targets
SIGMA_MAX_MM = 15          # reject zones the sensor is unsure about
# >>> CHANGE MINIMUM DETECTION DISTANCE HERE <<<
# Zones closer than this are ignored. mm = inches * 25.4  (8 in = 203.2 mm)
MIN_DIST_MM = 203          # 8 inches
MAX_DIST_MM = 1050         # tighten to your scene; strays drag the camera out

# RANSAC. 64 points is a sparse fit, so keep the tolerance generous --
# too tight and it latches onto spurious subsets instead of the real plane.
PLANE_TOL_M = 0.040
PLANE_MIN_PTS = 6

# DBSCAN. eps must exceed the spacing between adjacent zones, which GROWS
# with distance: ~130 mm at 1 m but ~390 mm at 3 m. A value tuned close up
# will fragment distant objects. See the note in cluster_points().
CLUSTER_EPS_M = 0.25
CLUSTER_MIN_PTS = 3

RAY_COLOR = (0.30, 0.30, 0.35)     # dim: 64 bright lines drown the points

# Zones to ignore entirely, loaded from zone_map.py's JSON (see --mask).
# Empty set = no exclusions.
EXCLUDED_ZONES = set()


def load_zone_mask(path):
    """Load zone_map.py's exclusion JSON into EXCLUDED_ZONES."""
    global EXCLUDED_ZONES
    with open(path) as f:
        data = json.load(f)
    EXCLUDED_ZONES = set(data.get('excluded', []))
    print(f'zone mask: excluding {len(EXCLUDED_ZONES)} zones from {path}')


def zone_angles(g_width):
    """
    Returns a list of the centers of each zone given grid of width g_width
    """
    step = SENSOR_FOV / g_width
    return np.array([math.radians(-SENSOR_FOV / 2 + (i + 0.5) * step)
                     for i in range(g_width)])


def rad_to_cartesian(distances, g_width):
    """
    Given an array of size 64 with reported distances (in mm), we convert this
    representation to an x,y,z point in Cartesian (also in mm)
    """
    distances = distances.reshape(g_width, g_width)
    angles = zone_angles(g_width)
    row_angles = angles[:, None]
    col_angles = angles[None, :]

    tx = np.tan(col_angles)
    ty = np.tan(row_angles)

    # z is the perpendicular depth
    z = distances / np.sqrt(1 + tx ** 2 + ty ** 2)
    x = z * tx
    y = -z * ty

    pts = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)

    return pts


# Verified empirically: row 0 = top of FoV, col 0 = sensor's left
# (appears on the observer's right when facing the sensor).


# -----------------------------------------------------------------------------
# SECTION 4 -- validity filtering
# -----------------------------------------------------------------------------

def zone_mask(frame):
    """Which zones to trust this frame.

    Conditions are kept separate so you can toggle them independently while
    tuning, and so the rejection counts tell you WHICH one is being harsh.
    An empty cloud caused by over-filtering looks exactly like one caused by
    a dead link -- these counters are how you tell them apart.

    Returns (mask, counts_dict).
    """
    status = np.asarray(frame['statuses'])
    sigma = np.asarray(frame['sigmas'])
    dist = np.asarray(frame['distances'], dtype=float)

    ok_status = np.isin(status, VALID_STATUS)
    ok_sigma = sigma <= SIGMA_MAX_MM
    ok_range = (dist >= MIN_DIST_MM) & (dist <= MAX_DIST_MM)
    ok_zone = ~np.isin(np.arange(status.size), list(EXCLUDED_ZONES)) \
        if EXCLUDED_ZONES else np.ones(status.size, dtype=bool)

    mask = ok_status & ok_sigma & ok_range & ok_zone

    counts = {
        'total': mask.size,
        'kept': int(mask.sum()),
        'bad_status': int((~ok_status).sum()),
        'bad_sigma': int((ok_status & ~ok_sigma).sum()),
        'bad_range': int((ok_status & ok_sigma & ~ok_range).sum()),
        'excluded': int((ok_status & ok_sigma & ok_range & ~ok_zone).sum()),
    }
    return mask, counts


def depth_colors(pts_m, near=0.1, far=3.5):
    """Red when near, blue when far. Readable without rotating the view."""
    if len(pts_m) == 0:
        return np.zeros((0, 3))
    t = np.clip((pts_m[:, 2] - near) / (far - near), 0.0, 1.0)
    return np.stack([1.0 - t, np.full_like(t, 0.25), t], axis=1)


# -----------------------------------------------------------------------------
# SECTION 6 -- analysis
# -----------------------------------------------------------------------------

def segment_plane(pts_m):
    """Fit the dominant plane. Returns (model, inlier_idx) or (None, None)."""
    if len(pts_m) < 8:
        return None, None

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts_m)
    model, inliers = cloud.segment_plane(distance_threshold=PLANE_TOL_M,
                                         ransac_n=3,
                                         num_iterations=200)
    if len(inliers) < PLANE_MIN_PTS:
        return None, None
    return model, np.asarray(inliers)


def cluster_points(pts_m):
    """DBSCAN. Returns per-point labels, -1 meaning noise.

    NOTE: CLUSTER_EPS_M is a constant, which is a compromise. Zone spacing
    scales with depth, so a single eps either fragments distant objects or
    merges nearby ones. A depth-scaled eps would be better -- one approach
    is to normalise the cloud by z before clustering, then cluster in
    angular space instead of metric space.
    """
    if len(pts_m) < CLUSTER_MIN_PTS:
        return np.full(len(pts_m), -1)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts_m)
    return np.asarray(cloud.cluster_dbscan(eps=CLUSTER_EPS_M,
                                           min_points=CLUSTER_MIN_PTS,
                                           print_progress=False))


# -----------------------------------------------------------------------------
# SECTION 5 -- Open3D rendering
# -----------------------------------------------------------------------------

PALETTE = np.array([[1.00, 0.85, 0.20],
                    [0.30, 0.90, 0.50],
                    [0.90, 0.40, 0.90],
                    [0.40, 0.70, 1.00]])

PLANE_COLOR = np.array([0.22, 0.22, 0.26])
NOISE_COLOR = np.array([0.35, 0.35, 0.35])


class CloudViewer:
    def __init__(self, do_plane=False, do_cluster=False, verbose=True):
        self.do_plane = do_plane
        self.do_cluster = do_cluster
        self.verbose = verbose
        self.boxes = []
        self.first = True
        self.last_report = time.time()

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window('VL53L7CX point cloud', width=1100, height=800)

        opt = self.vis.get_render_option()
        opt.point_size = 9.0
        opt.background_color = np.array([0.08, 0.08, 0.09])

        # Geometry is added ONCE, empty. Every frame we replace its contents
        # rather than removing and re-adding, which would churn the GPU
        # buffers and reset the camera.
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        self.rays = o3d.geometry.LineSet()
        self._update_rays(np.zeros((0, 3)))
        self.vis.add_geometry(self.rays)

        # 20 cm axes at the sensor. Without a reference, floating dots are
        # very hard to interpret.
        self.axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.vis.add_geometry(self.axes)

        self.vis.register_key_callback(ord('P'), self._toggle_plane)
        self.vis.register_key_callback(ord('C'), self._toggle_cluster)
        self.vis.register_key_callback(ord('R'), self._reset_view)
        self.vis.register_key_callback(ord('Q'), self._quit)

    # -- key callbacks ----------------------------------------------------
    def _toggle_plane(self, _v):
        self.do_plane = not self.do_plane
        print(f'plane removal: {self.do_plane}')
        return False

    def _toggle_cluster(self, _v):
        self.do_cluster = not self.do_cluster
        print(f'clustering: {self.do_cluster}')
        return False

    def _reset_view(self, _v):
        self.vis.reset_view_point(True)
        return False

    def _quit(self, v):
        v.close()
        return False

    # -- rays -------------------------------------------------------------
    def _update_rays(self, pts_m):
        """One line from the sensor origin to each point.

        Makes the cone structure visible -- you can see all 64 wedges
        diverging from a single point, which the bare dots never show.
        """
        if len(pts_m) == 0:
            # A LineSet with zero lines fails to bind in the shader and
            # floods the console. Give it one degenerate zero-length line
            # at the origin instead: valid to bind, invisible to look at.
            self.rays.points = o3d.utility.Vector3dVector(np.zeros((2, 3)))
            self.rays.lines = o3d.utility.Vector2iVector(
                np.array([[0, 1]], dtype=np.int32))
            self.rays.colors = o3d.utility.Vector3dVector(np.zeros((1, 3)))
            return

        n = len(pts_m)
        self.rays.points = o3d.utility.Vector3dVector(
            np.vstack([np.zeros((1, 3)), pts_m]))          # origin is vertex 0
        self.rays.lines = o3d.utility.Vector2iVector(
            np.column_stack([np.zeros(n, dtype=np.int32),
                             np.arange(1, n + 1, dtype=np.int32)]))
        self.rays.colors = o3d.utility.Vector3dVector(
            np.tile(RAY_COLOR, (n, 1)))

    # -- bounding boxes ---------------------------------------------------
    def _clear_boxes(self):
        for b in self.boxes:
            # reset_bounding_box=False stops the camera jumping every frame
            self.vis.remove_geometry(b, reset_bounding_box=False)
        self.boxes.clear()

    def _add_box(self, pts_m, color):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts_m)
        box = cloud.get_axis_aligned_bounding_box()
        box.color = color
        self.boxes.append(box)
        self.vis.add_geometry(box, reset_bounding_box=False)

    # -- main per-frame entry point ---------------------------------------
    def update(self, frame):
        n = frame['grid_width']
        mask, counts = zone_mask(frame)

        dist = np.asarray(frame['distances'], dtype=float)
        all_pts = rad_to_cartesian(dist, n) / 1000.0   # mm -> metres
        pts = all_pts[mask]
        radial_mm = dist[mask]      # raw sensor distance_mm, pre-projection

        self._clear_boxes()

        if len(pts) == 0:
            self.pcd.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self.pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self._update_rays(pts)
            self._render()
            self._report(frame, counts, 0, None, pts, radial_mm)
            return

        plane_pts = np.zeros((0, 3))
        plane_model = None

        if self.do_plane:
            plane_model, inliers = segment_plane(pts)
            if inliers is not None:
                plane_pts = pts[inliers]
                keep = np.setdiff1d(np.arange(len(pts)), inliers)
                pts = pts[keep]

        cols = depth_colors(pts)
        n_clusters = 0

        if self.do_cluster and len(pts):
            labels = cluster_points(pts)
            n_clusters = int(labels.max()) + 1 if labels.size else 0
            for k in range(n_clusters):
                idx = np.where(labels == k)[0]
                colour = PALETTE[k % len(PALETTE)]
                cols[idx] = colour
                self._add_box(pts[idx], colour)
            cols[labels < 0] = NOISE_COLOR

        # Plane points render dim so they recede behind the objects.
        if len(plane_pts):
            pts = np.vstack([pts, plane_pts])
            cols = np.vstack([cols, np.tile(PLANE_COLOR, (len(plane_pts), 1))])

        # Rays are built from the FINAL point set, after plane removal has
        # reassigned pts -- otherwise they reference stale indices.
        self._update_rays(pts)

        self.pcd.points = o3d.utility.Vector3dVector(pts)
        self.pcd.colors = o3d.utility.Vector3dVector(cols)
        self._render()
        self._report(frame, counts, n_clusters, plane_model, pts, radial_mm)

    def _report(self, frame, counts, n_clusters, plane_model, pts, radial_mm):
        if not self.verbose:
            return
        now = time.time()
        if now - self.last_report < 2.0:
            return
        self.last_report = now

        msg = (f"seq {frame['seq']}  "
               f"{counts['kept']}/{counts['total']} zones  "
               f"(status {counts['bad_status']}, "
               f"sigma {counts['bad_sigma']}, "
               f"range {counts['bad_range']}, "
               f"masked {counts.get('excluded', 0)} rejected)")
        if len(radial_mm):
            # Radial distance_mm (raw sensor reading), NOT the Cartesian Z
            # depth -- Z is foreshortened for off-axis zones (corner/edge
            # zones project to a smaller Z than their true straight-line
            # range), so reporting Z here made kept zones look like they
            # were outside the [MIN_DIST_MM, MAX_DIST_MM] filter when they
            # were not. This matches the band the filter actually applies.
            msg += (f"  near {radial_mm.min():.0f} mm"
                    f"  far {radial_mm.max():.0f} mm")
        if plane_model is not None:
            a, b, c, d = plane_model
            msg += f"  plane n=({a:.2f},{b:.2f},{c:.2f}) at {abs(d):.2f} m"
        if n_clusters:
            msg += f"  {n_clusters} object(s)"
        print(msg)

    def _render(self):
        # Both geometries must be flagged BEFORE pumping events, or the
        # shader renders last frame's data.
        self.vis.update_geometry(self.pcd)
        self.vis.update_geometry(self.rays)
        if self.first:
            # Programmatic camera placement is unreliable across Open3D
            # versions; this fits whatever geometry exists and works
            # everywhere. Press R to redo it by hand.
            self.vis.reset_view_point(True)
            self.first = False
        self.vis.poll_events()
        self.vis.update_renderer()

    def alive(self):
        """Pumps the event queue AND reports whether the window is open."""
        return self.vis.poll_events()

    def close(self):
        self.vis.destroy_window()


# -----------------------------------------------------------------------------
# SECTION 7 -- logging and replay
# -----------------------------------------------------------------------------

def run_serial(port, baud, plane, cluster, logfile):
    import serial

    ser = serial.Serial(port, baud, timeout=0)     # non-blocking
    view = CloudViewer(plane, cluster)
    ex = FrameExtractor()
    log = open(logfile, 'wb') if logfile else None

    print(f'{port} @ {baud}. P = plane, C = cluster, R = reset view, Q = quit')
    last_note = time.time()

    try:
        while view.alive():
            view.vis.update_renderer()

            data = ser.read(8192)
            if not data:
                time.sleep(0.005)
                if time.time() - last_note > 3.0 and ex.n_ok == 0:
                    print(f'no frames yet (bad: {ex.n_bad}) -- a fast-rising '
                          f'bad count means the baud rate is wrong; zero means '
                          f'nothing is arriving at all')
                    last_note = time.time()
                continue

            # Log the RAW bytes, before parsing: a parser bug then cannot
            # corrupt your recordings, and a fixed parser can re-read them.
            if log:
                log.write(data)

            for frame in ex.feed(data):
                view.update(frame)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if log:
            log.close()
        print(f'{ex.n_ok} frames decoded, {ex.n_bad} rejected')
        view.close()


def run_replay(path, plane, cluster, delay, loop):
    view = CloudViewer(plane, cluster)

    with open(path, 'rb') as f:
        blob = f.read()

    try:
        while True:
            ex = FrameExtractor()
            for i in range(0, len(blob), 4096):
                if not view.alive():
                    return
                for frame in ex.feed(blob[i:i + 4096]):
                    if not view.alive():
                        return
                    view.update(frame)
                    end = time.time() + delay
                    while time.time() < end:
                        if not view.alive():
                            return
                        time.sleep(0.01)
            if not loop:
                break
        while view.alive():
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        view.close()


# -----------------------------------------------------------------------------
# SECTION 8 -- entry point
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', help='serial port, e.g. COM3')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--plane', action='store_true', help='remove dominant plane')
    ap.add_argument('--cluster', action='store_true', help='cluster objects')
    ap.add_argument('--log', help='record raw bytes to this file')
    ap.add_argument('--replay', help='play back a recording instead')
    ap.add_argument('--delay', type=float, default=0.1)
    ap.add_argument('--loop', action='store_true', help='repeat the recording')
    ap.add_argument('--mask', default=os.path.join(os.path.dirname(__file__),
                                                   'zone_mask.json'),
                    help='zone exclusion JSON from zone_map.py '
                         '(default: tools/zone_mask.json if present; '
                         'pass --mask "" to disable)')
    args = ap.parse_args()

    if args.mask and os.path.exists(args.mask):
        load_zone_mask(args.mask)
    elif args.mask:
        print(f'note: no mask file at {args.mask}, showing all zones')

    if args.replay:
        run_replay(args.replay, args.plane, args.cluster, args.delay, args.loop)
    elif args.port:
        run_serial(args.port, args.baud, args.plane, args.cluster, args.log)
    else:
        sys.exit('need a port, or --replay <file>')
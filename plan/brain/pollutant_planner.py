#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import math
import json
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.time import Time
from rclpy.duration import Duration
import os
import sys
from typing import List, Tuple

# Earth radius (m)
R_EARTH = 6371000.0

def latlon_to_local_meters(lat_ref, lon_ref, lat, lon):
    """
    Equirectangular approximation relative to reference point (meters).
    Good for small areas (<< 10s km).
    """
    lat0 = math.radians(lat_ref)
    lat1 = math.radians(lat)
    dlat = lat1 - math.radians(lat_ref)
    dlon = math.radians(lon) - math.radians(lon_ref)
    x = dlon * math.cos(lat0) * R_EARTH
    y = dlat * R_EARTH
    return x, y

def local_meters_to_latlon(lat_ref, lon_ref, x, y):
    lat0 = math.radians(lat_ref)
    dlat = y / R_EARTH
    dlon = x / (R_EARTH * math.cos(lat0))
    lat = math.degrees(math.radians(lat_ref) + dlat)
    lon = math.degrees(math.radians(lon_ref) + dlon)
    return lat, lon

def point_in_poly(x, y, poly: List[Tuple[float,float]]):
    """Ray casting point-in-polygon. poly is list of (x,y)."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersect = ((yi > y) != (yj > y)) and \
                    (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside

class PollutantPlanner(Node):
    def __init__(self):
        super().__init__('pollutant_planner')
        # Parameters
        self.declare_parameter('mission_path', 'mission.yaml')
        mission_path = self.get_parameter('mission_path').get_parameter_value().string_value
        if not os.path.isabs(mission_path):
            mission_path = os.path.join(os.getcwd(), mission_path)
        if not os.path.exists(mission_path):
            self.get_logger().error(f"Mission file not found: {mission_path}")
            rclpy.shutdown()
            return

        with open(mission_path, 'r') as f:
            mission = yaml.safe_load(f)

        self.mission = mission.get('pollutant_mission', mission)  # support both layouts

        # Planner tunables (with defaults)
        planner_conf = self.mission.get('planner', {})
        self.swath_m = float(planner_conf.get('swath_m', 25.0))
        self.overlap = float(planner_conf.get('overlap', 0.2))
        self.point_spacing_m = float(planner_conf.get('point_spacing_m', 10.0))
        if self.overlap < 0.0 or self.overlap >= 1.0:
            self.get_logger().warn("Overlap out of range -> clamped to 0.2")
            self.overlap = 0.2

        # Search polygon in lat/lon
        self.poly_latlon = [(p['lat'], p['lon']) for p in self.mission['search_area']]
        if len(self.poly_latlon) < 3:
            self.get_logger().error("search_area must contain at least 3 points")
            rclpy.shutdown()
            return

        # Reference point (first vertex) for local frame
        self.lat_ref, self.lon_ref = self.poly_latlon[0]

        # Convert polygon to local meters
        self.poly_local = [latlon_to_local_meters(self.lat_ref, self.lon_ref, lat, lon)
                           for (lat, lon) in self.poly_latlon]

        # Publishers
        self.path_pub = self.create_publisher(Path, '/pollutant/waypoints_path', 10)
        self.geo_pub = self.create_publisher(String, '/pollutant/waypoints_geo', 10)
        self.mission_pub = self.create_publisher(String, '/pollutant/mission_info', 10)

        # Publish mission info once
        self.publish_mission_info()

        # Build plan immediately
        self.get_logger().info("Building coverage plan...")
        waypoints_local = self.build_lawnmower_plan()
        self.get_logger().info(f"Planned {len(waypoints_local)} waypoints (local frame).")

        # Publish path
        self.publish_path(waypoints_local)

        # Publish geo JSON
        self.publish_geo_waypoints(waypoints_local)

        self.get_logger().info("Pollutant planner initialized and published plan.")

    def publish_mission_info(self):
        info = {
            'target_pollutant': self.mission.get('target_pollutant', 'any'),
            'on_detection': self.mission.get('on_detection', 'stop_and_mark'),
            'detection_threshold': self.mission.get('detection_threshold', {})
        }
        msg = String()
        msg.data = json.dumps(info)
        self.mission_pub.publish(msg)

    def build_lawnmower_plan(self) -> List[Tuple[float,float]]:
        """
        Create lawnmower tracks across polygon bbox, sample points along-track,
        keep those inside polygon, compress contiguous segments to endpoints.
        Returns list of local (x,y) waypoints in meters in ENU relative to lat_ref/lon_ref.
        """
        poly = self.poly_local
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        # effective spacing between swaths (accounting for overlap)
        spacing = self.swath_m * (1.0 - self.overlap)
        if spacing <= 0.1:
            spacing = self.swath_m * 0.5

        # We'll orient tracks along the longer bbox axis (choose direction giving fewer tracks)
        bbox_width = xmax - xmin
        bbox_height = ymax - ymin
        rotate = False  # keep tracks parallel to x-axis by default
        if bbox_height > bbox_width:
            # rotate coordinate frame by 90 deg (i.e., make tracks along y)
            rotate = True

        # If rotated, swap x/y for bbox and polygon during track generation and swap back afterwards
        if rotate:
            poly_for = [(y, x) for (x,y) in poly]
            xmin, xmax = ymin, ymax
            ymin, ymax = min([p[0] for p in poly_for]), max([p[0] for p in poly_for])
        else:
            poly_for = poly

        # track positions (y positions) from ymin to ymax
        n_tracks = max(1, int(math.ceil((ymax - ymin) / spacing)) + 1)
        track_ys = [ymin + i * spacing for i in range(n_tracks)]
        # ensure last track reaches ymax
        if track_ys[-1] < ymax:
            track_ys.append(ymax)

        waypoints = []
        direction = 1  # left-to-right or right-to-left alternation
        # Sampling along x
        sample_step = self.point_spacing_m

        for ti, ty in enumerate(track_ys):
            # sample x from xmin to xmax
            x_samples = []
            x = xmin
            while x <= xmax + 1e-6:
                x_samples.append(x)
                x += sample_step
            # ensure xmax present
            if abs(x_samples[-1] - xmax) > 1e-3:
                x_samples.append(xmax)

            # Build sample points at (x, ty)
            pts = [(xv, ty) for xv in x_samples]
            # Check inside polygon (point_in_poly) using poly_for
            inside_mask = [point_in_poly(px, py, poly_for) for (px, py) in pts]

            # compress contiguous "inside" runs to endpoints
            i = 0
            runs = []
            N = len(pts)
            while i < N:
                if inside_mask[i]:
                    j = i
                    while j+1 < N and inside_mask[j+1]:
                        j += 1
                    # run from i..j (inclusive)
                    runs.append((pts[i], pts[j]))
                    i = j + 1
                else:
                    i += 1

            # for each run, add start and end as waypoints
            run_points = []
            for (pstart, pend) in runs:
                # order depending on direction
                if direction == 1:
                    run_points.append(pstart)
                    run_points.append(pend)
                else:
                    run_points.append(pend)
                    run_points.append(pstart)

            # alternate direction for next track
            direction *= -1

            # append to waypoints
            for rp in run_points:
                if rotate:
                    # swap back x,y since we rotated earlier
                    waypoints.append((rp[1], rp[0]))
                else:
                    waypoints.append(rp)

        # Remove duplicates that may appear
        compressed = []
        seen = set()
        for (x,y) in waypoints:
            key = (round(x,3), round(y,3))
            if key in seen:
                continue
            seen.add(key)
            compressed.append((x,y))

        return compressed

    def publish_path(self, waypoints_local: List[Tuple[float,float]]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'  # local ENU frame; adapt to your tf frames
        for idx, (x,y) in enumerate(waypoints_local):
            pose_st = PoseStamped()
            pose_st.header.stamp = self.get_clock().now().to_msg()
            pose_st.header.frame_id = path.header.frame_id
            # put (x,y) in Pose.position, z=0
            pose_st.pose.position.x = float(x)
            pose_st.pose.position.y = float(y)
            pose_st.pose.position.z = 0.0
            # Simple orientation: face next waypoint if possible
            q = Quaternion()
            if idx + 1 < len(waypoints_local):
                nx, ny = waypoints_local[idx+1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            # quaternion from yaw
            q.w = math.cos(yaw/2.0)
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(yaw/2.0)
            pose_st.pose.orientation = q
            path.poses.append(pose_st)

        # publish
        self.path_pub.publish(path)
        self.get_logger().info(f"Published Path with {len(path.poses)} poses to /pollutant/waypoints_path")

    def publish_geo_waypoints(self, waypoints_local: List[Tuple[float,float]]):
        # convert to lat/lon list
        geo = []
        for (x,y) in waypoints_local:
            lat, lon = local_meters_to_latlon(self.lat_ref, self.lon_ref, x, y)
            geo.append({'lat': float(lat), 'lon': float(lon)})

        msg = String()
        msg.data = json.dumps({'waypoints': geo})
        self.geo_pub.publish(msg)
        self.get_logger().info(f"Published {len(geo)} geo waypoints to /pollutant/waypoints_geo")

def main(args=None):
    rclpy.init(args=args)
    node = PollutantPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info("Shutting down pollutant_planner")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

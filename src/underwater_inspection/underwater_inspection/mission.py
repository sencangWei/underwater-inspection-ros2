"""ROS 2 vision-guided inspection mission and live sensor evidence recorder."""
import csv
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from stonefish_control_msgs.msg import TrajectoryPoint
from stonefish_msgs.srv import SetOceanCurrent
from visualization_msgs.msg import Marker

from .perception import detect_panel, visual_velocity
from .planning import clearance, plan_route


class InspectionMission(Node):
    def __init__(self):
        super().__init__('inspection_mission')
        self.declare_parameter('config_path', '')
        self.declare_parameter('output_dir', '/tmp/underwater-inspection-run')
        self.cfg = yaml.safe_load(Path(self.get_parameter('config_path').value).read_text())
        self.out = Path(self.get_parameter('output_dir').value)
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / 'screenshots').mkdir(exist_ok=True)
        self.bridge = CvBridge()
        self.pose = None
        self.yaw = 0.0
        self.speed = 0.0
        self.control_mode = 'position'
        self.velocity_command = [0.0, 0.0, 0.0]
        self.pose_wall = self.image_wall = 0.0
        self.panel_wall = 0.0
        self.camera = self.sonar = self.detection = None
        self.fx = 640 / (2 * math.tan(math.radians(75) / 2))
        self.phase = 'WAIT_FOR_SENSORS'
        self.started = None
        self.phase_started = 0.0
        self.finished = None
        self.events = []
        self.contacts = set()
        self.min_clearance = float('inf')
        self.track = []
        self.inspect_good = 0.0
        self.inspect_errors = []
        self.detection_frames = 0
        self.route = plan_route(self.cfg['spawn'][:2], self.cfg['approach_gate'][:2], self.cfg)
        self.waypoint = 1
        self.setpoint = list(self.cfg['spawn'])
        self.writer = None
        self.current_result = None
        self.last_tick = None
        self.last_video = -1.0
        self.saved_phases = set()
        self.sensor_pauses = 0
        self.paused = False
        self.telemetry = (self.out / 'telemetry.csv').open('w', newline='')
        self.frame_times = (self.out / 'frame_times.csv').open('w', newline='')
        self.frame_csv = csv.writer(self.frame_times)
        self.frame_csv.writerow(['ros_wall_time'])
        self.csv = csv.writer(self.telemetry)
        self.csv.writerow(['ros_wall_time', 'phase', 'x', 'y', 'z', 'yaw', 'panel_range',
                           'panel_cx', 'panel_cy', 'obstacle_clearance', 'inspect_good_seconds',
                           'cmd_u', 'cmd_v', 'cmd_w'])
        self.command = self.create_publisher(TrajectoryPoint, '/bluerov2/cmd_pose', 10)
        self.mode_pub = self.create_publisher(String, '/bluerov2/control_mode',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = self.create_publisher(String, '/inspection/status', 10)
        self.dashboard = self.create_publisher(Image, '/inspection/dashboard', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/bluerov2/odometry', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Image, '/bluerov2/stereo/left/image_color', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/bluerov2/stereo/left/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/bluerov2/fls/display', self.on_sonar, qos_profile_sensor_data)
        for body in ['seafloor', 'inspection_panel', 'panel_frame'] + [o['name'] for o in self.cfg['obstacles']]:
            self.create_subscription(Marker, '/inspection/contact/' + body,
                                     lambda msg, name=body: self.contacts.add(name), 10)
        self.current = self.create_client(SetOceanCurrent,
            '/stonefish_ros2/stonefish_simulator/set_ocean_current')
        self.current_future = None
        self.create_timer(.1, self.tick)
        (self.out / 'route.json').write_text(json.dumps(self.route, indent=2))
        self.get_logger().info('Known-map A* transit + RGB visual servo + PID hold; simulator odometry baseline.')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = [p.x, p.y, p.z]
        v = msg.twist.twist.linear
        self.speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose_wall = time.monotonic()

    def on_info(self, msg):
        if msg.k[0] > 0:
            self.fx = msg.k[0]

    def on_image(self, msg):
        self.camera = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.image_wall = time.monotonic()
        self.detection = detect_panel(self.camera, self.fx, self.cfg['panel_height'])
        if self.detection:
            self.detection_frames += 1
            self.panel_wall = time.monotonic()
        if self.started is not None and self.pose is not None:
            self.record_view()

    def on_sonar(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.sonar = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image

    def transition(self, phase):
        self.phase, self.phase_started = phase, self.now()
        event = {'ros_wall_time': self.now(), 'phase': phase, 'pose': list(self.pose)}
        self.events.append(event)
        (self.out / 'events.json').write_text(json.dumps(self.events, indent=2))
        self.get_logger().info(f'PHASE={phase} pose={self.pose}')
        mode = 'velocity' if phase in ('VISUAL_APPROACH', 'INSPECT_HOLD') else 'position'
        if mode != self.control_mode:
            self.mode_pub.publish(String(data=mode))
            self.control_mode = mode

    def send_pose(self, xyz, yaw, velocity=(0.0, 0.0, 0.0)):
        self.setpoint = list(xyz)
        msg = TrajectoryPoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world_ned'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.velocity_command = list(map(float, velocity))
        msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z = self.velocity_command
        self.command.publish(msg)

    def stop_for_missing_panel(self):
        self.inspect_good = 0.0
        self.send_pose(self.pose, 0.0)
        if time.monotonic() - self.panel_wall > 3.0:
            self.finish(False, 'Panel lost during visual servo')

    def tick(self):
        now = self.now()
        if self.pose is None:
            return
        if self.current_future is None and self.current.service_is_ready():
            request = SetOceanCurrent.Request(current_index=0, enable=True,
                velocity=list(map(float, self.cfg['water_current'])))
            self.current_future = self.current.call_async(request)
        if self.current_future and self.current_future.done() and self.current_result is None:
            response = self.current_future.result()
            self.current_result = {'success': response.success, 'message': response.message}
            self.get_logger().info('CURRENT=' + json.dumps(self.current_result))
        if self.started is None:
            self.send_pose(self.cfg['spawn'], 0.0)
            if self.camera is not None and self.current_result and self.current_result['success']:
                self.started = now
                self.last_tick = now
                self.transition('TRANSIT_AVOID_OBSTACLES')
            return
        dt = max(0.0, min(.2, now - self.last_tick))
        self.last_tick = now
        gap = clearance(self.pose, self.cfg['obstacles'], self.cfg['robot_radius'])
        self.min_clearance = min(self.min_clearance, gap)
        if self.phase not in ('COMPLETE', 'FAILED'):
            self.track.append(self.pose[:2])
            if self.contacts or gap < 0:
                self.finish(False, 'Collision or geometric clearance violation')
            elif now - self.started > self.cfg['timeout_seconds']:
                self.finish(False, 'Mission timeout')
            else:
                pose_age = time.monotonic() - self.pose_wall
                camera_age = time.monotonic() - self.image_wall
                if pose_age > .5 or camera_age > 1.0:
                    # Stop the visual velocity command or hold the last measured
                    # position while the simulator's render/sensor loop recovers.
                    self.send_pose(self.pose, 0.0)
                    if not self.paused:
                        self.sensor_pauses += 1
                        self.paused = True
                        self.get_logger().warning('SENSOR_PAUSE: stop advancement until fresh data')
                    self.inspect_good = 0.0 if self.phase == 'INSPECT_HOLD' else self.inspect_good
                    if max(pose_age, camera_age) > 5.0:
                        self.finish(False, 'Sensor outage exceeded 5 seconds')
                    return
                if self.paused:
                    self.paused = False
                    self.get_logger().info('SENSOR_RESUMED: continuing from fresh data')
        if self.phase in ('TRANSIT_AVOID_OBSTACLES', 'RETURN_HOME'):
            target = list(self.route[self.waypoint]) + [self.cfg['spawn'][2]]
            if math.dist(self.pose, target) < .25:
                self.waypoint += 1
                if self.waypoint == len(self.route):
                    if self.phase == 'RETURN_HOME':
                        self.finish(True, 'Inspection and return completed')
                    else:
                        self.transition('SEARCH_PANEL')
            if self.phase in ('TRANSIT_AVOID_OBSTACLES', 'RETURN_HOME'):
                self.send_pose(target, 0.0)
        elif self.phase == 'SEARCH_PANEL':
            self.send_pose(self.cfg['approach_gate'], 0.0)
            if self.detection is not None and self.speed < .12 and abs(self.yaw) < .06:
                self.transition('VISUAL_APPROACH')
            elif now - self.phase_started > 15:
                self.finish(False, 'Panel not detected at the inspection gate')
        elif self.phase in ('VISUAL_APPROACH', 'INSPECT_HOLD'):
            if self.detection is None:
                self.stop_for_missing_panel()
            else:
                d = self.detection
                h, w = self.camera.shape[:2]
                velocity, errors = visual_velocity(d, self.camera.shape, self.fx, self.cfg['standoff'])
                dr, _, dz = errors
                # A velocity PID with a fresh integral handles the slow visual
                # outer loop. Pose-relative incremental position commands carry
                # old transit integral into approach and can keep accelerating.
                self.send_pose(self.pose, 0.0, velocity)
                good = abs(dr) < .16 and abs(d['cx'] - w / 2) < 25 and abs(dz) < .12
                if good:
                    self.inspect_good += dt
                    self.inspect_errors.append([dr, dz, d['cx'] - w / 2])
                    if self.phase == 'VISUAL_APPROACH':
                        self.transition('INSPECT_HOLD')
                else:
                    self.inspect_good = 0.0
                if self.inspect_good >= self.cfg['inspect_seconds']:
                    cv2.imwrite(str(self.out / 'inspection-photo.png'), self.camera)
                    self.route = plan_route(self.pose[:2], self.cfg['spawn'][:2], self.cfg)
                    self.waypoint = 1
                    self.transition('RETURN_HOME')
        elif self.phase in ('COMPLETE', 'FAILED'):
            self.send_pose(self.setpoint, 0.0)
        d = self.detection or {}
        self.csv.writerow([now, self.phase, *self.pose, self.yaw, d.get('range', ''),
                           d.get('cx', ''), d.get('cy', ''), gap, self.inspect_good,
                           *self.velocity_command])
        self.telemetry.flush()
        self.status.publish(String(data=json.dumps({'phase': self.phase, 'pose': self.pose,
            'range': d.get('range'), 'clearance': gap, 'inspect_seconds': self.inspect_good})))
        self.get_logger().info(f'{self.phase}: xyz={np.round(self.pose, 2)} clearance={gap:.2f}m '
            f'panel={d.get("range", 0):.2f}m hold={self.inspect_good:.1f}s', throttle_duration_sec=2)

    def finish(self, success, reason):
        self.finished = self.now()
        self.setpoint = list(self.pose)
        self.transition('COMPLETE' if success else 'FAILED')
        errors = np.array(self.inspect_errors)
        summary = {'success': success, 'reason': reason, 'duration_wall_seconds': self.finished - self.started,
            'clock_source': 'ROS system clock matching upstream sensor stamps; upstream has no /clock publisher',
            'state_source': 'Stonefish ground-truth odometry baseline',
            'planner': 'known-site inflated A* at fixed depth',
            'perception': 'RGB yellow-panel geometric marker detector; no learned model',
            'current': self.current_result, 'current_velocity_ned': self.cfg['water_current'],
            'minimum_obstacle_clearance_m': self.min_clearance,
            'contact_bodies': sorted(self.contacts), 'detected_camera_frames': self.detection_frames,
            'sensor_pause_count': self.sensor_pauses,
            'inspection_required_seconds': self.cfg['inspect_seconds'],
            'inspection_completed_seconds': self.inspect_good,
            'home_error_m': math.dist(self.pose, self.cfg['spawn']),
            'inspection_range_rmse_m': float(np.sqrt(np.mean(errors[:, 0] ** 2))) if len(errors) else None,
            'events': self.events}
        (self.out / 'summary.json').write_text(json.dumps(summary, indent=2))
        self.get_logger().info('RESULT=' + json.dumps(summary))

    def record_view(self):
        now = self.now()
        if now - self.last_video < .085 or (self.finished is not None and now - self.finished > 3):
            return
        self.last_video = now
        view = np.full((720, 1280, 3), (28, 31, 39), dtype=np.uint8)
        image = self.camera.copy()
        if self.detection:
            x, y, w, h = self.detection['bbox']
            cv2.rectangle(image, (x, y), (x + w, y + h), (30, 255, 100), 2)
            cv2.putText(image, f'PANEL {self.detection["range"]:.2f} m', (x, max(20, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (30, 255, 100), 2)
        view[40:520, :640] = cv2.resize(image, (640, 480))
        if self.sonar is not None:
            view[530:710, :640] = cv2.resize(self.sonar, (640, 180))
        # This map is explicitly a telemetry view, with actual simulated odometry.
        def point(xy):
            return round(670 + (xy[0] + 6) / 12 * 570), round(500 - (xy[1] + 4) / 8 * 430)
        for obstacle in self.cfg['obstacles']:
            cv2.circle(view, point((obstacle['x'], obstacle['y'])), round(obstacle['radius'] / 12 * 570),
                       (100, 120, 140), -1)
        if len(self.route) > 1:
            cv2.polylines(view, [np.array([point(p) for p in self.route], dtype=np.int32)], False,
                          (100, 90, 40), 1)
        if len(self.track) > 1:
            cv2.polylines(view, [np.array([point(p) for p in self.track], dtype=np.int32)], False,
                          (240, 210, 70), 2)
        for label, location, color in [('HOME', self.cfg['spawn'], (200, 230, 60)),
                                        ('PANEL', self.cfg['target'], (0, 230, 240)),
                                        ('ROV', self.pose, (60, 255, 100))]:
            px, py = point(location)
            cv2.circle(view, (px, py), 7, color, -1)
            cv2.putText(view, label, (px + 9, py), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)
        texts = [('STONEFISH RGB | visual target detection', (12, 27)),
                 ('LIVE TRAJECTORY | ground-truth baseline', (660, 27)),
                 (f'{self.phase}', (660, 548)),
                 (f't={now - self.started:.1f}s  depth={self.pose[2]:.2f}m', (660, 583)),
                 (f'current NED={self.cfg["water_current"]} m/s', (660, 618)),
                 (f'clearance={self.min_clearance:.2f}m  contacts={len(self.contacts)}', (660, 653)),
                 (f'inspection hold={self.inspect_good:.1f}/{self.cfg["inspect_seconds"]:.1f}s', (660, 688))]
        for text, location in texts:
            cv2.putText(view, text, location, cv2.FONT_HERSHEY_SIMPLEX, .57, (235, 240, 245), 1, cv2.LINE_AA)
        if self.writer is None:
            self.writer = cv2.VideoWriter(str(self.out / 'sensors-telemetry.mp4'),
                cv2.VideoWriter_fourcc(*'mp4v'), 10.0, (1280, 720))
            if not self.writer.isOpened():
                raise RuntimeError('Could not open evidence video')
        self.writer.write(view)
        self.frame_csv.writerow([now])
        self.frame_times.flush()
        if self.phase not in self.saved_phases:
            cv2.imwrite(str(self.out / 'screenshots' / (self.phase + '.png')), view)
            self.saved_phases.add(self.phase)
        image_msg = self.bridge.cv2_to_imgmsg(view, 'bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'inspection_dashboard'
        self.dashboard.publish(image_msg)

    def close(self):
        if self.writer is not None:
            self.writer.release()
        self.telemetry.close()
        self.frame_times.close()


def main():
    rclpy.init()
    node = InspectionMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

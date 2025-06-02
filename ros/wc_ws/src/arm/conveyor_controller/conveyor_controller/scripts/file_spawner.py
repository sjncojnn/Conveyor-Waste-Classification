#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String

# ← Make sure this import is here:
from ament_index_python.packages import get_package_share_directory

class FileSpawner(Node):
    def __init__(self):
        super().__init__('file_spawner')
        # 1) parameters
        self.declare_parameter('spawn_list_file', 'spawn_list.txt')
        self.declare_parameter('spawn_rate', 5.0)        # seconds between spawns
        self.declare_parameter('model_package', 'conveyorbelt_gazebo')
        # default pose (x, y, z, roll, pitch, yaw)
        self.default_pose = (0.0, -4.0, 0.76, 0.0, 0.0, 0.0)

        # 2) load parameters
        self.list_path     = self.get_parameter('spawn_list_file').value
        self.spawn_rate    = self.get_parameter('spawn_rate').value
        self.model_package = self.get_parameter('model_package').value

        # 3) parse the spawn list
        self.entries = self._load_spawn_list(self.list_path)
        if not self.entries:
            self.get_logger().error(f"No valid entries in {self.list_path}")

        # 4) wait for Gazebo’s spawn service
        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # 5) set up a timer to step through entries
        self.counter = 0
        self.timer   = self.create_timer(self.spawn_rate, self._on_timer)
        self.get_logger().info(
            f"FileSpawner: {len(self.entries)} items, every {self.spawn_rate}s"
        )

    def _load_spawn_list(self, path):
        if not os.path.isfile(path):
            self.get_logger().error(f"Spawn list not found: {path}")
            return []
        entries = []
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [p.strip() for p in line.split(',')]
                # require at least sdf and prefix
                if len(parts) < 2:
                    self.get_logger().warn(f"Skipping invalid line (need sdf,prefix): {line}")
                    continue
                sdf, prefix = parts[0], parts[1]

                # parse up to 6 pose values, fallback to defaults
                coords = []
                for i in range(2, min(len(parts), 8)):
                    try:
                        coords.append(float(parts[i]))
                    except ValueError:
                        self.get_logger().warn(f"Skipping malformed number in line: {line}")
                        coords = []
                        break
                # pad coords with defaults
                while len(coords) < 6:
                    coords.append(self.default_pose[len(coords)])
                x, y, z, roll, pitch, yaw = coords

                entries.append({
                    'sdf': sdf,
                    'prefix': prefix,
                    'pose': (x, y, z, roll, pitch, yaw)
                })
        return entries

    def _on_timer(self):
        if self.counter >= len(self.entries):
            self.get_logger().info("All entries spawned; stopping.")
            self.timer.cancel()
            return

        e = self.entries[self.counter]
        # ----- P A T C H E D   L I N E   H E R E -----
        # Old: share = os.getenv('AMENT_PREFIX_PATH').split(':')[0]
        #      model_path = os.path.join(share, 'share', self.model_package, 'urdf', e['sdf'])
        # New (directly load from workspace package “conveyorbelt_gazebo/urdf/”):
        share = get_package_share_directory(self.model_package)
        model_path = os.path.join(share, 'urdf', e['sdf'])
        # -----------------------------------------------

        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model not found: {model_path}")
            self.counter += 1
            return

        xml = open(model_path, 'r').read()

        # build the request
        req = SpawnEntity.Request()
        req.name            = f"{e['prefix']}_{self.counter}"
        req.xml             = xml
        req.robot_namespace = req.name

        # pose
        pose = Pose()
        x, y, z, roll, pitch, yaw = e['pose']
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        req.initial_pose    = pose
        req.reference_frame = 'world'

        self.get_logger().info(f"Spawning {req.name} from {e['sdf']} at {x}, {y}, {z}")
        self.cli.call_async(req)
        self.counter += 1

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (radians) to quaternion (x,y,z,w)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    node = FileSpawner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

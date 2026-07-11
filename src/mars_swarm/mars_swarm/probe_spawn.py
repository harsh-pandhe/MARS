#!/usr/bin/env python3
"""
Spawn-clearance probe for the MARS cafe world (v2).

Why v2: v1 reused the spawn_multi launch (which always spawns tb1, contaminating
readings) and respawned into one long-lived Gazebo, which segfaults in the
depth-camera renderer on this NVIDIA/EGL box after the first spawn. v2 instead:
  - launches a BARE cafe world (no robots) directly via `gz sim`,
  - spawns ONE lidar probe robot per candidate,
  - kills and relaunches Gazebo fresh for every candidate (no respawn crash),
  - strips the depth camera from the robot SDF to avoid the Ogre crash entirely.

Run from the BUILT workspace:
    cd ~/GitHub/MARS
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/mars_swarm/mars_swarm/probe_spawn.py

Reads /probe/scan min range = nearest obstacle at each candidate. Pick a centroid
with clearance >= ~1.0 m, then set it into the 3 spawn locations.
"""
import os
import re
import time
import math
import signal
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory

# Candidate spawn centroids (world x, y). (0,0) already measured ~0.69 m in v1.
CANDIDATES = [
    (0.0,  0.0),
    (0.0,  2.0),
    (0.0,  3.0),
    (0.0, -3.0),
    (2.0,  0.0),
    (3.0,  0.0),
    (-2.0, 0.0),
    (-3.0, 0.0),
    (2.0,  2.0),
    (-2.0, 2.0),
]

BOOT_S = 9.0            # world boot time before spawning
SETTLE_S = 4.0         # settle + scans
SAFE_CLEARANCE = 1.0   # metres


class ScanProbe(Node):
    def __init__(self):
        super().__init__('spawn_probe')
        self.min_range = None
        self.create_subscription(LaserScan, '/probe/scan', self._cb, 10)

    def _cb(self, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        r = r[np.isfinite(r)]
        r = r[r > 0.05]
        if len(r):
            self.min_range = float(np.min(r))


def _env_with_resources():
    env = os.environ.copy()
    tb3 = get_package_share_directory('nav2_minimal_tb3_sim')
    extra = [os.path.join(tb3, 'models'), os.path.dirname(os.path.abspath(tb3))]
    env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [p for p in [env.get('GZ_SIM_RESOURCE_PATH', '')] + extra if p])
    return env


def _lidar_only_sdf():
    """xacro the waffle, then strip the depth-camera sensor (Ogre crash source)."""
    tb3 = get_package_share_directory('nav2_minimal_tb3_sim')
    robot_sdf = os.path.join(tb3, 'urdf', 'gz_waffle.sdf.xacro')
    sdf = subprocess.check_output(['xacro', 'namespace:=probe', robot_sdf]).decode()
    # Remove any <sensor ... type="depth"/"rgbd"/"camera" ...> ... </sensor> block.
    sdf = re.sub(
        r'<sensor[^>]*type=["\'](?:depth|rgbd|camera)["\'][^>]*>.*?</sensor>',
        '', sdf, flags=re.DOTALL)
    return sdf


def start_world(world, env):
    proc = subprocess.Popen(
        ['gz', 'sim', '-r', '-s', '--force-version', '8', world],
        preexec_fn=os.setsid, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(BOOT_S)
    return proc


def start_bridge():
    return subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/probe/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def spawn(sdf, x, y, env):
    subprocess.run(
        ['ros2', 'run', 'ros_gz_sim', 'create', '-name', 'probe',
         '-string', sdf, '-x', str(x), '-y', str(y), '-z', '0.05', '-Y', '0.0'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass


def main():
    share = get_package_share_directory('mars_swarm')
    world = os.path.join(share, 'worlds', 'cafe.sdf')
    env = _env_with_resources()
    sdf = _lidar_only_sdf()

    rclpy.init()
    node = ScanProbe()
    results = []

    for (x, y) in CANDIDATES:
        world_proc = start_world(world, env)
        bridge = start_bridge()
        spawn(sdf, x, y, env)

        node.min_range = None
        t0 = time.time()
        while time.time() - t0 < SETTLE_S:
            rclpy.spin_once(node, timeout_sec=0.2)
        clr = node.min_range

        tag = 'OPEN ' if (clr is not None and clr >= SAFE_CLEARANCE) else 'BLOCKED'
        print(f'[{tag}] spawn ({x:+.2f}, {y:+.2f})  min_clearance = '
              f'{clr if clr is not None else float("nan"):.3f} m', flush=True)
        results.append(((x, y), clr))

        kill(bridge)
        kill(world_proc)
        time.sleep(2.0)

    node.destroy_node()
    rclpy.shutdown()

    good = [(c, r) for c, r in results if r is not None and r >= SAFE_CLEARANCE]
    print('\n=== OPEN SPAWN CENTROIDS (clearance desc) ===')
    for c, r in sorted(good, key=lambda kv: -kv[1]):
        print(f'  ({c[0]:+.2f}, {c[1]:+.2f})  clearance {r:.2f} m')
    if not good:
        best = max((r for _, r in results if r is not None), default=0.0)
        print(f'  none >= {SAFE_CLEARANCE:.1f} m (best {best:.2f} m) — widen CANDIDATES.')


if __name__ == '__main__':
    main()

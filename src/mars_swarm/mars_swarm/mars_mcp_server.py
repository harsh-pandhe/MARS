#!/usr/bin/env python3
"""MARS Robotics Swarm MCP Server.

Provides a Model Context Protocol (MCP) interface for AI agents to inspect,
monitor, and control multi-robot swarms (tb1, tb2, tb3) in Gazebo and ROS 2 Jazzy.
"""

import sys
import os
import subprocess
import json
import time
from typing import Dict, Any, List

from fastmcp import FastMCP

mcp = FastMCP("mars-swarm-mcp")


@mcp.tool()
def list_ros2_topics() -> List[str]:
    """List all currently active ROS 2 topics."""
    try:
        res = subprocess.run(
            ["bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 topic list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        return [t.strip() for t in res.stdout.strip().splitlines() if t.strip()]
    except Exception as e:
        return [f"Error querying topics: {e}"]


@mcp.tool()
def get_swarm_status() -> Dict[str, Any]:
    """Inspect the status of all swarm robots (tb1, tb2, tb3)."""
    status = {"robots": {}, "simulation_running": False}
    try:
        res = subprocess.run(
            ["bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 topic list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        topics = res.stdout.strip().splitlines()
        
        for robot_id in ["tb1", "tb2", "tb3"]:
            odom_topic = f"/{robot_id}/odom"
            scan_topic = f"/{robot_id}/scan"
            cmd_topic = f"/{robot_id}/cmd_vel"
            
            has_odom = odom_topic in topics
            has_scan = scan_topic in topics
            has_cmd = cmd_topic in topics
            
            status["robots"][robot_id] = {
                "active": has_odom and has_scan,
                "odom_topic": odom_topic if has_odom else None,
                "scan_topic": scan_topic if has_scan else None,
                "cmd_vel_topic": cmd_topic if has_cmd else None,
            }
            if has_odom:
                status["simulation_running"] = True
                
        return status
    except Exception as e:
        status["error"] = str(e)
        return status


@mcp.tool()
def read_robot_odometry(robot_id: str = "tb1") -> Dict[str, Any]:
    """Read the latest odometry pose and velocity for a specific robot.
    
    Args:
        robot_id: The namespace of the robot (tb1, tb2, tb3).
    """
    topic = f"/{robot_id}/odom"
    try:
        res = subprocess.run(
            ["bash", "-c", f"source /opt/ros/jazzy/setup.bash && ros2 topic echo {topic} --once"],
            capture_output=True,
            text=True,
            timeout=4
        )
        if res.returncode != 0 or not res.stdout:
            return {"robot_id": robot_id, "status": "no data", "raw": res.stderr}
        return {"robot_id": robot_id, "status": "ok", "sample": res.stdout[:800]}
    except Exception as e:
        return {"robot_id": robot_id, "error": str(e)}


@mcp.tool()
def send_velocity_command(robot_id: str = "tb1", linear_x: float = 0.0, angular_z: float = 0.0) -> Dict[str, Any]:
    """Send a Twist velocity command to a specific robot.
    
    Args:
        robot_id: The namespace of the robot (tb1, tb2, tb3).
        linear_x: Linear forward velocity in m/s (clamped to [-0.22, 0.22]).
        angular_z: Angular yaw velocity in rad/s (clamped to [-1.0, 1.0]).
    """
    # Safety guardrails
    v = max(min(linear_x, 0.22), -0.22)
    w = max(min(angular_z, 1.0), -1.0)
    
    topic = f"/{robot_id}/cmd_vel"
    msg_str = f"{{linear: {{x: {v}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {w}}}}}"
    try:
        res = subprocess.run(
            ["bash", "-c", f"source /opt/ros/jazzy/setup.bash && ros2 topic pub --once {topic} geometry_msgs/msg/Twist '{msg_str}'"],
            capture_output=True,
            text=True,
            timeout=3
        )
        return {"robot_id": robot_id, "command_sent": {"v": v, "w": w}, "status": "success"}
    except Exception as e:
        return {"robot_id": robot_id, "error": str(e)}


@mcp.tool()
def run_headless_coverage_test(steps: int = 50, world: str = "cafe") -> Dict[str, Any]:
    """Execute a headless coverage test run and return resulting metrics.
    
    Args:
        steps: Number of simulation steps to run (e.g. 50).
        world: World environment ('cafe', 'warehouse', 'depot', 'office', or 'maze').
    """
    cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source install/setup.bash && "
        f"python3 src/mars_swarm/mars_swarm/evaluate_benchmarks.py --coverage-demo --episodes 1 --max-steps {steps} --headless --world {world}"
    )
    try:
        res = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=120
        )
        output = res.stdout
        summary_lines = [l for l in output.splitlines() if "FINAL COVERAGE" in l or "Step" in l]
        return {
            "exit_code": res.returncode,
            "summary": summary_lines[-5:] if summary_lines else ["No summary found"],
            "stdout_tail": output[-1000:]
        }
    except Exception as e:
        return {"error": str(e)}


def main():
    mcp.run()


if __name__ == "__main__":
    main()

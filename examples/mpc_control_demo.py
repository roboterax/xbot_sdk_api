#!/usr/bin/env python3
"""
MPC Control Demo

Steps:
- Activate algorithm control permission (/activate_service)
- Start MPC via teleoperation service
- Publish a ServoPose to lift arms
- Query MPC status
- Stop MPC and deactivate permission
"""

import rclpy
import sys
import os
import time

from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped

# Add the parent directory to the path to import robot_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_controller import RobotController, MPCController, TrajectoryController
from xbot_common_interfaces.msg import ServoPose


def make_pose_stamped(x, y, z, qx, qy, qz, qw, frame_id='base_link') -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


def main():
    rclpy.init()
    robot = None
    try:
        robot = RobotController("mpc_control_demo")
        # Robot model: default is Q5. For L3 or L7, change "Q5" to "L3" or "L7".
        model = "Q5"
        trajectory_controller = TrajectoryController(robot, model)
        mpc = MPCController(robot)

        if robot.start_joint_service("", False, "pos"):
            print("✓ Joint service started successfully")
        else:
            print("✗ Failed to start joint service")
            return
        
        # Step 2: Initialize joints (this takes about 20 seconds)
        print("\n=== Step 2: Initializing Joints ===")
        print("This process takes about 20 seconds...")
        if robot.initialize_joints():
            print("✓ Joints initialized successfully")
        else:
            print("✗ Failed to initialize joints")
            return
        
        # Step 3: Set to zero position
        print("\n=== Step 3: Setting Zero Position ===")
        print("This process takes about 2 seconds...")
        if trajectory_controller.set_zero_position(duration=4.0):
            print("✓ Zero position set successfully")
        else:
            print("✗ Failed to set zero position")
            return

        # Step 4: Set to lift up position
        print("\n=== Step 4: Setting Lift Up Position ===")
        print("This process takes about 3 seconds...")
        if trajectory_controller.set_lift_up_position(duration=3.0):
            print("✓ Lift up position set successfully")
        else:
            print("✗ Failed to set lift up position")
            return

        print("Activating algorithm control...")
        if not robot.activate_algorithm_control():
            print("✗ Failed to activate algorithm control")
            return
        print("✓ Activated")

        print("Starting MPC...")
        if not mpc.start_mpc():
            print("✗ Failed to start MPC")
            return
        print("✓ MPC started")

        status = mpc.query_mpc()
        print("MPC status:", status)

        # Build a ServoPose message; use Q5-specific parameters if model is Q5
        msg = ServoPose()
        if model == "Q5":
            msg.left_pose = make_pose_stamped(0.257, -0.346, 1.412, 0.612, -0.016, 0.79, -0.006)
            msg.right_pose = make_pose_stamped(-0.017, -0.208, 0.777, 0.996, -0.010, 0.070, 0.045)
            msg.head_pose = make_pose_stamped(-0.101, 0.000, 1.411, -0.000, 0.003, -0.000, 0.999)
        else:
            # Generic/default example pose
            msg.left_pose = make_pose_stamped(0.425, 0.3, 0.292, 0.612, -0.016, 0.79, -0.006)
            msg.right_pose = make_pose_stamped(0.417, -0.146, 0.382, 0.521, 0.1, 0.842, -0.096)
            msg.head_pose = make_pose_stamped(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        print("Publishing ServoPose at 80Hz for 3s...")
        mpc.publish_servo_pose(msg, rate_hz=80.0, duration=3.0)

        print("Stopping MPC...")
        mpc.stop_mpc()

        print("Deactivating algorithm control...")
        robot.deactivate_algorithm_control()

        print("Demo completed")

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        try:
            if robot:
                robot.shutdown()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()



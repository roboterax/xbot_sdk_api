#!/usr/bin/env python3
"""
Basic Robot Control Example
Demonstrates basic robot initialization and control using the robot_controller library.
"""

import rclpy
import sys
import os
import time

# Add the parent directory to the path to import robot_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_controller import RobotController, TrajectoryController


def main():
    """Main function demonstrating basic robot control."""
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot controller
        print("Creating robot controller...")
        robot = RobotController("basic_robot_control")
        
        # Create specialized controllers
        # Default model is Q5. For L3 or L7, change "Q5" to "L3" or "L7".
        trajectory_controller = TrajectoryController(robot, "Q5")
        
        print("Robot controller created successfully!")
        
        # Step 1: Start joint service
        print("\n=== Step 1: Starting Joint Service ===")
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
        
        # Step 4: Get joint information
        print("\n=== Step 4: Getting Joint Information ===")
        time.sleep(2.0)
        
        # Get all joints information
        joints_info = robot.get_all_joints_info()
        if joints_info:
            print(f"\nAvailable joints: {[joint['name'] for joint in joints_info]}")
            
            # Check first joint
            first_joint = joints_info[0]
            print(f"\nFirst joint ({first_joint['name']}) info:")
            print(f"  Position: {first_joint['position']:.3f} rad")
            print(f"  Velocity: {first_joint['velocity']:.3f} rad/s")
            print(f"  Effort: {first_joint['effort']:.3f} Nm")
        
        # Step 5: Set lift up position
        print("\n=== Step 5: Setting Lift Up Position ===")
        if trajectory_controller.set_lift_up_position(duration=3.0):
            print("✓ Lift up position set successfully")
        else:
            print("✗ Failed to set lift up position")
        
        # Step 6: Move to specific joint positions using the new API
        print("\n=== Step 6: Move To Specific Joint Positions ===")
        target = {}
        joints_info = robot.get_all_joints_info()
        if joints_info and len(joints_info) >= 2:
            # Example: move first two joints slightly
            target[joints_info[0]['name']] = joints_info[0]['position'] + 0.2
            target[joints_info[1]['name']] = joints_info[1]['position'] - 0.2
            if trajectory_controller.move_to_joint_positions(target, duration=2.0):
                print("✓ Moved to target joint positions")
            else:
                print("✗ Failed to move to target joint positions")
        else:
            print("No sufficient joint names available to demonstrate move")

        # Step 7: Final joint status
        print("\n=== Step 7: Final Joint Status ===")
        final_joints_info = robot.get_all_joints_info()
        if final_joints_info:
            print("Final joint positions:")
            for joint in final_joints_info:
                print(f"  {joint['name']}: {joint['position']:.3f} rad")
        
        print("\n=== Basic Robot Control Demo Completed Successfully! ===")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error during demo: {str(e)}")
    finally:
        # Cleanup
        try:
            if 'robot' in locals():
                robot.shutdown()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Base Control Demo
专门演示底盘控制功能的独立示例。
演示前进+转向组合运动。
"""

import rclpy
import sys
import os
import time

# Add the parent directory to the path to import robot_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_controller import RobotController, TrajectoryController


def main():
    """Main function demonstrating base control functionality."""
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot controller
        print("Creating robot controller for base control demo...")
        robot = RobotController("base_control_demo")
        
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
        
        # Step 4: Set lift up position (to prepare for movement)
        print("\n=== Step 4: Setting Lift Up Position ===")
        if trajectory_controller.set_lift_up_position(duration=3.0):
            print("✓ Lift up position set successfully")
        else:
            print("✗ Failed to set lift up position")
        
        # Step 5: Activate algorithm control
        print("\n=== Step 5: Activating Algorithm Control ===")
        if robot.activate_algorithm_control():
            print("✓ Algorithm control activated successfully")
        else:
            print("✗ Failed to activate algorithm control")
            return
        
        # Step 6: Base movement example
        print("\n=== Step 6: Base Movement Example ===")
        print("Moving forward while turning (linear_x=0.2 m/s, angular_z=0.5 rad/s)...")
        if robot.control_base_drive(linear_x=0.2, angular_z=0.5, frequency=10.0, duration=1.0):
            print("✓ Base movement command sent successfully")
        else:
            print("✗ Failed to send base movement command")
        
        # Stop
        print("\nStopping...")
        robot.stop_base_drive()
        
        print("\n=== Base Control Demo Completed Successfully! ===")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        # Emergency stop
        try:
            if 'robot' in locals():
                robot.stop_base_drive()
                print("Emergency stop executed")
        except:
            pass
    except Exception as e:
        print(f"Error during demo: {str(e)}")
        # Emergency stop
        try:
            if 'robot' in locals():
                robot.stop_base_drive()
                print("Emergency stop executed")
        except:
            pass
    finally:
        # Cleanup
        try:
            if 'robot' in locals():
                robot.stop_base_drive()  # Ensure robot is stopped
                robot.shutdown()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

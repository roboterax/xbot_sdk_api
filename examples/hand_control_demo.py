#!/usr/bin/env python3
"""
Hand Control Demo
专门演示手部控制功能的独立示例。
包括XHand Lite、XHand抓取动作和自定义手部关节控制。
"""

import rclpy
import sys
import os
import time

# Add the parent directory to the path to import robot_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_controller import RobotController, TrajectoryController


def main():
    """Main function demonstrating hand control functionality."""
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot controller
        print("Creating robot controller for hand control demo...")
        robot = RobotController("hand_control_demo")
        
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
        
        # Step 4: Set lift up position (to prepare for hand control)
        print("\n=== Step 4: Setting Lift Up Position ===")
        if trajectory_controller.set_lift_up_position(duration=3.0):
            print("✓ Lift up position set successfully")
        else:
            print("✗ Failed to set lift up position")
        
        # Step 5: XHand Lite Control Tests
        print("\n=== Step 5: XHand Lite Control Tests ===")
        
        # Test 1: XHand Lite gentle grasp
        print("\n--- Test 1: XHand Lite Gentle Grasp ---")
        print("Controlling XHand Lite to perform gentle grasp...")
        if robot.control_xhand_lite_grasp():
            print("✓ XHand Lite grasp command sent successfully")
            print("Waiting 3 seconds to observe hand movement...")
            time.sleep(3.0)
        else:
            print("✗ Failed to send XHand Lite grasp command")
        
        # Test 2: XHand Lite with custom parameters
        print("\n--- Test 2: XHand Lite Custom Parameters ---")
        print("Controlling XHand Lite with custom parameters...")
        
        # XHand Lite joint names
        xhand_lite_joints = [
            'left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1', 
            'left_hand_index_joint1', 'left_hand_mid_joint1', 
            'left_hand_ring_joint1', 'left_hand_pinky_joint1',
            'right_hand_thumb_bend_joint', 'right_hand_thumb_rota_joint1', 
            'right_hand_index_joint1', 'right_hand_mid_joint1', 
            'right_hand_ring_joint1', 'right_hand_pinky_joint1'
        ]
        
        # Custom parameters for stronger grasp
        custom_positions = [0.8, 0.8, 1.2, 1.2, 1.2, 1.2,
                           0.8, 0.8, 1.2, 1.2, 1.2, 1.2]
        custom_velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                            0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        custom_feedforward = [400.0, 400.0, 400.0, 400.0, 400.0, 400.0,
                             400.0, 400.0, 400.0, 400.0, 400.0, 400.0]
        custom_kp = [120.0, 120.0, 120.0, 120.0, 120.0, 120.0,
                     120.0, 120.0, 120.0, 120.0, 120.0, 120.0]
        custom_kd = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                     5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        
        if robot.control_hand_joints(
            joint_names=xhand_lite_joints,
            positions=custom_positions,
            velocities=custom_velocities,
            feedforward=custom_feedforward,
            kp=custom_kp,
            kd=custom_kd
        ):
            print("✓ XHand Lite custom control command sent successfully")
            print("Waiting 3 seconds to observe hand movement...")
            time.sleep(3.0)
        else:
            print("✗ Failed to send XHand Lite custom control command")
        
        # Step 6: XHand Control Tests
        print("\n=== Step 6: XHand Control Tests ===")
        
        # Test 3: XHand gentle grasp
        print("\n--- Test 3: XHand Gentle Grasp ---")
        print("Controlling XHand to perform gentle grasp...")
        if robot.control_xhand_grasp():
            print("✓ XHand grasp command sent successfully")
            print("Waiting 3 seconds to observe hand movement...")
            time.sleep(3.0)
        else:
            print("✗ Failed to send XHand grasp command")
        
        # Test 4: XHand with custom parameters
        print("\n--- Test 4: XHand Custom Parameters ---")
        print("Controlling XHand with custom parameters...")
        
        # XHand joint names
        xhand_joints = [
            'left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1', 
            'left_hand_thumb_rota_joint2', 'left_hand_index_bend_joint', 
            'left_hand_index_joint1', 'left_hand_index_joint2',
            'left_hand_mid_joint1', 'left_hand_mid_joint2', 
            'left_hand_ring_joint1', 'left_hand_ring_joint2', 
            'left_hand_pinky_joint1', 'left_hand_pinky_joint2',
            'right_hand_thumb_bend_joint', 'right_hand_thumb_rota_joint1', 
            'right_hand_thumb_rota_joint2', 'right_hand_index_bend_joint', 
            'right_hand_index_joint1', 'right_hand_index_joint2',
            'right_hand_mid_joint1', 'right_hand_mid_joint2', 
            'right_hand_ring_joint1', 'right_hand_ring_joint2', 
            'right_hand_pinky_joint1', 'right_hand_pinky_joint2'
        ]
        
        # Custom parameters for different grasp pattern
        custom_positions = [0.3, 0.3, 0.8, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8,
                           0.3, 0.3, 0.8, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
        custom_velocities = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
                            0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        custom_feedforward = [300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0,
                              300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0]
        custom_kp = [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0,
                     80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
        custom_kd = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                     2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        
        if robot.control_hand_joints(
            joint_names=xhand_joints,
            positions=custom_positions,
            velocities=custom_velocities,
            feedforward=custom_feedforward,
            kp=custom_kp,
            kd=custom_kd
        ):
            print("✓ XHand custom control command sent successfully")
            print("Waiting 3 seconds to observe hand movement...")
            time.sleep(3.0)
        else:
            print("✗ Failed to send XHand custom control command")
        
        # Step 7: Individual Finger Control Tests
        print("\n=== Step 7: Individual Finger Control Tests ===")
        
        # Test 5: Left thumb control
        print("\n--- Test 5: Left Thumb Control ---")
        print("Controlling left thumb joints individually...")
        
        left_thumb_joints = ['left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1']
        left_thumb_positions = [0.9, 0.9]  # More bent
        left_thumb_velocities = [0.2, 0.2]  # Faster movement
        left_thumb_feedforward = [500.0, 500.0]  # Higher force
        left_thumb_kp = [150.0, 150.0]  # Higher proportional gain
        left_thumb_kd = [10.0, 10.0]  # Higher derivative gain
        
        if robot.control_hand_joints(
            joint_names=left_thumb_joints,
            positions=left_thumb_positions,
            velocities=left_thumb_velocities,
            feedforward=left_thumb_feedforward,
            kp=left_thumb_kp,
            kd=left_thumb_kd
        ):
            print("✓ Left thumb control command sent successfully")
            print("Waiting 2 seconds to observe thumb movement...")
            time.sleep(2.0)
        else:
            print("✗ Failed to send left thumb control command")
        
        # Test 6: Right index finger control
        print("\n--- Test 6: Right Index Finger Control ---")
        print("Controlling right index finger joints...")
        
        right_index_joints = ['right_hand_index_joint1']
        right_index_positions = [1.5]  # More extended
        right_index_velocities = [0.15, 0.15]  # Medium speed
        right_index_feedforward = [350.0]  # Standard force
        right_index_kp = [100.0]  # Standard proportional gain
        right_index_kd = [5.0]  # Standard derivative gain
        
        if robot.control_hand_joints(
            joint_names=right_index_joints,
            positions=right_index_positions,
            velocities=right_index_velocities,
            feedforward=right_index_feedforward,
            kp=right_index_kp,
            kd=right_index_kd
        ):
            print("✓ Right index finger control command sent successfully")
            print("Waiting 2 seconds to observe finger movement...")
            time.sleep(2.0)
        else:
            print("✗ Failed to send right index finger control command")
        
        # Step 8: Sequential Finger Movement
        print("\n=== Step 8: Sequential Finger Movement ===")
        print("Demonstrating sequential finger movement...")
        
        # Define finger sequences
        finger_sequences = [
            ("Left Thumb", ['left_hand_thumb_bend_joint'], [0.2]),
            ("Left Index", ['left_hand_index_joint1'], [0.3]),
            ("Left Middle", ['left_hand_mid_joint1'], [0.4]),
            ("Left Ring", ['left_hand_ring_joint1'], [0.5]),
            ("Left Pinky", ['left_hand_pinky_joint1'], [0.6]),
            ("Right Thumb", ['right_hand_thumb_bend_joint'], [0.2]),
            ("Right Index", ['right_hand_index_joint1'], [0.3]),
            ("Right Middle", ['right_hand_mid_joint1'], [0.4]),
            ("Right Ring", ['right_hand_ring_joint1'], [0.5]),
            ("Right Pinky", ['right_hand_pinky_joint1'], [0.6])
        ]
        
        for finger_name, joint_names, positions in finger_sequences:
            print(f"  Moving {finger_name}...")
            if robot.control_hand_joints(
                joint_names=joint_names,
                positions=positions,
                velocities=[0.1],
                feedforward=[300.0],
                kp=[100.0],
                kd=[3.0]
            ):
                print(f"  ✓ {finger_name} movement command sent")
                time.sleep(0.5)  # Short delay between movements
            else:
                print(f"  ✗ Failed to send {finger_name} movement command")
        
        # Step 9: Final hand position
        print("\n=== Step 9: Final Hand Position ===")
        print("Setting hands to final relaxed position...")
        
        # Return to gentle grasp position
        if robot.control_xhand_lite_grasp():
            print("✓ Final hand position set successfully")
            print("Waiting 2 seconds to observe final position...")
            time.sleep(2.0)
        else:
            print("✗ Failed to set final hand position")
        
        # Step 10: Final status
        print("\n=== Step 10: Final Status ===")
        final_joints_info = robot.get_all_joints_info()
        if final_joints_info:
            print("Final joint positions:")
            # Show hand-related joints
            hand_joints = [joint for joint in final_joints_info if 'hand' in joint['name']]
            for joint in hand_joints[:10]:  # Show first 10 hand joints
                print(f"  {joint['name']}: {joint['position']:.3f} rad")
            if len(hand_joints) > 10:
                print(f"  ... and {len(hand_joints) - 10} more hand joints")
        
        print("\n=== Hand Control Demo Completed Successfully! ===")
        print("The robot has completed all hand control tests.")
        
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


























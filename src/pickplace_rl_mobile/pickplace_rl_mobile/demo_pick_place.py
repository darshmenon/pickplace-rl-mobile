#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Publishers
        self.shoulder_pub = self.create_publisher(Float64, '/shoulder_joint/cmd_vel', 10)
        self.shoulder_pitch_pub = self.create_publisher(Float64, '/shoulder_pitch_joint/cmd_vel', 10)
        self.elbow_pub = self.create_publisher(Float64, '/elbow_joint/cmd_vel', 10)
        self.wrist_roll_pub = self.create_publisher(Float64, '/wrist_roll_joint/cmd_vel', 10)
        self.wrist_pitch_pub = self.create_publisher(Float64, '/wrist_pitch_joint/cmd_vel', 10)
        self.left_finger_pub = self.create_publisher(Float64, '/left_finger_joint/cmd_vel', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/right_finger_joint/cmd_vel', 10)
        
        # Give publishers time to connect
        time.sleep(1.0)
        self.get_logger().info('Starting pick and place sequence...')
        
    def move_joint(self, pub, velocity, duration):
        msg = Float64()
        msg.data = velocity
        
        # Send velocity commands for the duration
        start_time = time.time()
        while time.time() - start_time < duration:
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
        # Stop joint
        msg.data = 0.0
        pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        
    def execute_sequence(self):
        # 1. Open gripper
        self.get_logger().info('Opening gripper...')
        self.move_joint(self.left_finger_pub, 0.5, 1.0)
        self.move_joint(self.right_finger_pub, 0.5, 1.0)
        
        # 2. Lower arm towards object
        self.get_logger().info('Lowering arm...')
        self.move_joint(self.shoulder_pitch_pub, 0.5, 1.5)
        self.move_joint(self.elbow_pub, -0.3, 1.0)
        
        # 3. Close gripper (grasp)
        self.get_logger().info('Grasping object...')
        self.move_joint(self.left_finger_pub, -0.5, 1.5)
        self.move_joint(self.right_finger_pub, -0.5, 1.5)
        
        # 4. Lift arm with object
        self.get_logger().info('Lifting object...')
        self.move_joint(self.shoulder_pitch_pub, -0.6, 1.5)
        self.move_joint(self.elbow_pub, 0.3, 1.0)
        
        # 5. Rotate shoulder to target
        self.get_logger().info('Moving to target...')
        self.move_joint(self.shoulder_pub, 0.5, 2.0)
        
        # 6. Lower arm to target bin
        self.get_logger().info('Lowering to target...')
        self.move_joint(self.shoulder_pitch_pub, 0.4, 1.0)
        
        # 7. Release object
        self.get_logger().info('Releasing object...')
        self.move_joint(self.left_finger_pub, 0.5, 1.0)
        self.move_joint(self.right_finger_pub, 0.5, 1.0)
        
        # 8. Return to home position
        self.get_logger().info('Returning to home...')
        self.move_joint(self.shoulder_pitch_pub, -0.4, 1.0)
        self.move_joint(self.shoulder_pub, -0.5, 2.0)
        
        self.get_logger().info('Sequence complete!')

def main(args=None):
    rclpy.init(args=args)
    demo = PickPlaceDemo()
    
    try:
        demo.execute_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

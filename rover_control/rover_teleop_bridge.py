#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        
        # Subscribe to the keyboard commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
            
        # Publisher to  rover controller
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/rover_controller/joint_trajectory', 
            10)

        # Joint Names 
        self.steer_joints = ['steer1', 'steer2', 'steer_3', 'steer4']
        self.wheel_joints = ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']
        
        # Internal state
        self.wheel_position = 0.0

    def listener_callback(self, msg):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.steer_joints + self.wheel_joints
        
        point = JointTrajectoryPoint()
        
        # 1. STEERING (Angular Z -> Joint Angle)
        # Limit steering to approx 45 degrees (0.78 rad)
        steer_angle = msg.angular.z 
        if steer_angle > 0.8: steer_angle = 0.8
        if steer_angle < -0.8: steer_angle = -0.8
        
        # 2. DRIVING (Linear X -> Continuous Wheel Rotation)
        # We simulate "velocity" by constantly increasing the position goal
        speed_factor = 2.0 # Adjust to go faster/slower
        self.wheel_position += (msg.linear.x * speed_factor)
        
        # Create the position command list
        # Steers get set directly, Wheels get incremented
        steer_cmds = [steer_angle] * 4 
        wheel_cmds = [self.wheel_position] * 4
        
        point.positions = steer_cmds + wheel_cmds
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 250000000 # 0.25 seconds response
        
        trajectory_msg.points = [point]
        self.publisher_.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


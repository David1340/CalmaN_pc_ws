#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, LaserScan
import numpy as np

class Navigator(Node):
    def __init__(self):
        super().__init__('example_navigator')
        self.get_logger().info('Navigator node has been started.')
        self.cmd_vel = Twist()
        self.encoder_data = np.array([])
        self.scanAngles = np.array([])
        self.scanDistances = np.array([])
        self.imuLinearAcceleration = np.array([0.0, 0.0, 0.0])
        self.imuAngularVelocity = np.array([0.0, 0.0, 0.0])
        # Publishers (cmd_vel)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'robot/cmd_vel', 
            2)
        
        # Subscrivers (encoder, scan, imu)
        self.encoder_subscriber = self.create_subscriber(
            Float32MultiArray,
            '/robot/encoder',
            self.encoder_callback,
            2
        )

        self.laserScan_subscriber = self.create_subscriber(
            LaserScan,
            '/robot/scan',
            self.scan_callback,
            2
        )

        self.imu_subscriber = self.create_subscriber(
            Imu,
            '/robot/imu',
            self.imu_callback,
            2
        )

        # Cria um timer para chamar a estratégia de navegação
        self.timer = self.create_timer(10e-1, self.navigate)

    def encoder_callback(self, msg):
        #   self.get_logger().info(f'encoder_callback: {msg.data}')
        self.encoder_data = np.array(msg.data)
    
    def scan_callback(self, msg):
        #self.get_logger().info(f'scan_callback: {msg.ranges}')
        distances = np.array(msg.ranges)
        valid = np.where(distances > 0)
        self.scanAngles = np.linspace(msg.angle_min, msg.angle_max, len(distances))[valid]
        self.scanDistances = distances[valid]
    
    def imu_callback(self, msg):
        #self.get_logger().info(f'imu_callback: {msg.linear_acceleration} {msg.angular_velocity}')
        self.imuLinearAcceleration = np.array(msg.linear_acceleration)
        self.imuAngularVelocity = np.array(msg.angular_velocity)
    
    

    def navigate(self):
        # use the following variables to implement your navigation algorithm
        # self.encoder_data, self.scanAngles, self.scanDistances, self.imuLinearAcceleration, self.imuAngularVelocity
        # publish velocity commands using self.cmd_vel_publisher.publish(twist)
            
        # Se há algum obstáculo próximo (ex: < 0.5 m), gira
        if len(self.scanDistances) > 0 and np.min(self.scanDistances) < 0.5:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.5
        else:
            self.cmd_vel.linear.x = 0.1
            self.cmd_vel.angular.z = 0.0

        if self.scanDistances.size == 0:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
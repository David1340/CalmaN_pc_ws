#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, LaserScan
import numpy as np
import time
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
        self.encoder_subscriber = self.create_subscription(
            Float32MultiArray,
            '/robot/encoder',
            self.encoder_callback,
            2
        )

        self.laserScan_subscriber = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.scan_callback,
            2
        )

        self.imu_subscriber = self.create_subscription(
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
    
    def stop(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def navigate(self):
        # use the following variables to implement your navigation algorithm
        # self.encoder_data, self.scanAngles, self.scanDistances, self.imuLinearAcceleration, self.imuAngularVelocity
        # publish velocity commands using self.cmd_vel_publisher.publish(twist)
            
        # Considera apenas obstáculos na frente (ex: ±30° do ângulo 0)
        if len(self.scanDistances) > 0:
            # Supondo que o scan seja em radianos, com ângulo 0 na frente
            # e que self.scanAngles contenha os ângulos correspondentes:
            front_mask = (self.scanAngles > -0.52) & (self.scanAngles < 0.52)  # -30° a +30°
            front_distances = self.scanDistances[front_mask]

            if front_distances.size > 0 and np.min(front_distances) < 0.5:
                # Obstáculo na frente → para e gira
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.5
            else:
                # Livre na frente → segue em frente
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0.0
        else:
            # Nenhum dado do laser → para
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
        # Garante que o robô pare antes de destruir o nó e desligar o ROS
        try:
            node.stop()   # publica velocidade zero
            time.sleep(0.2)  # dá tempo da mensagem chegar ao robô
        except Exception as e:
            print(f"[WARN] Falha ao parar o robô com segurança: {e}")

        # Agora sim, libera recursos do nó e encerra o ROS
        node.destroy_node()
        rclpy.shutdown()
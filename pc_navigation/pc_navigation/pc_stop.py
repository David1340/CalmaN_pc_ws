#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = Node('stop_robot')
    try:
        # Cria o publisher
        cmd_vel_publisher = node.create_publisher(Twist, 'robot/cmd_vel', 10)
        
        # Publica comando de parada
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        node.get_logger().info('Stopping the robot...')
        cmd_vel_publisher.publish(stop_msg)
        node.get_logger().info('Robot stopped.')
        
        # Pequena pausa para garantir que a mensagem seja enviada
        rclpy.spin_once(node, timeout_sec=0.1)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

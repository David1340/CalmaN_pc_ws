import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class VelocityPlotter(Node):

    def __init__(self):
        super().__init__('velocity_plotter')
        
        # Subscrever velocidade enviada
        self.cmd_sub = self.create_subscription(Twist, '/robot/cmd_vel', self.cmd_callback, 10)
        # Subscrever velocidade medida
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Dados
        self.cmd_data_linear = []
        self.cmd_data_angular = []
        self.odom_data_linear = []
        self.odom_data_angular = []

        # Plot setup
        plt.ion()
        self.fig, (self.ax_linear, self.ax_angular) = plt.subplots(2, 1, figsize=(8,6))
        self.ax_linear.set_title('Velocidade Linear: Enviada vs Medida')
        self.ax_linear.set_ylabel('Linear [m/s]')
        self.ax_linear.grid(True)

        self.ax_angular.set_title('Velocidade Angular: Enviada vs Medida')
        self.ax_angular.set_xlabel('Amostras')
        self.ax_angular.set_ylabel('Angular [rad/s]')
        self.ax_angular.grid(True)

    def cmd_callback(self, msg: Twist):
        self.cmd_data_linear.append(msg.linear.x)
        self.cmd_data_angular.append(msg.angular.z)
        self.update_plot()

    def odom_callback(self, msg: Odometry):
        self.odom_data_linear.append(msg.twist.twist.linear.x)
        self.odom_data_angular.append(msg.twist.twist.angular.z)
        self.update_plot()

    def update_plot(self):
        # Linear
        self.ax_linear.clear()
        self.ax_linear.plot(self.cmd_data_linear, '-b', label='Linear enviada')
        self.ax_linear.plot(self.odom_data_linear, '--b', label='Linear medida')
        self.ax_linear.set_title('Velocidade Linear: Enviada vs Medida')
        self.ax_linear.set_ylabel('Linear [m/s]')
        self.ax_linear.grid(True)
        self.ax_linear.legend()

        # Angular
        self.ax_angular.clear()
        self.ax_angular.plot(self.cmd_data_angular, '-g', label='Angular enviada')
        self.ax_angular.plot(self.odom_data_angular, '--g', label='Angular medida')
        self.ax_angular.set_title('Velocidade Angular: Enviada vs Medida')
        self.ax_angular.set_xlabel('Amostras')
        self.ax_angular.set_ylabel('Angular [rad/s]')
        self.ax_angular.grid(True)
        self.ax_angular.legend()

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

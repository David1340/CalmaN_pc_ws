import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import math

class PositionPlotter(Node):

    def __init__(self):
        super().__init__('position_plotter')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/pose_topic', self.pose_callback, 10)

        # Data storage
        self.odom_data = {'x': [], 'y': [], 'theta': []}
        self.pose_data = {'x': [], 'y': [], 'theta': []}

        # Plot setup
        plt.ion()
        self.fig, (self.ax_x, self.ax_y, self.ax_theta) = plt.subplots(3, 1, figsize=(8, 8))
        self.ax_x.set_title('Posição X: Odometria vs Pose')
        self.ax_x.set_ylabel('X [m]')
        self.ax_x.grid(True)

        self.ax_y.set_title('Posição Y: Odometria vs Pose')
        self.ax_y.set_ylabel('Y [m]')
        self.ax_y.grid(True)

        self.ax_theta.set_title('Orientação Theta: Odometria vs Pose')
        self.ax_theta.set_xlabel('Amostras')
        self.ax_theta.set_ylabel('Theta [rad]')
        self.ax_theta.grid(True)

    # Callbacks
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Extrair yaw do quaternion
        q = msg.pose.pose.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        self.odom_data['x'].append(x)
        self.odom_data['y'].append(y)
        self.odom_data['theta'].append(theta)
        self.update_plot()

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        self.pose_data['x'].append(x)
        self.pose_data['y'].append(y)
        self.pose_data['theta'].append(theta)
        self.update_plot()

    def update_plot(self):
        samples = range(max(len(self.odom_data['x']), 1))

        # X
        self.ax_x.clear()
        self.ax_x.plot(self.odom_data['x'], '-b', label='Odometria')
        self.ax_x.plot(self.pose_data['x'], '-r', label='Pose')
        self.ax_x.set_ylabel('X [m]')
        self.ax_x.set_title('Posição X: Odometria vs Pose')
        self.ax_x.grid(True)
        self.ax_x.legend()

        # Y
        self.ax_y.clear()
        self.ax_y.plot(self.odom_data['y'], '-b', label='Odometria')
        self.ax_y.plot(self.pose_data['y'], '-r', label='Pose')
        self.ax_y.set_ylabel('Y [m]')
        self.ax_y.set_title('Posição Y: Odometria vs Pose')
        self.ax_y.grid(True)
        self.ax_y.legend()

        # Theta
        self.ax_theta.clear()
        self.ax_theta.plot(self.odom_data['theta'], '-b', label='Odometria')
        self.ax_theta.plot(self.pose_data['theta'], '-r', label='Pose')
        self.ax_theta.set_xlabel('Amostras')
        self.ax_theta.set_ylabel('Theta [rad]')
        self.ax_theta.set_title('Orientação Theta: Odometria vs Pose')
        self.ax_theta.grid(True)
        self.ax_theta.legend()

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = PositionPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

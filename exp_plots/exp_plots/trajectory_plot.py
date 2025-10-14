import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Subscrever odometria
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Subscrever posição absoluta da câmera
        self.vision_sub = self.create_subscription(PoseStamped, '/pose_topic', self.vision_callback, 10)
        
        # Dados
        self.odom_data = []
        self.vision_data = []

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Trajetória: Odometria vs Visão')
        self.ax.grid(True)
        self.ax.axis('equal')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_data.append([x, y])
        self.update_plot()

    def vision_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.vision_data.append([x, y])
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        if self.odom_data:
            odom_arr = self.odom_data
            self.ax.plot([p[0] for p in odom_arr], [p[1] for p in odom_arr], '-b', label='Odometria')
        if self.vision_data:
            vis_arr = self.vision_data
            self.ax.plot([p[0] for p in vis_arr], [p[1] for p in vis_arr], '-r', label='Visão')
        self.ax.legend()
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Trajetória: Odometria vs Visão')
        self.ax.grid(True)
        self.ax.axis('equal')
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

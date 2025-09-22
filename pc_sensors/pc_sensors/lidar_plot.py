import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LidarPlotter(Node):
    def __init__(self):
        super().__init__('lidar_plotter')
        self.create_subscription(LaserScan, '/robot/scan', self.lidar_callback, 2)
        self.laser_scan_data = None
        self.get_logger().info('Plot do LIDAR iniciado')

        # Inicializa o plot uma vez
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.plot([], [], 'bo')[0]  # retorna uma linha/plot handle
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Leituras do LIDAR')
        self.ax.axis('equal')
        self.ax.grid(True)
        self.ax.set_xlim([-8, 8])
        self.ax.set_ylim([-8, 8])
        plt.ion()  # modo interativo

    def lidar_callback(self, msg):
        distances = np.array(msg.ranges)
        valid = np.where(distances > 0)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(distances))[valid]
        distances = distances[valid]

        x = distances * np.cos(angles + (np.pi / 2.0))
        y = distances * np.sin(angles + (np.pi / 2.0))

        # Atualiza os dados do scatter plot existente
        self.sc.set_data(x, y)
        plt.pause(0.1)

def main():
    rclpy.init()
    node = LidarPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

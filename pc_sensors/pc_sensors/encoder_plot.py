import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque

class EncoderPlotter(Node):
    def __init__(self):
        super().__init__('encoder_plotter')
        self.create_subscription(Float32MultiArray, '/robot/encoder', self.callback, 2)
        self.left = deque(maxlen=200)
        self.right = deque(maxlen=200)
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.get_logger().info('Plot das velocidades iniciado')

    def callback(self, msg):
        self.left.append(msg.data[2])  # Velocidade motor esquerdo
        self.right.append(msg.data[3]) # Velocidade motor direito
        self.ax.clear()
        self.ax.plot(self.left, label='Esquerda')
        self.ax.plot(self.right, label='Direita')
        self.ax.legend()
        self.ax.set_title('Velocidades das rodas (últimos 5s)')
        plt.pause(0.1)

def main():
    rclpy.init()
    node = EncoderPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

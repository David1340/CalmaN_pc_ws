import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import time


class OdomFrequencyPlotter(Node):
    def __init__(self):
        super().__init__('odom_frequency_plotter')

        # Variáveis de controle
        self.last_time = None
        self.freq_history = []
        self.time_history = []

        # Assinatura do tópico /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Inicializa gráfico interativo
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.ax.set_title('Evolução Temporal da Frequência de Publicação - /odom')
        self.ax.set_xlabel('Tempo (s)')
        self.ax.set_ylabel('Frequência (Hz)')
        self.ax.grid(True)
        self.start_time = time.time()

        # Timer para atualizar o gráfico a cada 0.5 s
        self.timer = self.create_timer(0.5, self.update_plot)

    def odom_callback(self, msg):
        now = time.time()
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                freq = 1.0 / dt
                self.freq_history.append(freq)
                self.time_history.append(now - self.start_time)
        self.last_time = now

    def update_plot(self):
        if len(self.time_history) < 2:
            return

        self.ax.clear()
        self.ax.set_title('Evolução Temporal da Frequência de Publicação - /odom')
        self.ax.set_xlabel('Tempo (s)')
        self.ax.set_ylabel('Frequência (Hz)')
        self.ax.grid(True)

        self.ax.plot(self.time_history, self.freq_history, label='/odom', color='tab:blue')
        self.ax.legend()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFrequencyPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

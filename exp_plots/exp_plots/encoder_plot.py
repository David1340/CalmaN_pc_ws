import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt


class WheelPlotter(Node):

    def __init__(self):
        super().__init__('wheel_plotter')

        # Subscrição do tópico dos encoders
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/robot/encoder', self.encoder_callback, 10)

        # Armazenamento dos dados
        self.pos_left = []
        self.pos_right = []
        self.vel_left = []
        self.vel_right = []

        # Configuração das figuras
        plt.ion()

        # Figura 1 - Velocidades
        self.fig_vel, (self.ax_vl, self.ax_vr) = plt.subplots(2, 1, figsize=(7, 6))
        self.fig_vel.suptitle('Velocidade das Rodas (Encoder)')
        self.ax_vl.set_ylabel('Vel. Esquerda [rad/s]')
        self.ax_vl.grid(True)
        self.ax_vr.set_ylabel('Vel. Direita [rad/s]')
        self.ax_vr.set_xlabel('Amostras')
        self.ax_vr.grid(True)

        # Figura 2 - Posições
        self.fig_pos, (self.ax_pl, self.ax_pr) = plt.subplots(2, 1, figsize=(7, 6))
        self.fig_pos.suptitle('Posição Angular das Rodas (Encoder)')
        self.ax_pl.set_ylabel('Pos. Esquerda [contagens]')
        self.ax_pl.grid(True)
        self.ax_pr.set_ylabel('Pos. Direita [contagens]')
        self.ax_pr.set_xlabel('Amostras')
        self.ax_pr.grid(True)

    def encoder_callback(self, msg: Float32MultiArray):
        # Estrutura: [posE, posD, velE, velD]
        if len(msg.data) < 4:
            return

        self.pos_left.append(msg.data[0])
        self.pos_right.append(msg.data[1])
        self.vel_left.append(msg.data[2])
        self.vel_right.append(msg.data[3])

        self.update_plots()

    def update_plots(self):
        # Atualiza figura 1 - Velocidades
        self.ax_vl.clear()
        self.ax_vr.clear()
        self.ax_vl.plot(self.vel_left, '-b', label='Esquerda')
        self.ax_vr.plot(self.vel_right, '-r', label='Direita')
        self.ax_vl.set_title('Velocidade Esquerda')
        self.ax_vr.set_title('Velocidade Direita')
        self.ax_vl.set_ylabel('Vel [rad/s]')
        self.ax_vr.set_ylabel('Vel [rad/s]')
        self.ax_vr.set_xlabel('Amostras')
        self.ax_vl.grid(True)
        self.ax_vr.grid(True)

        # Atualiza figura 2 - Posições
        self.ax_pl.clear()
        self.ax_pr.clear()
        self.ax_pl.plot(self.pos_left, '-b', label='Esquerda')
        self.ax_pr.plot(self.pos_right, '-r', label='Direita')
        self.ax_pl.set_title('Posição Esquerda')
        self.ax_pr.set_title('Posição Direita')
        self.ax_pl.set_ylabel('Pos [rad]')
        self.ax_pr.set_ylabel('Pos [rad]')
        self.ax_pr.set_xlabel('Amostras')
        self.ax_pl.grid(True)
        self.ax_pr.grid(True)

        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = WheelPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import time

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')

        # Buffer para armazenar as poses
        self.pose_data = []

        # Subscriber
        self.sub = self.create_subscription(
            PoseStamped,
            '/pose_topic',
            self.pose_callback,
            10
        )

        self.get_logger().info('Inscrito em /pose_topic. Aguardando dados...')

    def pose_callback(self, msg):
        pose_d = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.pose_data.append(pose_d)
        # Opcional: imprime para debug
        self.get_logger().info(f"Pose recebida: {pose_d}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()

    # Aguarda 5 segundos recebendo mensagens
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Fecha o nó
    node.destroy_subscription(node.sub)
    node.destroy_node()
    rclpy.shutdown()

    # Converte lista em array numpy
    data = np.array(node.pose_data).T  # shape (3, N)
    if data.size == 0:
        print("Nenhum dado recebido.")
        return

    # Extrai ângulo (Z)
    z_angle_deg = data[2, :] * 180 / np.pi

    # Estatísticas
    ang_mean = np.mean(z_angle_deg)
    ang_min = np.min(z_angle_deg)
    ang_max = np.max(z_angle_deg)

    print("\n Estatisticas do angulo (Z):")
    print(f"   Media: {ang_mean:.2f}°")
    print(f"   Minimo: {ang_min:.2f}°")
    print(f"   Maximo: {ang_max:.2f}°")

    # Plotagem
    plt.figure(figsize=(8,6))
    plt.subplot(3,1,1)
    plt.plot(data[0,:]*100)
    #plt.ylim([0,500])
    plt.grid(True)
    plt.ylabel("X (cm)")

    plt.subplot(3,1,2)
    plt.plot(data[1,:]*100)
    #plt.ylim([0,500])
    plt.grid(True)
    plt.ylabel("Y (cm)")

    plt.subplot(3,1,3)
    plt.plot(data[2,:]*180/np.pi)
    plt.ylim([-180,180])
    plt.grid(True)
    plt.ylabel("Z (graus)")
    plt.xlabel("Amostra")

    plt.tight_layout()
    plt.show()
    

if __name__ == '__main__':
    main()

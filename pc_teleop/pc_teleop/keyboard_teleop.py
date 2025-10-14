import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard  # pip install keyboard
import time

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.velPub = self.create_publisher(Twist, '/robot/cmd_vel', 2)

        # Velocidade linear e angular padrão
        Vmax = 25.2*(0.03)
        Wmax = 25.2*(0.03/0.173)
        self.linear_speed =Vmax
        self.angular_speed = 0.5*Wmax
        self.last_cmd_time = time.time()

        self.get_logger().info(
            "\nTeleop iniciado! Use as teclas:\n"
            "  W/S -> Frente/Tras\n"
            "  A/D -> Giro Esquerda/Direita\n"
            "  ESPAÇO -> Parar\n"
            "  Q -> Sair"
        )

        # Timer para enviar comandos a 20 Hz
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        twist = Twist()
        if keyboard.is_pressed('w'):
            twist.linear.x = self.linear_speed
        elif keyboard.is_pressed('s'):
            twist.linear.x = -self.linear_speed

        if keyboard.is_pressed('a'):
            twist.angular.z = self.angular_speed
        elif keyboard.is_pressed('d'):
            twist.angular.z = -self.angular_speed

        # Parar com espaço
        if keyboard.is_pressed('space'):
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publica apenas se algo mudou ou a cada ~0.5 s
        if (twist.linear.x != 0.0 or twist.angular.z != 0.0 or
            time.time() - self.last_cmd_time > 0.5):
            self.velPub.publish(twist)
            self.last_cmd_time = time.time()

        # Sair
        if keyboard.is_pressed('q'):
            self.get_logger().info("Encerrando teleop...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

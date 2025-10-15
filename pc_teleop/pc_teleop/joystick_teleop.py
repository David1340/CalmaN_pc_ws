import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # Publicador para enviar velocidades
        self.velPub = self.create_publisher(Twist, '/robot/cmd_vel', 10)

        # Inicializa pygame e joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error('Nenhum joystick detectado!')
            raise SystemExit

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'Usando joystick: {self.joystick.get_name()}')

        # Cria um timer para ler o joystick a ~300 Hz (3.33 ms)
        self.timer = self.create_timer(3.333e-2, self.timer_callback)

    def timer_callback(self):
        # Atualiza eventos do pygame
        pygame.event.pump()

        eixo_x = self.joystick.get_axis(0)  # Eixo X do analógico esquerdo
        eixo_y = self.joystick.get_axis(1)  # Eixo Y do analógico esquerdo

        self.get_logger().info(f'angular: {eixo_x:.2f} | linear: {-eixo_y:.2f}')

        # Publica velocidades no formato Twist
        cmd = Twist()
        vrmax = 25.2*0.5
        Vmax = vrmax*(0.03)
        Wmax = vrmax*(0.03/0.173)
        cmd.angular.z = -eixo_x*Wmax
        cmd.linear.x = -eixo_y*Vmax*(1-abs(eixo_x))
        self.velPub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

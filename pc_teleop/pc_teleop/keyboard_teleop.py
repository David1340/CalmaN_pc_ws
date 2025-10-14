import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard  # pip install pynput
import time

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.velPub = self.create_publisher(Twist, '/robot/cmd_vel', 2)


        Vmax = 25.2*(0.03)
        Wmax = 25.2*(0.03/0.173)
        self.linear_speed = Vmax
        self.angular_speed = 0.5*Wmax
        self.last_cmd_time = time.time()

        self.pressed_keys = set()

        self.get_logger().info(
            "\nTeleop iniciado! Use as teclas:\n"
            "  W/S -> Frente/Tras\n"
@@ -25,60 +24,35 @@ def __init__(self):
            "  Q -> Sair"
        )

        # Start listener de teclado
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.timer = self.create_timer(0.05, self.update)

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char.lower())
        except AttributeError:
            # Para teclas especiais, como espaço e 'q'
            if key == keyboard.Key.space:
                self.pressed_keys.add('space')
            elif key == keyboard.Key.esc:
                # Opcional: usar ESC para sair
                self.pressed_keys.add('esc')

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key.char.lower())
        except AttributeError:
            if key == keyboard.Key.space:
                self.pressed_keys.discard('space')
            elif key == keyboard.Key.esc:
                self.pressed_keys.discard('esc')

    def update(self):
        twist = Twist()

        if 'w' in self.pressed_keys:
            twist.linear.x = self.linear_speed
        elif 's' in self.pressed_keys:
            twist.linear.x = -self.linear_speed

        if 'a' in self.pressed_keys:
            twist.angular.z = self.angular_speed
        elif 'd' in self.pressed_keys:
            twist.angular.z = -self.angular_speed

        if 'space' in self.pressed_keys:

            twist.linear.x = 0.0
            twist.angular.z = 0.0


        if (twist.linear.x != 0.0 or twist.angular.z != 0.0 or
            time.time() - self.last_cmd_time > 0.5):
            self.velPub.publish(twist)
            self.last_cmd_time = time.time()

        if 'q' in self.pressed_keys or 'esc' in self.pressed_keys:

            self.get_logger().info("Encerrando teleop...")
            self.listener.stop()
            rclpy.shutdown()

def main(args=None):
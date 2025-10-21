import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import xml.etree.ElementTree as ET
import os
import time

class RobotPoseEstimation(Node):
    def __init__(self):
        super().__init__('robot_pose_estimation')
        pkg_dir = os.path.dirname(os.path.realpath(__file__))
        self.thresholds_xml_path = os.path.join(pkg_dir, "test_data", "limiares.xml")
        # -- Parâmetros
        self.declare_parameter('camera_index', 1)
        self.declare_parameter('thresholds_xml', self.thresholds_xml_path)
        self.declare_parameter('escala', 1.30*100.0)  # pixel/m

        self.pose_pub = self.create_publisher(PoseStamped, '/pose_topic', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.escala = self.get_parameter('escala').get_parameter_value().double_value
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.thresholds_xml = self.get_parameter('thresholds_xml').get_parameter_value().string_value
        self.thresholds = self.load_thresholds(self.thresholds_xml)

        # Inicializa vídeo
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Nao foi possivel abrir camera {self.camera_index}")
            exit(1)

        
        # Define resolução
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Define foco manual (nem todas as câmeras suportam)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # 0 = manual, 1 = automático
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)       # valor em uma faixa dependente da câmera

        # Define exposição manual
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual em algumas câmeras; 3 = automático (varia conforme driver)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -8)

        # Define contraste, saturação e brilho
        self.cap.set(cv2.CAP_PROP_CONTRAST, 10)
        self.cap.set(cv2.CAP_PROP_SATURATION, 200)
        brilho = 60
        print("Definindo brilho para:", brilho)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brilho)
        time.sleep(0.1)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 100)
        time.sleep(0.1)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brilho)
        #self.frame_teste = cv2.imread(self.image_path)
        print("Brilho aplicado:", self.cap.get(cv2.CAP_PROP_BRIGHTNESS))


        # Timer para processamento
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def load_thresholds(self, xml_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        thresholds = {}
        for color in root:
            thresholds[color.tag] = {}
            for child in color:
                thresholds[color.tag][child.tag] = int(child.text)
        return thresholds

    def calc_centroid(self, mask):
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy)
        else:
            return None

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Fim do video")
            return
        #frame = self.frame_teste.copy()  # Usar imagem fixa para teste

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Processa Azul e Vermelho (exemplo)
        #self.get_logger().info("Obtendo centroides...")
        centroids = {}
        for color in ["Azul", "Vermelho"]:
            t = self.thresholds[color]
            lower = (t["Rmin"], t["Gmin"], t["Bmin"])
            upper = (t["Rmax"], t["Gmax"], t["Bmax"])
            mask = cv2.inRange(frame_rgb, lower, upper)
            #Aplicar operações morfológicas
            kernel = np.ones((5, 5), np.uint8)  # kernel para operações morfológicas
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove ruído
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # preenche buracos
            centroids[color] = self.calc_centroid(mask)

        print(centroids)
        if centroids["Azul"] is None or centroids["Vermelho"] is None:
            return

        # Obter pose do robô
        #self.get_logger().info("Obtendo Pose...")
        h, w, _ = frame.shape
        centro_azul = centroids["Azul"]
        centro_vermelho = centroids["Vermelho"]
        x = np.array([centro_azul[0]/self.escala, centro_vermelho[0]/self.escala]) if centro_azul and centro_vermelho else 0
        y = np.array([(h-centro_azul[1])/self.escala, (h-centro_vermelho[1])/self.escala]) if centro_azul and centro_vermelho else 0
        # Pose do robô: posição = centro azul, orientação = direção azul->vermelho
        xr = np.mean(x) if len(x)>0 else 0
        yr = np.mean(y) if len(y)>0 else 0


        yaw = np.arctan2(y[1]-y[0], x[1]-x[0]) if len(x)>1 else 0 # radianos
        self.get_logger().info(f"Pose: x={xr:.2f} m, y={yr:.2f} m, yaw={yaw:.2f} rad")
        
        # Publica no /pose_topic
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"  
        pose_msg.pose.position.x = float(xr)
        pose_msg.pose.position.y = float(yr)
        pose_msg.pose.position.z = yaw
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(yaw/2)
        pose_msg.pose.orientation.w = np.cos(yaw/2)
        self.pose_pub.publish(pose_msg)

        # Publica TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link_cam'
        t.transform.translation.x = float(xr)
        t.transform.translation.y = float(yr)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(yaw/2)
        t.transform.rotation.w = np.cos(yaw/2)
        self.tf_broadcaster.sendTransform(t)

        # Desenhar na imagem
        # Converter para coordenadas de imagem
        draw_x = int(xr * self.escala)
        draw_y = int(h - yr * self.escala)  # inverte Y para desenhar no frame
        cv2.circle(frame, (draw_x, draw_y), 5, (0,255,255), -1)

        dir_x = xr*self.escala + (0.2)*self.escala*np.cos(yaw)
        dir_y = h - (yr*self.escala + (0.2)*self.escala*np.sin(yaw))

        cv2.arrowedLine(
            frame,
            (int(xr*self.escala), int(h-yr*self.escala)),
            (int(dir_x), int(dir_y)),
            (0,255,0),
            2
        )
        # --- Reduzir tamanho da imagem para exibição ---
        scale = 0.6  # 60% do tamanho original, ajuste conforme desejar
        frame_small = cv2.resize(frame, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        cv2.imshow("Robot Pose Estimation", frame_small)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseEstimation()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

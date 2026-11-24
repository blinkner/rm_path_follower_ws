"""Nó extrator de mapa 2D (Occupancy Grid) a partir de vista superior simulada.

Captura um frame da câmera zenital no Gazebo Sim, realiza limiarização binária
para separar o chão branco dos obstáculos, apaga a silhueta do robô utilizando
sua pose atual via odometria, aplica inflação morfológica radial (Configuration
Space) e exporta o mapa nos formatos padrão do ecossistema ROS (.pgm e .yaml).

Parâmetros ROS:
    map_dir (str): Diretório onde os arquivos do mapa serão salvos
        (padrão: '~/rm_maps').

Tópicos Assinados:
    /odom (nav_msgs/msg/Odometry): Pose atual do robô para projeção e
        mascaramento na imagem.
    /map_camera/image_raw (sensor_msgs/msg/Image): Imagem RGB capturada pela
        câmera superior centralizada no cenário.

Arquivos Gerados:
    ~/rm_path_follower_ws/rm_maps/meu_mapa.pgm: Bitmap binário (255 = espaço livre, 0 = obstáculo).
    ~/rm_path_follower_ws/rm_maps/meu_mapa.yaml: Metadados espaciais (resolução em m/px e origem).
"""

import math
import os

import cv2
import yaml
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class MapExtractor(Node):
    """Nó ROS 2 para geração única de mapas estáticos de ocupação."""

    def __init__(self):
        """Inicializa parâmetros geométricos, assinantes e diretório de saída."""
        super().__init__('map_extractor')
        self.bridge = CvBridge()       # CvBridge

        # Declaração e leitura do diretório de destino do mapa
        self.declare_parameter('map_dir', os.path.expanduser('~/rm_path_follower_ws/rm_maps'))
        self.map_dir = self.get_parameter('map_dir').value    # str
        os.makedirs(self.map_dir, exist_ok=True)

        # Parâmetros geométricos intrínsecos e extrínsecos da câmera no SDF
        self.cam_x = 2.0               # float (m)
        self.cam_y = 1.0               # float (m)
        self.cam_z = 100.0             # float (m)
        self.fov = 0.1                 # float (rad)
        self.img_width = 1200          # int (px)
        self.img_height = 1200         # int (px)

        # Parâmetros de pré-processamento morfológico
        self.robot_mask_size_m = 0.70  # float (lado do quadrado branco em m)
        self.inflation_radius_m = 0.22 # float (margem radial de segurança em m)

        # Estado dinâmico da posição do robô no mundo
        self.robo_x = None             # float (m)
        self.robo_y = None             # float (m)

        # Assinantes de odometria e imagem da câmera superior
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/map_camera/image_raw', self.image_callback, 10
        )
        self.get_logger().info(
            "Aguardando /odom e /map_camera/image_raw para gerar o mapa..."
        )

    def odom_callback(self, msg: Odometry):
        """Armazena a posição planar (X, Y) atual do robô.

        Args:
            msg (Odometry): Mensagem de odometria recebida de /odom.
        """
        self.robo_x = msg.pose.pose.position.x
        self.robo_y = msg.pose.pose.position.y

    def image_callback(self, msg: Image):
        """Processa a primeira imagem sincronizada, exporta o mapa e encerra o nó.

        Executa a conversão ROS -> OpenCV, limiarização binária, projeção da
        posição do robô para coordenadas de pixel, mascaramento do robô, erosão
        morfológica do espaço livre (inflação de obstáculos) e escrita dos
        arquivos .pgm e .yaml.

        Args:
            msg (Image): Frame RGB recebido de /map_camera/image_raw.

        Raises:
            SystemExit: Sinaliza o término limpo do nó após salvar os arquivos.
        """
        # Garante sincronização entre a primeira leitura de odometria e o frame
        if self.robo_x is None or self.robo_y is None:
            return

        # 1. Conversão para OpenCV e segmentação binária (chão = 255, obstáculos = 0)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_map = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)

        # 2. Cálculo da escala espacial (metros por pixel) via aproximação paraxial
        largura_visivel_m = 2.0 * self.cam_z * math.tan(self.fov / 2.0)
        resolucao_m_px = largura_visivel_m / self.img_width

        # 3. Projeção da posição do robô (m) para coordenadas de imagem (u, v)
        u_robo = int(round((self.img_width / 2.0) + (self.robo_x - self.cam_x) / resolucao_m_px))
        v_robo = int(round((self.img_height / 2.0) - (self.robo_y - self.cam_y) / resolucao_m_px))
        meio_lado_px = int(round((self.robot_mask_size_m / 2.0) / resolucao_m_px))

        # Mascara a silhueta do robô como espaço livre (branco) antes da inflação
        cv2.rectangle(
            binary_map,
            (u_robo - meio_lado_px, v_robo - meio_lado_px),
            (u_robo + meio_lado_px, v_robo + meio_lado_px),
            255,
            thickness=-1,
        )

        # 4. Inflação radial dos obstáculos (erosão do espaço livre branco)
        raio_inflacao_px = int(round(self.inflation_radius_m / resolucao_m_px))
        if raio_inflacao_px > 0:
            k = 2 * raio_inflacao_px + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            binary_map = cv2.erode(binary_map, kernel, iterations=1)

        # 5. Exportação do bitmap (.pgm) e metadados (.yaml)
        pgm_path = os.path.join(self.map_dir, 'meu_mapa.pgm')
        yaml_path = os.path.join(self.map_dir, 'meu_mapa.yaml')
        cv2.imwrite(pgm_path, binary_map)

        # Origem do mapa no padrão ROS corresponde ao canto inferior esquerdo (m)
        origem_x = self.cam_x - (largura_visivel_m / 2.0)
        origem_y = self.cam_y - (largura_visivel_m / 2.0)

        yaml_data = {
            'image': 'meu_mapa.pgm',
            'resolution': float(round(resolucao_m_px, 6)),
            'origin': [float(round(origem_x, 4)), float(round(origem_y, 4)), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }

        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        self.get_logger().info(f"Mapa salvo com sucesso em: {yaml_path}")
        raise SystemExit


def main(args=None):
    """Ponto de entrada do nó extrator de mapas; encerra após capturar um frame."""
    rclpy.init(args=args)
    node = MapExtractor()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
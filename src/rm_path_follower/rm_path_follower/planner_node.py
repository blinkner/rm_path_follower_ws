"""Nó planejador global de trajetórias sobre mapas de ocupação 2D.

Carrega o mapa estático (.pgm e .yaml), converte as coordenadas iniciais do robô
(via odometria) e o objetivo desejado de metros para índices de grade, executa o
algoritmo de roteamento selecionado (ex: A*, Wavefront), exporta a rota discreta
bruta para análise e publica a trajetória parametrizada no tempo para o nó
controlador.

Parâmetros ROS:
    map_dir (str): Diretório contendo os arquivos do mapa e onde será salvo o
        caminho bruto (padrão: '~/rm_path_follower_ws/rm_maps').
    algorithm (str): Nome do algoritmo de busca registrado em PLANNERS
        (padrão: 'astar'; opções atuais: 'astar', 'wavefront').
    goal_x (float): Coordenada X do objetivo no referencial global em metros
        (padrão: 5.5).
    goal_y (float): Coordenada Y do objetivo no referencial global em metros
        (padrão: 4.5).
    cruise_speed (float): Velocidade linear de referência utilizada na
        parametrização temporal da trajetória em m/s (padrão: 0.2).

Tópicos Assinados:
    /odom (nav_msgs/msg/Odometry): Pose atual do robô para definição do ponto
        de partida do planejamento.

Tópicos Publicados:
    /planned_path (nav_msgs/msg/Path): Sequência de waypoints convertidos em
        metros com carimbos de tempo relativos (QoS Transient Local).

Arquivos Gerados:
    ~/rm_path_follower_ws/rm_maps/caminho_bruto.csv: Coordenadas (x, y) em metros da rota discreta
        retornada pelo algoritmo antes da subamostragem e ajuste por spline.
"""

import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from rm_path_follower.planners import astar, wavefront
from rm_path_follower.utils.map_tools import (
    carregar_mapa_para_planejamento,
    mundo_para_pixel,
    parametrizar_trajetoria,
    pixel_para_mundo,
)


# Registro central de algoritmos de roteamento disponíveis.
# Novos planejadores devem seguir a assinatura: planejar(mapa, inicio, objetivo) -> list
PLANNERS = {
    'astar': astar.planejar,
    'wavefront': wavefront.planejar,
}


class PlannerNode(Node):
    """Nó ROS 2 responsável pelo planejamento global e parametrização temporal."""

    def __init__(self):
        """Declara parâmetros, configura QoS, publicador, assinante e timer."""
        super().__init__('planner_node')

        # Declaração dos parâmetros configuráveis via launch ou linha de comando
        self.declare_parameter('map_dir', os.path.expanduser('~/rm_path_follower_ws/rm_maps'))
        self.declare_parameter('algorithm', 'astar')
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('cruise_speed', 0.2)

        # Leitura e armazenamento dos parâmetros
        self.map_dir = self.get_parameter('map_dir').value                  # str
        self.algorithm_name = self.get_parameter('algorithm').value         # str
        self.goal_x = float(self.get_parameter('goal_x').value)             # float (m)
        self.goal_y = float(self.get_parameter('goal_y').value)             # float (m)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value) # float (m/s)

        # QoS Transient Local: retém a trajetória na memória para entregá-la
        # mesmo se o controlador ou RViz2 conectarem após a publicação
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', path_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Variáveis de controle de estado do planejamento
        self.planned = False          # bool
        self.robo_pos = None          # tuple (x_m, y_m)

        # Timer periódico (2 Hz) aguarda odometria e disponibilidade do arquivo de mapa
        self.timer = self.create_timer(0.5, self.try_plan)

    def odom_callback(self, msg: Odometry):
        """Atualiza a posição planar (X, Y) atual do robô via odometria.

        Args:
            msg (Odometry): Mensagem recebida do tópico /odom.
        """
        self.robo_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def try_plan(self):
        """Executa o fluxo completo de planejamento assim que mapa e pose estiverem prontos.

        Carrega e reduz a resolução do mapa de ocupação, converte início e meta
        para pixels, invoca o algoritmo escolhido em PLANNERS, grava o arquivo
        caminho_bruto.csv e publica a trajetória temporal em /planned_path.
        """
        if self.planned or self.robo_pos is None:
            return

        yaml_path = os.path.join(self.map_dir, 'meu_mapa.yaml')           # str
        if not os.path.exists(yaml_path):
            self.get_logger().info("Aguardando arquivo meu_mapa.yaml ser gerado...")
            return

        if self.algorithm_name not in PLANNERS:
            self.get_logger().error(
                f"Algoritmo '{self.algorithm_name}' não registrado em PLANNERS."
            )
            self.planned = True
            return

        self.get_logger().info(
            f"Planejando rota com [{self.algorithm_name}] de {self.robo_pos} "
            f"até ({self.goal_x}, {self.goal_y})..."
        )

        try:
            # 1. Carregamento do mapa com redução conservadora de escala (5x)
            grid, info = carregar_mapa_para_planejamento(yaml_path, fator_reducao=5)

            # 2. Conversão de coordenadas do mundo (m) para índices da matriz (row, col)
            inicio_px = mundo_para_pixel(self.robo_pos[0], self.robo_pos[1], info)
            objetivo_px = mundo_para_pixel(self.goal_x, self.goal_y, info)

            # 3. Execução do algoritmo de busca selecionado
            caminho_px = PLANNERS[self.algorithm_name](grid, inicio_px, objetivo_px)
            if caminho_px is None:
                self.get_logger().error("Nenhum caminho livre encontrado até o objetivo!")
                self.planned = True
                return

            # 4. Exportação da trajetória discreta bruta (em metros) para comparação
            bruto_path = os.path.join(self.map_dir, 'caminho_bruto.csv')  # str
            with open(bruto_path, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y'])
                for r, c in caminho_px:
                    xm, ym = pixel_para_mundo(r, c, info)
                    writer.writerow([round(xm, 4), round(ym, 4)])

            # 5. Subamostragem, parametrização temporal e publicação no tópico
            trajetoria = parametrizar_trajetoria(
                caminho_px, info, vel_cruzeiro=self.cruise_speed
            )
            self.publicar_caminho(trajetoria)
            self.planned = True
            self.get_logger().info(
                f"Trajetória publicada com {len(trajetoria)} pontos "
                f"(duração: {trajetoria[-1][0]:.1f}s)."
            )
        except Exception as e:
            self.get_logger().error(f"Falha no planejamento: {e}")
            self.planned = True

    def publicar_caminho(self, trajetoria):
        """Converte a lista de waypoints temporais em nav_msgs/msg/Path e publica.

        Cada PoseStamped armazena as coordenadas (x_m, y_m) em pose.position e o
        instante de tempo relativo t_sec (em segundos e nanossegundos) no campo
        header.stamp para reconstrução das splines no controlador.

        Args:
            trajetoria (list[tuple[float, float, float]]): Lista de tuplas no
                formato (tempo_s, x_m, y_m).
        """
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for t_sec, x_m, y_m in trajetoria:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'

            # Codifica o instante relativo t_i nos campos sec e nanosec do stamp
            sec = int(t_sec)                                              # int
            nanosec = int((t_sec - sec) * 1e9)                            # int
            pose.header.stamp.sec = sec
            pose.header.stamp.nanosec = nanosec

            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    """Ponto de entrada do nó planejador ROS 2."""
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
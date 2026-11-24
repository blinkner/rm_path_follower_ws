"""Nó controlador de seguimento de trajetória para robô móvel diferencial.

Implementa uma lei de controle por Linearização por Realimentação Entrada-Saída
(Input-Output Feedback Linearization) aplicada a um ponto virtual deslocado à
frente do eixo das rodas, garantindo rastreamento suave de trajetórias contínuas
geradas por interpolação via Splines Cúbicas.

Tópicos Assinados:
    /odom (nav_msgs/msg/Odometry): Pose atual (posição e orientação) do robô.
    /planned_path (nav_msgs/msg/Path): Waypoints parametrizados no tempo
        (QoS Transient Local).

Tópicos Publicados:
    /cmd_vel (geometry_msgs/msg/Twist): Comandos de velocidade linear (m/s) e
        angular (rad/s).

Arquivos Gerados:
    ~/rm_path_follower_ws/rm_maps/dados.csv: Log telemétrico da execução (tempo, estados reais,
        estados virtuais, referências e esforços de controle).
"""

import csv
import os

import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class ControllerTrajetori(Node):
    """Controlador cinemático por linearização por realimentação no ROS 2."""

    def __init__(self):
        """Inicializa publicadores, assinantes, parâmetros de controle e arquivo de log."""
        super().__init__('trajectory_tracker')

        # Publicador de comandos de velocidade
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Assinante de odometria
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # QoS Transient Local garante o recebimento da trajetória mesmo se o
        # planejador publicá-la antes da inicialização completa deste nó
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, path_qos
        )

        # Variáveis de controle temporal e interpolação
        self.start_time = None        # float
        self.odom_received = False    # bool
        self.spline_x = None          # CubicSpline
        self.spline_y = None          # CubicSpline
        self.total_time = 0.0         # float

        # Configuração do timer de controle (20 Hz)
        self.dt = 0.05                # float (período de amostragem em s)
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Estados atuais do robô no referencial global
        self.x = 0.0                  # float (m)
        self.y = 0.0                  # float (m)
        self.yaw = 0.0                # float (rad)

        # Parâmetros geométricos e ganhos da lei de controle
        self.d = 0.12                 # float (distância do ponto virtual em m)
        self.k1 = 0.8                 # float (ganho proporcional em X)
        self.k2 = 1.5                 # float (ganho proporcional em Y)

        # Inicialização do arquivo de registro de dados
        map_dir = os.path.expanduser('~/rm_path_follower_ws/rm_maps')              # str
        os.makedirs(map_dir, exist_ok=True)
        self.csv_path = os.path.join(map_dir, 'dados.csv')     # str

        with open(self.csv_path, mode='w', newline='', encoding='utf-8') as arquivo:
            escritor = csv.writer(arquivo)
            escritor.writerow(['tempo', 'x', 'y', 'yaw', 'xz', 'yz', 'xz_d', 'yz_d', 'v_cmd', 'w_cmd'])


    def quat(self, q):
        """Converte uma orientação em quatérnio para o ângulo de guinada (yaw).

        Args:
            q (geometry_msgs.msg.Quaternion): Componente de orientação da pose.

        Returns:
            float: Ângulo yaw (rotação em torno do eixo Z) em radianos [-pi, pi].
        """
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return float(np.arctan2(siny_cosp, cosy_cosp))


    def odom_callback(self, msg: Odometry):
        """Atualiza a pose atual (x, y, yaw) do robô a partir da odometria.

        Args:
            msg (Odometry): Mensagem recebida do tópico /odom.
        """
        self.odom_received = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quat(msg.pose.pose.orientation)


    def path_callback(self, msg: Path):
        """Constrói as curvas polinomiais contínuas a partir da rota planejada.

        Extrai os carimbos de tempo e coordenadas (x, y) de cada PoseStamped
        e ajusta splines cúbicas com derivadas nulas nas extremidades
        (bc_type='clamped'), garantindo partida e parada suaves.

        Args:
            msg (Path): Trajetória discreta publicada no tópico /planned_path.
        """
        if len(msg.poses) < 2:
            self.get_logger().warn("Trajetória recebida com menos de 2 pontos.")
            return

        t_pts = [p.header.stamp.sec + p.header.stamp.nanosec * 1e-9 for p in msg.poses]
        x_pts = [p.pose.position.x for p in msg.poses]
        y_pts = [p.pose.position.y for p in msg.poses]

        self.spline_x = CubicSpline(t_pts, x_pts, bc_type='clamped')
        self.spline_y = CubicSpline(t_pts, y_pts, bc_type='clamped')
        self.total_time = t_pts[-1]
        self.start_time = None
        self.get_logger().info(
            f"Nova trajetória carregada no controlador (Duração: {self.total_time:.2f}s)."
        )

    def salvar_dados(self, t, xz, yz, xz_d, yz_d, v_cmd, w_cmd):
        """Anexa uma amostra telemétrica da malha de controle ao arquivo CSV.

        Args:
            t (float): Tempo decorrido desde o início da trajetória (s).
            xz (float): Posição X real do ponto virtual controlado (m).
            yz (float): Posição Y real do ponto virtual controlado (m).
            xz_d (float): Posição X desejada na spline no instante t (m).
            yz_d (float): Posição Y desejada na spline no instante t (m).
            v_cmd (float): Velocidade linear comandada após saturação (m/s).
            w_cmd (float): Velocidade angular comandada após saturação (rad/s).
        """
        with open(self.csv_path, mode='a', newline='', encoding='utf-8') as arquivo:
            escritor = csv.writer(arquivo)
            escritor.writerow([
                round(t, 3),
                round(self.x, 4),
                round(self.y, 4),
                round(self.yaw, 4),
                round(xz, 4),
                round(yz, 4),
                round(xz_d, 4),
                round(yz_d, 4),
                round(v_cmd, 4),
                round(w_cmd, 4),
            ])

    def control_loop(self):
        """Executa uma iteração da lei de controle em malha fechada (20 Hz).

        Calcula a posição do ponto virtual (xz, yz), avalia a referência e sua
        primeira derivada temporal na spline, aplica a matriz de desacoplamento
        cinemático inversa G_inv e publica as velocidades saturadas em /cmd_vel.
        """
        if not self.odom_received or self.spline_x is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = current_time
            return

        t = current_time - self.start_time

        # Cinemática direta do ponto virtual deslocado à frente do eixo
        xz = self.x + self.d * np.cos(self.yaw)
        yz = self.y + self.d * np.sin(self.yaw)

        # Avaliação da posição e velocidade desejadas na trajetória
        if t <= self.total_time:
            xz_d = float(self.spline_x(t))
            yz_d = float(self.spline_y(t))
            dxz_d = float(self.spline_x(t, 1))
            dyz_d = float(self.spline_y(t, 1))
        else:
            # Ao concluir o tempo previsto, mantém o alvo final com feedforward nulo
            xz_d = float(self.spline_x(self.total_time))
            yz_d = float(self.spline_y(self.total_time))
            dxz_d = 0.0
            dyz_d = 0.0

        # Erros de rastreamento no referencial global
        error_x = xz_d - xz
        error_y = yz_d - yz

        # Inversa da matriz de desacoplamento cinemático G(yaw)
        G_inv = np.array([
            [np.cos(self.yaw), np.sin(self.yaw)],
            [-np.sin(self.yaw) / self.d, np.cos(self.yaw) / self.d],
        ])

        # Controle auxiliar (Feedforward de velocidade + Realimentação proporcional)
        p1 = dxz_d + self.k1 * error_x
        p2 = dyz_d + self.k2 * error_y
        u = G_inv @ np.array([p1, p2])

        # Saturação dos atuadores por segurança
        v_cmd = float(np.clip(u[0], -1.0, 1.0))
        w_cmd = float(np.clip(u[1], -1.5, 1.5))

        # Publicação do comando e registro telemétrico
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)
        self.salvar_dados(t, xz, yz, xz_d, yz_d, v_cmd, w_cmd)


def main(args=None):
    """Ponto de entrada do nó ROS 2; garante parada segura do robô ao encerrar."""
    rclpy.init(args=args)
    node = ControllerTrajetori()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Envia velocidades nulas antes de destruir o nó
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
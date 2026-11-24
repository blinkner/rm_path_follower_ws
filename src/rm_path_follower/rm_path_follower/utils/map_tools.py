"""Utilitários espaciais e temporais para mapas de ocupação e trajetórias.

Fornece funções puras (independentes de ROS) para carregar mapas no padrão
.pgm/.yaml com redução conservadora de resolução, converter coordenadas entre
o referencial contínuo do mundo (metros) e índices discretos de grade (pixels),
e parametrizar rotas discretas no tempo para interpolação suave no controlador.
"""

import os

import cv2
import numpy as np
import yaml


def carregar_mapa_para_planejamento(yaml_path, fator_reducao=5):
    """Carrega os metadados (.yaml) e a imagem (.pgm) para gerar a grade de busca.

    Realiza uma redução de escala em blocos utilizando o operador de mínimo
    (.min), garantindo de forma conservadora que qualquer célula contendo ao
    menos um pixel de obstáculo seja marcada como ocupada na grade reduzida.

    Args:
        yaml_path (str): Caminho absoluto para o arquivo de metadados .yaml.
        fator_reducao (int): Fator inteiro de subamostragem espacial da imagem
            (ex: fator 5 reduz uma imagem 1200x1200 para 240x240). Padrão: 5.

    Returns:
        tuple[np.ndarray, dict]: Tupla contendo:
            - grid (np.ndarray): Matriz 2D uint8 (0 = espaço livre, 1 = obstáculo).
            - info_mapa (dict): Dicionário com 'resolution', 'origin_x',
              'origin_y', 'rows' e 'cols' ajustados para a grade reduzida.

    Raises:
        FileNotFoundError: Se a imagem .pgm referenciada no .yaml não existir.
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)                                  # dict

    pgm_path = meta['image']                                      # str
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)              # np.ndarray
    if img is None:
        raise FileNotFoundError(f"Não foi possível abrir a imagem do mapa: {pgm_path}")

    resolucao = float(meta['resolution'])                         # float (m/px)
    origem_x = float(meta['origin'][0])                           # float (m)
    origem_y = float(meta['origin'][1])                           # float (m)

    # Redução conservadora em blocos (ex: 1200x1200 -> 240x240)
    if fator_reducao > 1:
        h, w = img.shape                                          # int, int
        h_novo = h // fator_reducao                               # int
        w_novo = w // fator_reducao                               # int
        img_cortada = img[:h_novo * fator_reducao, :w_novo * fator_reducao]

        # .min() preserva obstáculos mesmo se ocuparem apenas 1 pixel no bloco
        img = img_cortada.reshape(
            h_novo, fator_reducao, w_novo, fator_reducao
        ).min(axis=(1, 3))
        resolucao *= fator_reducao

    # Binarização para planejamento: 0 = livre (branco >= 240), 1 = obstáculo
    grid = np.where(img < 240, 1, 0).astype(np.uint8)             # np.ndarray

    info_mapa = {                                                 # dict
        'resolution': resolucao,
        'origin_x': origem_x,
        'origin_y': origem_y,
        'rows': grid.shape[0],
        'cols': grid.shape[1],
    }
    return grid, info_mapa


def mundo_para_pixel(x_m, y_m, info):
    """Converte coordenadas cartesianas do mundo (metros) para índices da matriz.

    Considera que a origem da imagem/matriz (0, 0) fica no canto superior
    esquerdo, invertendo o sentido vertical em relação ao eixo +Y do Gazebo.

    Args:
        x_m (float): Coordenada X no referencial global (m).
        y_m (float): Coordenada Y no referencial global (m).
        info (dict): Dicionário de metadados retornado por
            carregar_mapa_para_planejamento.

    Returns:
        tuple[int, int]: Índices discretos (linha, coluna) na matriz do mapa.
    """
    col = int(round((x_m - info['origin_x']) / info['resolution']))              # int
    row = int(round(info['rows'] - (y_m - info['origin_y']) / info['resolution'])) # int
    return (row, col)


def pixel_para_mundo(row, col, info):
    """Converte índices discretos da matriz para coordenadas cartesianas (metros).

    Args:
        row (int): Índice da linha na matriz do mapa.
        col (int): Índice da coluna na matriz do mapa.
        info (dict): Dicionário de metadados retornado por
            carregar_mapa_para_planejamento.

    Returns:
        tuple[float, float]: Coordenadas contínuas (x_m, y_m) no referencial global.
    """
    x_m = info['origin_x'] + (col * info['resolution'])                          # float (m)
    y_m = info['origin_y'] + ((info['rows'] - row) * info['resolution'])         # float (m)
    return (x_m, y_m)


def parametrizar_trajetoria(caminho_px, info, vel_cruzeiro=0.35, passo_amostragem=6):
    """Subamostra a rota em pixels, converte para metros e atribui carimbos de tempo.

    A subamostragem evita que a interpolação por spline cúbica no controlador
    oscile sobre os degraus discretos da grade. O carimbo temporal de cada ponto
    é acumulado proporcionalmente à distância Euclidiana entre vértices consecutivos.

    Args:
        caminho_px (list[tuple[int, int]]): Lista ordenada de células (row, col)
            retornada pelo algoritmo de planejamento.
        info (dict): Dicionário de metadados espaciais do mapa.
        vel_cruzeiro (float): Velocidade linear média desejada ao longo do
            percurso (m/s). Padrão: 0.35.
        passo_amostragem (int): Intervalo de índices utilizado para filtrar
            waypoints intermediários (mantendo sempre o início e o fim). Padrão: 6.

    Returns:
        list[tuple[float, float, float]]: Sequência de waypoints parametrizados
            no formato [(t_0, x_0, y_0), (t_1, x_1, y_1), ...], com tempo em
            segundos e coordenadas em metros.
    """
    if len(caminho_px) > 2 and passo_amostragem > 1:
        pontos_filtrados = caminho_px[::passo_amostragem]         # list
        # Garante que o ponto exato de objetivo seja sempre preservado
        if pontos_filtrados[-1] != caminho_px[-1]:
            pontos_filtrados.append(caminho_px[-1])
    else:
        pontos_filtrados = caminho_px                             # list

    pontos_metros = [pixel_para_mundo(r, c, info) for (r, c) in pontos_filtrados] # list

    trajetoria_temporal = [(0.0, pontos_metros[0][0], pontos_metros[0][1])]       # list
    tempo_acumulado = 0.0                                         # float (s)

    for i in range(1, len(pontos_metros)):
        x_ant, y_ant = pontos_metros[i - 1]                       # float, float
        x_atual, y_atual = pontos_metros[i]                       # float, float
        dist = float(np.hypot(x_atual - x_ant, y_atual - y_ant))  # float (m)

        if dist > 1e-6:
            tempo_acumulado += dist / vel_cruzeiro
            trajetoria_temporal.append((tempo_acumulado, x_atual, y_atual))

    return trajetoria_temporal
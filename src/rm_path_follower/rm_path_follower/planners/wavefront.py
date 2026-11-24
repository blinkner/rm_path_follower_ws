"""Algoritmo de planejamento de trajetória Wavefront (Propagação de Frente de Onda).

Implementa a busca em largura (BFS) a partir da célula objetivo para construir
uma matriz potencial de distâncias discretas sobre uma grade de ocupação 2D
com conectividade de 8 vizinhos e prevenção de corte de quinas (corner cutting)
entre obstáculos adjacentes. A trajetória é extraída por descida de gradiente
do ponto inicial até o mínimo global no objetivo.

Segue a interface padrão exigida pelo nó planejador:
    planejar(mapa, inicio, objetivo) -> list[tuple[int, int]] | None
"""

from collections import deque
import numpy as np


def wavefront(mapa, objetivo, movimentos):
    """Propaga a frente de onda a partir do objetivo para calcular o campo de distâncias.

    Args:
        mapa (np.ndarray): Matriz binária 2D (0 = espaço livre, != 0 = obstáculo).
        objetivo (tuple[int, int]): Índices (linha, coluna) do destino desejado.
        movimentos (list[tuple[int, int]]): Lista de deslocamentos (dx, dy)
            permitidos na vizinhança da grade.

    Returns:
        np.ndarray: Matriz 2D de floats contendo a distância discreta mínima
            de cada célula alcançável até o objetivo (np.inf para obstáculos
            ou regiões isoladas).

    Raises:
        ValueError: Se o objetivo estiver fora dos limites do mapa ou
            posicionado sobre um obstáculo.
    """
    mapa = np.array(mapa, dtype=int)                              # np.ndarray
    linhas, colunas = mapa.shape                                  # int, int

    # Matriz potencial inicializada com infinito em todas as células
    distancias = np.full((linhas, colunas), np.inf)               # np.ndarray

    x, y = objetivo                                               # int, int

    # Validação prévia das coordenadas do objetivo
    if not (0 <= x < linhas and 0 <= y < colunas):
        raise ValueError(f"Objetivo {objetivo} fora dos limites do mapa {mapa.shape}")

    if mapa[x, y] != 0:
        raise ValueError(f"Objetivo {objetivo} localizado em obstáculo")

    # Inicializa a fila FIFO com o objetivo (potencial zero)
    fila = deque([objetivo])                                      # deque
    distancias[x, y] = 0.0

    while fila:
        atual = fila.popleft()                                    # tuple
        x, y = atual                                              # int, int

        for dx, dy in movimentos:
            nx = x + dx                                           # int
            ny = y + dy                                           # int

            # Descarta índices fora dos limites da matriz
            if not (0 <= nx < linhas and 0 <= ny < colunas):
                continue

            # Descarta células ocupadas por obstáculos
            if mapa[nx, ny] != 0:
                continue

            # Descarta células já visitadas pela frente de onda
            if distancias[nx, ny] != np.inf:
                continue

            # Impede passagem diagonal entre dois obstáculos adjacentes
            if dx != 0 and dy != 0:
                if mapa[x + dx, y] != 0 or mapa[x, y + dy] != 0:
                    continue

            # Atualiza o potencial da célula vizinha e enfileira para expansão
            distancias[nx, ny] = distancias[x, y] + 1.0
            fila.append((nx, ny))

    return distancias


def planejar(mapa, inicio, objetivo):
    """Planeja uma trajetória livre de colisões usando o método Wavefront.

    Constrói o mapa potencial via wavefront() e realiza a descida de
    gradiente a partir de inicio, selecionando iterativamente o vizinho
    acessível de menor distância até alcançar objetivo.

    Args:
        mapa (np.ndarray): Matriz binária 2D (0 = espaço livre, 1 = obstáculo).
        inicio (tuple[int, int]): Índices (linha, coluna) do ponto de partida.
        objetivo (tuple[int, int]): Índices (linha, coluna) do destino desejado.

    Returns:
        list[tuple[int, int]] | None: Lista ordenada de células (linha, coluna)
            do início ao objetivo, ou None caso não exista caminho viável.

    Raises:
        ValueError: Se inicio ou objetivo estiverem fora dos limites do mapa ou
            posicionados sobre um obstáculo.
    """
    linhas, colunas = mapa.shape                                  # int, int
    ix, iy = inicio                                               # int, int

    # Validação prévia do ponto inicial (evita IndexError na descida de gradiente)
    if not (0 <= ix < linhas and 0 <= iy < colunas):
        raise ValueError(f"Início {inicio} fora dos limites do mapa {mapa.shape}")
    if mapa[ix, iy] != 0:
        raise ValueError(f"Início {inicio} localizado em obstáculo")

    movimentos = [                                                # list
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    distancias = wavefront(mapa, objetivo, movimentos)            # np.ndarray
    atual = inicio                                                # tuple
    caminho = [atual]                                             # list

    while atual != objetivo:
        x, y = atual                                              # int, int
        melhor_vizinho = None                                     # tuple | None
        menor_distancia = distancias[x, y]                        # float

        for dx, dy in movimentos:
            nx = x + dx                                           # int
            ny = y + dy                                           # int

            # Descarta índices fora dos limites da matriz
            if not (0 <= nx < linhas and 0 <= ny < colunas):
                continue

            # Descarta células ocupadas por obstáculos
            if mapa[nx, ny] != 0:
                continue

            # Impede movimento diagonal entre dois obstáculos adjacentes
            if dx != 0 and dy != 0:
                if mapa[x + dx, y] != 0 or mapa[x, y + dy] != 0:
                    continue

            # Seleciona o vizinho com menor potencial de distância
            if distancias[nx, ny] < menor_distancia:
                menor_distancia = distancias[nx, ny]
                melhor_vizinho = (nx, ny)

        # Caminho bloqueado ou mínimo local sem saída até o objetivo
        if melhor_vizinho is None:
            return None

        atual = melhor_vizinho
        caminho.append(atual)

    return caminho
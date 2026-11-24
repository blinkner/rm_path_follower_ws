"""Algoritmo de planejamento de trajetória A* (A-Star) em grade 2D.

Implementa a busca heurística informada sobre uma matriz de ocupação binária
com conectividade de 8 vizinhos (movimentos ortogonais e diagonais), utilizando
a Distância de Octile como heurística admissível e prevenção de corte de quinas
(corner cutting) entre obstáculos adjacentes.

Segue a interface padrão exigida pelo nó planejador:
    planejar(mapa, inicio, objetivo) -> list[tuple[int, int]] | None
"""

import heapq
import math


def heuristica(a, b):
    """Calcula a Distância de Octile entre duas células da grade.

    É a heurística exata (admissível e consistente) para movimentação em
    8 direções com custo unitário nos eixos ortogonais e custo sqrt(2) nas
    diagonais.

    Args:
        a (tuple[int, int]): Coordenada (linha, coluna) da célula atual.
        b (tuple[int, int]): Coordenada (linha, coluna) da célula objetivo.

    Returns:
        float: Estimativa de custo mínimo entre a e b.
    """
    dx = abs(a[0] - b[0])                                         # int
    dy = abs(a[1] - b[1])                                         # int
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def obter_vizinhos(posicao, mapa):
    """Retorna as células vizinhas acessíveis e seus respectivos custos de passo.

    Avalia os 8 movimentos possíveis ao redor de posicao, descartando células
    fora dos limites da matriz, células ocupadas e movimentos diagonais que
    tangenciem quinas de obstáculos adjacentes.

    Args:
        posicao (tuple[int, int]): Célula atual (linha, coluna).
        mapa (np.ndarray): Matriz 2D de ocupação (0 = livre, != 0 = obstáculo).

    Returns:
        list[tuple[tuple[int, int], float]]: Lista de pares contendo a
            coordenada (nx, ny) do vizinho válido e o custo de transição.
    """
    movimentos = [                                                # list
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
    ]
    vizinhos = []                                                 # list
    linhas, colunas = mapa.shape                                  # int, int
    x, y = posicao                                                # int, int

    for dx, dy, custo in movimentos:
        nx = x + dx                                               # int
        ny = y + dy                                               # int

        # Descarta índices fora dos limites da matriz
        if not (0 <= nx < linhas and 0 <= ny < colunas):
            continue

        # Descarta células ocupadas por obstáculos
        if mapa[nx, ny] != 0:
            continue

        # Impede atravessar diagonalmente entre dois obstáculos adjacentes
        if dx != 0 and dy != 0:
            if mapa[x + dx, y] != 0 or mapa[x, y + dy] != 0:
                continue

        vizinhos.append(((nx, ny), custo))

    return vizinhos


def reconstruir_caminho(came_from, atual):
    """Reconstrói a sequência ordenada de células do início até o objetivo.

    Args:
        came_from (dict[tuple[int, int], tuple[int, int]]): Mapa de
            predecessores gerado durante a expansão dos nós.
        atual (tuple[int, int]): Célula objetivo alcançada pela busca.

    Returns:
        list[tuple[int, int]]: Caminho ordenado [(row_0, col_0), ..., (row_n, col_n)].
    """
    caminho = [atual]                                             # list
    while atual in came_from:
        atual = came_from[atual]
        caminho.append(atual)
    return caminho[::-1]


def planejar(mapa, inicio, objetivo):
    """Executa a busca A* para encontrar o caminho de menor custo na grade.

    Args:
        mapa (np.ndarray): Matriz binária 2D (0 = espaço livre, 1 = obstáculo).
        inicio (tuple[int, int]): Índices (linha, coluna) do ponto de partida.
        objetivo (tuple[int, int]): Índices (linha, coluna) do destino desejado.

    Returns:
        list[tuple[int, int]] | None: Lista ordenada de células (linha, coluna)
            do início ao objetivo, ou None caso não exista caminho viável.

    Raises:
        ValueError: Se inicio ou objetivo estiverem fora dos limites do mapa ou
            posicionados sobre uma célula de obstáculo.
    """
    linhas, colunas = mapa.shape                                  # int, int

    # Validação prévia das coordenadas de início e destino
    for posicao in [inicio, objetivo]:
        x, y = posicao                                            # int, int
        if not (0 <= x < linhas and 0 <= y < colunas):
            raise ValueError(f"Posição {posicao} fora dos limites do mapa {mapa.shape}")
        if mapa[x, y] != 0:
            raise ValueError(f"Posição {posicao} está sobre um obstáculo")

    # Estruturas de controle da busca A*
    open_set = []                                                 # list (min-heap)
    heapq.heappush(open_set, (heuristica(inicio, objetivo), inicio))

    came_from = {}                                                # dict
    g_score = {inicio: 0.0}                                       # dict
    closed_set = set()                                            # set

    while open_set:
        _, atual = heapq.heappop(open_set)                        # tuple

        if atual in closed_set:
            continue

        if atual == objetivo:
            return reconstruir_caminho(came_from, atual)

        closed_set.add(atual)

        for vizinho, custo_movimento in obter_vizinhos(atual, mapa):
            if vizinho in closed_set:
                continue

            custo_tentativo = g_score[atual] + custo_movimento    # float
            if custo_tentativo < g_score.get(vizinho, float("inf")):
                came_from[vizinho] = atual
                g_score[vizinho] = custo_tentativo
                f_score = custo_tentativo + heuristica(vizinho, objetivo) # float
                heapq.heappush(open_set, (f_score, vizinho))

    # Retorna None quando toda a região alcançável foi explorada sem sucesso
    return None
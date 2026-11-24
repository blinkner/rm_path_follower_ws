import os
import csv
import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt


def carregar_csv_colunas(caminho_csv: str) -> dict:
    with open(caminho_csv, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        dados = {col: [] for col in reader.fieldnames}
        for linha in reader:
            for col in reader.fieldnames:
                dados[col].append(float(linha[col]))
    return {col: np.array(vals) for col, vals in dados.items()}


def main(args=None):
    map_dir = os.path.expanduser('~/rm_path_follower_ws/rm_maps')
    yaml_path = os.path.join(map_dir, 'meu_mapa.yaml')
    bruto_path = os.path.join(map_dir, 'caminho_bruto.csv')
    dados_path = os.path.join(map_dir, 'dados.csv')

    for arq in [yaml_path, bruto_path, dados_path]:
        if not os.path.exists(arq):
            raise FileNotFoundError(f"Arquivo necessário não encontrado: {arq}")

    # 1. Carregar mapa e metadados espaciais
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)

    pgm_path = os.path.join(map_dir, meta['image'])
    img_mapa = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    res = float(meta['resolution'])
    orig_x = float(meta['origin'][0])
    orig_y = float(meta['origin'][1])
    altura_m = img_mapa.shape[0] * res
    largura_m = img_mapa.shape[1] * res

    # 2. Carregar trajetórias e logs do controlador
    bruto = carregar_csv_colunas(bruto_path)
    ctrl = carregar_csv_colunas(dados_path)

    t = ctrl['tempo']
    erro_x = ctrl['xz_d'] - ctrl['xz']
    erro_y = ctrl['yz_d'] - ctrl['yz']
    erro_euclidiano = np.hypot(erro_x, erro_y)

    # Métricas numéricas no terminal
    rmse = float(np.sqrt(np.mean(erro_euclidiano ** 2)))
    erro_max = float(np.max(erro_euclidiano))
    print(f"--- Métricas de Desempenho ---")
    print(f"Duração registrada : {t[-1]:.2f} s")
    print(f"Erro RMSE (ponto z): {rmse:.4f} m")
    print(f"Erro Máximo        : {erro_max:.4f} m")

    # =========================================================================
    # FIGURA 1: Comparação Espacial das Três Trajetórias sobre o Mapa
    # =========================================================================
    fig1, ax1 = plt.subplots(figsize=(8, 8))
    ax1.imshow(
        img_mapa,
        cmap='gray',
        origin='upper',
        extent=[orig_x, orig_x + largura_m, orig_y, orig_y + altura_m],
        alpha=0.85
    )

    ax1.plot(
        bruto['x'], bruto['y'],
        color='royalblue', linestyle=':', linewidth=1.8, marker='.', markersize=4,
        label='1. Trajetória Bruta (A*)'
    )
    ax1.plot(
        ctrl['xz_d'], ctrl['yz_d'],
        color='limegreen', linestyle='--', linewidth=2.2,
        label='2. Trajetória Suavizada (Spline Cúbica)'
    )
    ax1.plot(
        ctrl['x'], ctrl['y'],
        color='crimson', linestyle='-', linewidth=2.0,
        label='3. Trajetória Percorrida (Centro do Robô)'
    )
    ax1.plot(
        ctrl['xz'], ctrl['yz'],
        color='darkorange', linestyle='-.', linewidth=1.2, alpha=0.8,
        label='Ponto Virtual Controlado (xz, yz)'
    )

    ax1.scatter(bruto['x'][0], bruto['y'][0], color='green', s=90, zorder=5, label='Início')
    ax1.scatter(bruto['x'][-1], bruto['y'][-1], color='blue', marker='X', s=100, zorder=5, label='Objetivo')

    # Foca a janela na área útil da sala 
    ax1.set_xlim(-3.2, 7.2)
    ax1.set_ylim(-4.2, 6.2)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Comparação: Roteamento vs. Spline vs. Trajetória Executada')
    ax1.grid(True, linestyle='--', alpha=0.4)
    ax1.legend(loc='upper left', fontsize=9)

    # =========================================================================
    # FIGURA 2: Sinais Temporais de Erro e Ações de Controle
    # =========================================================================
    fig2, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    # Subplot 1: Erros de posição
    axs[0].plot(t, erro_x, label='Erro X ($x_{z,d} - x_z$)', linewidth=1.5)
    axs[0].plot(t, erro_y, label='Erro Y ($y_{z,d} - y_z$)', linewidth=1.5)
    axs[0].plot(t, erro_euclidiano, 'k--', label='Erro Euclidiano $\\|e\\|$', linewidth=1.8)
    axs[0].set_ylabel('Erro [m]')
    axs[0].set_title('Evolução Temporal dos Erros e Sinais de Controle')
    axs[0].grid(True, alpha=0.4)
    axs[0].legend(loc='upper right', fontsize=9)

    # Subplot 2: Velocidade Linear
    axs[1].plot(t, ctrl['v_cmd'], color='tab:blue', linewidth=1.8, label='$v_{cmd}$')
    axs[1].axhline(1.0, color='gray', linestyle=':', linewidth=1.0, label='Saturação ($\pm 1.0$ m/s)')
    axs[1].axhline(-1.0, color='gray', linestyle=':', linewidth=1.0)
    axs[1].set_ylabel('Vel. Linear [m/s]')
    axs[1].grid(True, alpha=0.4)
    axs[1].legend(loc='upper right', fontsize=9)

    # Subplot 3: Velocidade Angular
    axs[2].plot(t, ctrl['w_cmd'], color='tab:red', linewidth=1.8, label='$\omega_{cmd}$')
    axs[2].axhline(1.5, color='gray', linestyle=':', linewidth=1.0, label='Saturação ($\pm 1.5$ rad/s)')
    axs[2].axhline(-1.5, color='gray', linestyle=':', linewidth=1.0)
    axs[2].set_ylabel('Vel. Angular [rad/s]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].grid(True, alpha=0.4)
    axs[2].legend(loc='upper right', fontsize=9)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
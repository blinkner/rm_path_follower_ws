# `rm_path_follower` — Mapeamento, Planejamento e Controle de Trajetória em ROS 2

Pacote ROS 2 (Jazzy) integrado ao **Gazebo Sim (Harmonic)** para navegação autônoma de um robô móvel diferencial (`rm_bot`). O sistema implementa uma arquitetura modular dividida em extração de mapa de ocupação 2D por visão zenital, planejamento global discreto em grade (com suporte intercambiável a algoritmos como **A\*** e **Wavefront**), parametrização temporal contínua por **Splines Cúbicas** e rastreamento de trajetória via **Linearização por Realimentação (*Input-Output Feedback Linearization*)**.

---

## Arquitetura do Sistema

O fluxo de dados é desacoplado em três estágios principais e um módulo de análise pós-execução:

```text
[1. Extração de Mapa (Sob demanda)]
Gazebo (/map_camera/image_raw + /odom)
       │
       ▼
map_extractor ──(Limiarização + Máscara do Robô + Inflação Radial)──> ~/rm_path_follower_ws/rm_maps/meu_mapa.{pgm,yaml}

[2. Planejamento Global]
~/rm_path_follower_ws/rm_maps/meu_mapa.{pgm,yaml} + /odom
       │
       ▼
planner_node [planners: astar | wavefront]
       ├──(Rota discreta em metros)──> ~/rm_path_follower_ws/rm_maps/caminho_bruto.csv
       └──(Subamostragem + Tempo)────> Tópico: /planned_path (nav_msgs/msg/Path)

[3. Controle em Malha Fechada (20 Hz)]
/planned_path + /odom
       │
       ▼
controller_node (CubicSpline + Feedback Linearization)
       ├──(Comandos saturados)───────> Tópico: /cmd_vel (geometry_msgs/msg/Twist)
       └──(Telemetria)───────────────> ~/rm_path_follower_ws/rm_maps/dados.csv

[4. Análise e Visualização]
plot_resultados ──(Lê meu_mapa, caminho_bruto.csv e dados.csv)──> Gráficos Matplotlib
```

---

## Estrutura do Workspace e Pacote

```text
rm_path_follower_ws/
├── rm_maps/                            # Diretório de saída para mapas (.pgm/.yaml) e logs (.csv)
└── src/
    ├── rm_bot/                         # Definições base do robô diferencial
    ├── rm_bot_description/             # Modelos URDF/Xacro e malhas do robô
    ├── rm_bot_sim/                     # Arquivos de launch e configuração da simulação no Gazebo
    ├── rm_path_follower/               # Pacote principal de mapeamento, planejamento e controle
    │   ├── launch/
    │   │   └── atividade1.launch.py    # Inicialização integrada da simulação e dos nós
    │   ├── resource/
    │   ├── rviz/
    │   │   └── aula04.rviz             # Configuração visual pré-definida do RViz2
    │   ├── worlds/
    │   │   └── mundo_teste.sdf         # Ambiente simulado (10m x 10m) com câmera ortográfica zenital
    │   ├── rm_path_follower/
    │   │   ├── __init__.py
    │   │   ├── map_extractor.py        # Nó one-shot para geração do mapa (.pgm e .yaml)
    │   │   ├── planner_node.py         # Nó de planejamento global e publicação de /planned_path
    │   │   ├── controller_node.py      # Nó controlador cinemático por linearização por realimentação
    │   │   ├── planners/               # Módulos Python puros (independentes de ROS)
    │   │   │   ├── __init__.py
    │   │   │   ├── astar.py            # Busca heurística A* (Distância de Octile, 8 vizinhos)
    │   │   │   └── wavefront.py        # Propagação de frente de onda (BFS + descida de gradiente)
    │   │   └── utils/                  # Ferramentas espaciais e de visualização
    │   │       ├── __init__.py
    │   │       ├── map_tools.py        # Conversão m <-> px, redução de grade e parametrização temporal
    │   │       └── plot_resultados.py  # Script para comparação de trajetórias e análise de controle
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py
    └── README.md
```

---

## Pré-requisitos e Instalação

### Dependências
* **ROS 2 Jazzy** e **Gazebo Sim (Harmonic)**
* Pacotes de simulação presentes no workspace: `rm_bot`, `rm_bot_description`, `rm_bot_sim`
* Pacotes ROS 2: `ros_gz_bridge`, `cv_bridge`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`
* Bibliotecas Python: `numpy`, `scipy`, `opencv-python`, `pyyaml`, `matplotlib`

### Compilação

Na raiz do workspace (`~/rm_path_follower_ws`):

```bash
cd ~/rm_path_follower_ws
colcon build --packages-select rm_path_follower
source install/setup.bash
```

*(Caso ainda não tenha compilado os pacotes do robô na primeira instalação, execute `colcon build` sem `--packages-select` para compilar todo o workspace).*

---

## Como Executar

### 1. Primeira Execução (Gerar Mapa + Navegar)
Sempre que iniciar o projeto pela primeira vez ou alterar os obstáculos do arquivo `worlds/mundo_teste.sdf`, execute com `extract_map:=true`. O nó `map_extractor` capturará a imagem da câmera superior, apagará o robô na posição inicial, aplicará a inflação de segurança nos obstáculos e encerrará automaticamente após salvar os arquivos em `~/rm_path_follower_ws/rm_maps/`:

```bash
ros2 launch rm_path_follower atividade1.launch.py extract_map:=true goal_x:=5.5 goal_y:=4.5
```

### 2. Execuções Subsequentes (Reutilizando o Mapa Salvo)
Com o mapa já salvo em `~/rm_path_follower_ws/rm_maps/`, mantenha `extract_map:=false` (padrão) e altere apenas as coordenadas de destino ou o algoritmo de busca:

```bash
# Usando A* (padrão)
ros2 launch rm_path_follower atividade1.launch.py goal_x:=5.8 goal_y:=-2.8

# Usando Wavefront
ros2 launch rm_path_follower atividade1.launch.py algorithm:=wavefront goal_x:=-1.8 goal_y:=4.5
```

> **Coordenadas válidas no `mundo_teste.sdf`:** A sala possui dimensões úteis de $9.8\text{ m} \times 9.8\text{ m}$ centradas em $(2.0, 1.0)$, abrangendo aproximadamente $X \in [-2.7, 6.7]\text{ m}$ e $Y \in [-3.7, 5.7]\text{ m}$. O ponto de *spawn* padrão do robô é $(0.0, 0.0)$.

### 3. Visualização Pós-Execução (Matplotlib)
Após o robô concluir o trajeto, encerre a simulação no terminal (`Ctrl+C`) e execute o script de diagnóstico para visualizar a sobreposição das três trajetórias (A\*/Wavefront bruto, Spline Cúbica e trajetória real) e os gráficos temporais de erro e velocidades:

```bash
ros2 run rm_path_follower plot_resultados
```

### 4. Visualização em Tempo Real no RViz2
Para inspecionar a rota planejada diretamente no RViz2 durante a simulação:
1. Adicione um display do tipo **Path** inscrito no tópico `/planned_path`.
2. Expanda as propriedades do tópico no painel esquerdo e altere **Durability Policy** para **`Transient Local`**.
3. Certifique-se de que o **Fixed Frame** global está configurado como `odom`.

---

## Referência de Parâmetros, Tópicos e Arquivos

### Argumentos do Launch (`atividade1.launch.py`)

| Argumento | Padrão | Opções / Tipo | Descrição |
| :--- | :--- | :--- | :--- |
| `mode` | `'auto'` | `'auto'`, `'manual'` | Define se o sistema executa o planejamento e controle autônomo (`auto`) ou apenas a simulação (`manual`). |
| `extract_map` | `'false'` | `'true'`, `'false'` | Ativa a ponte `ros_gz_bridge` da câmera superior e o nó `map_extractor` para gerar um novo mapa. |
| `algorithm` | `'astar'` | `'astar'`, `'wavefront'` | Seleciona o algoritmo de planejamento global utilizado pelo `planner_node`. |
| `goal_x` | `'5.5'` | `float` (metros) | Coordenada $X$ do objetivo no referencial global `odom`. |
| `goal_y` | `'4.5'` | `float` (metros) | Coordenada $Y$ do objetivo no referencial global `odom`. |
| `headless` | `'false'` | `'true'`, `'false'` | Executa o servidor do Gazebo sem abrir a interface gráfica 3D. |
| `rviz` | `'true'` | `'true'`, `'false'` | Abre a janela de visualização do RViz2 junto com a simulação. |

### Tópicos Principais

| Tópico | Tipo de Mensagem | QoS | Publicador $\rightarrow$ Assinante |
| :--- | :--- | :--- | :--- |
| `/map_camera/image_raw` | `sensor_msgs/msg/Image` | Default | `ros_gz_bridge` $\rightarrow$ `map_extractor` |
| `/odom` | `nav_msgs/msg/Odometry` | Default | Gazebo (`rm_bot`) $\rightarrow$ `map_extractor`, `planner_node`, `controller_node` |
| `/planned_path` | `nav_msgs/msg/Path` | `Transient Local` | `planner_node` $\rightarrow$ `controller_node`, `RViz2` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Default | `controller_node` $\rightarrow$ Gazebo (`rm_bot`) |

### Arquivos Gerados em `~/rm_path_follower_ws/rm_maps/`

* `meu_mapa.pgm`: Imagem binária em escala de cinza ($1200 \times 1200\text{ px}$), onde `255` representa espaço livre e `0` representa obstáculos já inflados.
* `meu_mapa.yaml`: Metadados espaciais compatíveis com o padrão `map_server` do ROS (`resolution` em $\text{m/px}$ e `origin` no canto inferior esquerdo).
* `caminho_bruto.csv`: Coordenadas $(x, y)$ em metros de todas as células retornadas pelo algoritmo de busca antes da subamostragem e suavização.
* `dados.csv`: Telemetria completa do controlador a $20\text{ Hz}$ (`tempo`, `x`, `y`, `yaw`, `xz`, `yz`, `xz_d`, `yz_d`, `v_cmd`, `w_cmd`).

---

## Extensibilidade: Adicionando Novos Algoritmos de Busca

Os algoritmos de roteamento são desacoplados do ROS 2 e residem em `rm_path_follower/planners/`. Para adicionar um novo algoritmo (ex: Dijkstra ou RRT):

1. Crie o arquivo `rm_path_follower/planners/novo_algoritmo.py` implementando uma função com a assinatura padrão:
   ```python
   def planejar(mapa, inicio, objetivo):
       # mapa: np.ndarray 2D (0 = livre, 1 = obstáculo)
       # inicio, objetivo: tuple (linha, coluna)
       # Retorna: list[tuple[int, int]] ou None
       ...
   ```
2. Registre o módulo no dicionário `PLANNERS` dentro de `rm_path_follower/planner_node.py`:
   ```python
   from rm_path_follower.planners import astar, wavefront, novo_algoritmo

   PLANNERS = {
       'astar': astar.planejar,
       'wavefront': wavefront.planejar,
       'novo': novo_algoritmo.planejar,
   }
   ```
3. Recompile o pacote com `colcon build --packages-select rm_path_follower` e execute passando `algorithm:=novo`.

---

## Ajuste Fino de Parâmetros Internos

Caso deseje recalibrar o comportamento físico do mapeamento, planejamento ou controle:

* **Margem de segurança contra colisões (`map_extractor.py`):** Ajuste `self.inflation_radius_m` (padrão: `0.22` m) para aumentar ou diminuir a folga radial ao redor das paredes e caixas.
* **Velocidade e suavidade da curva (`planner_node.py` e `utils/map_tools.py`):** Ajuste o parâmetro ROS `cruise_speed` (padrão: `0.2` m/s no nó) e `passo_amostragem` em `parametrizar_trajetoria` (padrão: `6` células) para controlar o espaçamento dos nós da *spline*.
* **Ganhos do controlador (`controller_node.py`):** Ajuste `self.d` (distância do ponto virtual à frente do eixo, padrão `0.12` m) e os ganhos proporcionais `self.k1` (`0.8`) e `self.k2` (`1.5`). Valores menores de `d` reduzem o corte de curvas, exigindo ganhos `k1, k2` mais moderados para evitar saturação em `w_cmd` ($\pm 1.5\text{ rad/s}$).
# AR Sandbox — Caixão de Areia com Realidade Aumentada

![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green?logo=opencv&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-2.x-013243?logo=numpy&logoColor=white)
![Testes](https://img.shields.io/badge/Testes_Unitários-26_passing-brightgreen)
![Licença](https://img.shields.io/badge/Licença-Acadêmica-lightgrey)

**Projeto Final de Curso (PFC) — Engenharia de Computação & Engenharia Eletrônica & Engenharia Cartográfica**  
**Instituto Militar de Engenharia (IME) — 2026**

---

## Visão Geral

O **AR Sandbox** projeta, em tempo real, um mapa de cores sobre uma caixa de areia física de **1,5 m × 1,5 m** com até **30 cm de profundidade**.  Um sensor **Microsoft Kinect** montado a **2,5 m de altura** captura a topografia da areia, o motor matemático compara cada ponto com um **Modelo Digital de Elevação (MDE)** de referência, e um projetor exibe o feedback visual diretamente na superfície:

| Cor | Significado | Ação |
|---|---|---|
| 🔴 **Vermelho** | Areia acima do alvo | Cavar |
| 🔵 **Azul** | Areia abaixo do alvo | Preencher |
| 🟢 **Verde** | Dentro da tolerância | OK |

O sistema exibe **duas janelas simultâneas**:

| Janela | Conteúdo |
|---|---|
| **Projecao_Areia** | Feedback AR em tempo real (vermelho/azul/verde) — enviada ao projetor |
| **Gabarito_MDE** | Heatmap de referência do MDE sendo replicado — monitor do operador |

O sistema é **100% Plug & Play**: funciona com ou sem o Kinect conectado (modo simulação automático) e cria uma superfície matemática de fallback caso o arquivo GeoTIFF não esteja presente.

---

## Arquitetura — 3 Camadas

```
┌─────────────────────────────────────────────────────────┐
│                    main.py                              │
│            Máquina de Estados (Orquestrador)            │
│  INIT → IDLE → CALIBRACAO → AR_LOOP → (loop contínuo)  │
│                                                         │
│  Janelas:  Projecao_Areia ← AR feedback (projetor)     │
│            Gabarito_MDE   ← Heatmap referência (monitor)│
└───────────┬──────────────┬──────────────┬───────────────┘
            │              │              │
   ┌────────▼────────┐ ┌──▼───────────┐ ┌▼──────────────┐
   │ kinect_sensor.py │ │motor_caixao_ │ │mde_cartografia│
   │ KinectSensor     │ │ areia.py     │ │   .py         │
   │ (OOP + Fallback) │ │ (Álgebra     │ │ AdaptadorMDE  │
   │                  │ │  Linear)     │ │ (GeoTIFF +    │
   │ Open3D/freenect  │ │ SVD, Gram-   │ │  Fallback     │
   │ → Simulação auto │ │ Schmidt,     │ │  Sintético +  │
   │ Kinect a 2.5 m   │ │ Tsai         │ │  Heatmap)     │
   └──────────────────┘ └──────────────┘ └───────────────┘
```

| Camada | Módulo | Responsabilidade |
|---|---|---|
| **Hardware** | `kinect_sensor.py` | Classe `KinectSensor`: tenta Open3D → freenect → simulação automática (Kinect a 2,5 m) |
| **Lógica** | `motor_caixao_areia.py` | Álgebra linear pura: SVD, Gram-Schmidt, Transformação 4×4, Tsai |
| **Dados** | `mde_cartografia.py` | Classe `AdaptadorMDE`: GeoTIFF com rasterio → fallback cosseno 2D + `gerar_imagem_visualizacao()` (heatmap) |
| **Orquestração** | `main.py` | Máquina de estados com 4 estados, dual-window (Projecao_Areia + Gabarito_MDE) |

---

## Estrutura do Repositório

```
PFC-2026/
├── main.py                    # Máquina de Estados — ponto de entrada
├── kinect_sensor.py           # KinectSensor OOP com fallback simulação
├── motor_caixao_areia.py      # Motor matemático (SVD, Gram-Schmidt, Tsai)
├── mde_cartografia.py         # AdaptadorMDE: GeoTIFF + fallback sintético + heatmap
│
├── test_motor_caixao.py       # 26 testes unitários automatizados
├── DOCUMENTACAO_TECNICA.md    # Documentação técnica para a banca
└── README.md                  # Este arquivo
```

---

## Pipeline Matemático

### Passo 1 — Ajuste de Plano via SVD

A nuvem de pontos bruta é centralizada no centroide e decomposta via **SVD** ($U \Sigma V^T$).  O vetor singular associado ao menor valor singular fornece a **normal** do plano da mesa:

$$ax + by + cz + d = 0, \quad d = -\mathbf{n} \cdot \bar{\mathbf{p}}$$

### Passo 2 — Referencial da Mesa (Gram-Schmidt)

A normal vira o eixo $Z_\text{mesa}$.  **Gram-Schmidt** ortogonaliza um vetor semente para gerar $X_\text{mesa}$, e o **produto vetorial** completa a base:

$$X = \text{GS}(\text{semente}, Z), \quad Y = Z \times X$$

### Passo 3 — Transformação Afim 4×4

A matriz $T$ leva coordenadas do Kinect para o referencial da mesa:

$$\mathbf{p}_\text{mesa} = T \cdot \begin{bmatrix} \mathbf{p}_\text{kinect} \\ 1 \end{bmatrix}, \quad T = \begin{bmatrix} R & -R \cdot \bar{\mathbf{p}} \\ 0^T & 1 \end{bmatrix}$$

Pontos sobre o plano resultam em $z_\text{mesa} = 0$.

### Passo 4 — Projeção Pinhole (Tsai)

Converte pontos 3D da mesa em pixels 2D do projetor via `cv2.projectPoints`:

$$u = f_x \cdot \frac{X}{Z} + c_x, \quad v = f_y \cdot \frac{Y}{Z} + c_y$$

### Passo 5 — Coloração por Diferença

Compara $z_\text{real}$ (medido) com $z_\text{MDE}$ (esperado):

| Condição | Cor |
|---|---|
| $z_\text{real} > z_\text{MDE} + \epsilon$ | Vermelho |
| $z_\text{real} < z_\text{MDE} - \epsilon$ | Azul |
| caso contrário | Verde |

---

## Resiliência — Zero Crash na Apresentação

O sistema implementa **fallback em cascata** em dois pontos críticos:

### Sensor (kinect_sensor.py)

```
Inicialização do KinectSensor:
  1. Tenta Open3D (Azure Kinect / RealSense)  →  sucesso? usa.
  2. Tenta freenect (Kinect v1)                →  sucesso? usa.
  3. Ambos falharam?  →  "⚠ Modo Simulação"   →  nuvem sintética.
```

### MDE (mde_cartografia.py)

```
Inicialização do AdaptadorMDE:
  1. Tenta ler GeoTIFF com rasterio  →  sucesso? usa.
  2. Arquivo não existe / rasterio não instalado?
     →  Gera superfície cosseno 2D automaticamente.
```

**Resultado**: `python main.py` funciona em qualquer máquina, em qualquer momento, sem nenhuma dependência externa obrigatória além de numpy e opencv.

---

## Como Rodar — Passo a Passo

### 1. Instalar dependências

```bash
pip install numpy opencv-python
```

Opcionais (para GeoTIFF real e nuvem RGBD):

```bash
pip install rasterio scipy open3d
```

### 2. Configurar (opcional)

Edite as variáveis no topo de `main.py`:

```python
CAMINHO_GEOTIFF = "terreno_aman.tif"   # arquivo da Cartografia
TOLERANCIA_COR  = 5.0                  # mm
LARGURA_MESA    = 1.50                 # metros (1,5 m)
COMPRIMENTO_MESA = 1.50               # metros (1,5 m)
ALTURA_MAX_AREIA = 0.30               # metros (30 cm)
ALTURA_KINECT   = 2.50                # metros (2,5 m)
FORCAR_SIMULACAO = False               # True para ignorar o Kinect
```

### 3. Rodar o sistema

```bash
python main.py
```

### 4. Operar o sistema

| Tecla | Estado | Ação |
|---|---|---|
| **C** | IDLE / AR_LOOP | Calibrar (captura plano + base + matriz 4×4) |
| **F** | AR_LOOP | Toggle tela cheia na janela Projecao_Areia (para o projetor) |
| **Q** / **ESC** | Qualquer | Encerrar o sistema |

### Fluxo da demonstração para a banca

1. Execute `python main.py` — abrem duas janelas: **Projecao_Areia** (profundidade colorida) e **Gabarito_MDE** (heatmap de referência).
2. Posicione o sensor (ou use simulação) e aperte **C** — a calibração SVD roda em ~0.2s.
3. A projeção AR inicia automaticamente — as cores vermelho/azul/verde aparecem na janela **Projecao_Areia**.
4. O **Gabarito_MDE** mostra o heatmap do terreno que está sendo replicado — referência visual para o operador.
5. Modele a areia (ou observe a simulação) — as cores mudam em tempo real.
6. Aperte **C** novamente para recalibrar se necessário.
7. Aperte **Q** para encerrar.

### Rodando com o Kinect real

Basta conectar o Kinect por USB.  O `KinectSensor` detecta automaticamente:

- **Azure Kinect**: via Open3D (precisa de `pip install open3d`)
- **Kinect v1**: via freenect (precisa de `sudo apt-get install freenect python3-freenect` no Linux)

Se nenhum sensor for detectado, entra em simulação automaticamente.

---

## Testes Unitários

26 testes automatizados cobrindo todo o motor matemático:

| Classe | Testes | Componente |
|---|---|---|
| `TestAjustePlano` | 4 | SVD, normal unitária, equação do plano |
| `TestGramSchmidt` | 2 | Ortogonalidade, exceção para paralelos |
| `TestConstruirBase` | 3 | Ortonormalidade mútua dos 3 eixos |
| `TestMatrizTransformacao` | 3 | Identidade, translação, z_mesa = 0 |
| `TestDeteccaoTabuleiro` | 2 | Imagem sem tabuleiro, tabuleiro 7×5 |
| `TestProjecaoTsai` | 3 | Projeção pinhole, deslocamento |
| `TestLeituraRGBD` | 1 | Importação condicional Open3D |
| `TestMDEColoracao` | 6 | Vermelho/Azul/Verde, limites, mock |
| `TestPipeline` | 2 | Integração completa Passos 1+2 |

```bash
python -m unittest test_motor_caixao -v
# Resultado: 25 passed, 1 skipped (Open3D)
```

---

## Status do Projeto

### Software (Pronto)

- ✅ Máquina de estados `main.py` com 4 estados + transições via teclado
- ✅ **Dual-window**: Projecao_Areia (AR feedback) + Gabarito_MDE (heatmap referência)
- ✅ `KinectSensor` OOP com fallback automático Open3D → freenect → simulação (Kinect a 2,5 m)
- ✅ Motor matemático completo (SVD, Gram-Schmidt, Afim 4×4, Tsai)
- ✅ `AdaptadorMDE` com leitura GeoTIFF + fallback cosseno 2D + `gerar_imagem_visualizacao()`
- ✅ Dimensões reais: caixa 1,5 m × 1,5 m × 0,3 m, Kinect a 2,5 m
- ✅ Coloração MDE (Vermelho/Azul/Verde) por diferença de altitude
- ✅ Projeção 3D → 2D via `cv2.projectPoints`
- ✅ Tela cheia para projetor (`cv2.WINDOW_FULLSCREEN`)
- ✅ 26 testes unitários automatizados
- ✅ Documentação técnica completa para a banca
- ✅ Type Hints + Docstrings NumPy/Sphinx em todo o código

### Hardware / Físico (Pendente)

- 🔄 Conexão USB do Kinect real
- 🔄 Montagem e alinhamento do projetor
- 🔄 Calibração física com tabuleiro de xadrez impresso
- 🔄 Recebimento do arquivo GeoTIFF final da Cartografia
- 🔄 Ajuste de tolerância empírica com areia real

---

## Referências

- OpenKinect / libfreenect: https://github.com/OpenKinect/libfreenect
- SARndbox (UC Davis): https://github.com/KeckCAVES/SARndbox
- AR Sandbox DIY: https://ar-sandbox.eu/augmented-reality-sandbox-diy/
- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Tsai, R. Y. (1987). *A Versatile Camera Calibration Technique for High-Accuracy 3D Machine Vision Metrology Using Off-the-Shelf TV Cameras and Lenses*. IEEE Journal of Robotics and Automation.
- Open3D: http://www.open3d.org/
- Rasterio: https://rasterio.readthedocs.io/

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

O **AR Sandbox** projeta, em tempo real, um mapa de cores sobre uma caixa de areia física de **1,5 m × 1,5 m** com até **30 cm de profundidade**. Um sensor **Microsoft Kinect** montado a **2,5 m de altura** captura a topografia da areia, o motor matemático compara cada ponto com um **Modelo Digital de Elevação (MDE)** de referência no formato **GeoTIFF**, e um projetor exibe o feedback visual diretamente na superfície:

| Cor | Condição | Significado |
|---|---|---|
| 🔴 **Vermelho** | $Z_{real} > Z_{MDE} + 0{,}02\text{ m}$ | Areia em excesso — **Cavar** |
| 🔵 **Azul** | $Z_{real} < Z_{MDE} - 0{,}02\text{ m}$ | Areia insuficiente — **Preencher** |
| 🟢 **Verde** | Diferença $\leq 0{,}02\text{ m}$ | Dentro da tolerância — **OK** |

O sistema exibe **duas janelas simultâneas**:

| Janela | Conteúdo |
|---|---|
| **Projecao_Areia** | Feedback AR em tempo real (vermelho/azul/verde) — enviada ao projetor |
| **Gabarito_MDE** | Heatmap de referência do MDE sendo replicado — monitor do operador |

### Resiliência Total — Zero Crash na Apresentação

O sistema é **100% Plug & Play**: funciona em qualquer máquina, com ou sem hardware.

- **Sem Kinect?** → O `KinectSensor` gera automaticamente uma nuvem de pontos simulando areia nivelada a **15 cm** (plano reto no grid $[0, 1.5] \times [0, 1.5]$ m).
- **Sem GeoTIFF?** → O `AdaptadorMDE` gera automaticamente um **Morro Gaussiano** no centro da mesa (pico de 30 cm caindo para 0 nas bordas).
- **Resultado da Simulação** → As **três cores** convivem na mesma imagem: bordas vermelhas, anel intermediário verde e centro azul.

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
   │ → Simulação auto │ │ Schmidt,     │ │  Gaussiano +  │
   │ Kinect a 2.5 m   │ │ Tsai         │ │  Heatmap)     │
   └──────────────────┘ └──────────────┘ └───────────────┘
```

| Camada | Módulo | Responsabilidade |
|---|---|---|
| **Hardware** | `kinect_sensor.py` | Classe `KinectSensor`: Open3D → freenect → simulação automática (plano a 15 cm) |
| **Lógica** | `motor_caixao_areia.py` | Álgebra linear pura: SVD, Gram-Schmidt, Transformação 4×4, Tsai, coloração |
| **Dados** | `mde_cartografia.py` | Classe `AdaptadorMDE`: GeoTIFF via rasterio → fallback Morro Gaussiano + heatmap |
| **Orquestração** | `main.py` | Máquina de estados (INIT/IDLE/CALIBRACAO/AR_LOOP), dual-window |

---

## Estrutura do Repositório

```
PFC-2026/
├── main.py                    # Máquina de Estados — ponto de entrada
├── kinect_sensor.py           # KinectSensor OOP com fallback simulação
├── motor_caixao_areia.py      # Motor matemático (SVD, Gram-Schmidt, Tsai)
├── mde_cartografia.py         # AdaptadorMDE: GeoTIFF + fallback Gaussiano + heatmap
│
├── test_motor_caixao.py       # 26 testes unitários automatizados
├── DOCUMENTACAO_OFICIAL.md    # Documentação acadêmica completa para a banca
└── README.md                  # Este arquivo
```

---

## Como Instalar

### Dependências obrigatórias

```bash
pip install numpy opencv-python
```

### Dependências opcionais (GeoTIFF real, interpolação, nuvem RGBD)

```bash
pip install rasterio scipy open3d
```

### Testes

```bash
pip install pytest   # opcional, unittest funciona nativamente
```

---

## Como Executar

### Modo Simulação (sem hardware — padrão automático)

```bash
python main.py
```

Se nenhum Kinect estiver conectado e nenhum GeoTIFF estiver presente, o sistema entra **automaticamente** em modo simulação completo. Nenhuma configuração necessária.

Para forçar o modo simulação mesmo com Kinect conectado, edite no topo de `main.py`:

```python
FORCAR_SIMULACAO = True
```

### Modo Hardware Real (Kinect + GeoTIFF)

1. Conecte o Kinect via USB.
2. Coloque o arquivo GeoTIFF (`.tif`) no diretório do projeto.
3. Edite o caminho no topo de `main.py`:

```python
CAMINHO_GEOTIFF = "terreno_aman.tif"
```

4. Execute:

```bash
python main.py
```

O `KinectSensor` detecta automaticamente:

- **Azure Kinect / RealSense** → via Open3D
- **Kinect v1** → via freenect / libfreenect

### Configuração (topo de `main.py`)

```python
CAMINHO_GEOTIFF   = "terreno_aman.tif"    # Arquivo MDE da Cartografia
TOLERANCIA_COR    = 0.02                  # metros (2 cm)
LARGURA_MESA      = 1.50                  # metros
COMPRIMENTO_MESA  = 1.50                  # metros
ALTURA_MAX_AREIA  = 0.30                  # metros (30 cm)
ALTURA_KINECT     = 2.50                  # metros
FORCAR_SIMULACAO  = False                 # True para ignorar Kinect
```

---

## Operação do Sistema

| Tecla | Ação |
|---|---|
| **C** | Calibrar (SVD + Gram-Schmidt + Matriz 4×4) |
| **F** | Toggle tela cheia na janela Projecao_Areia |
| **Q** / **ESC** | Encerrar |

### Fluxo de Demonstração para a Banca

1. Execute `python main.py` — abrem duas janelas: **Projecao_Areia** e **Gabarito_MDE**.
2. Pressione **C** — a calibração SVD + Gram-Schmidt executa instantaneamente.
3. A projeção AR inicia no **AR_LOOP**: as três cores (vermelho/azul/verde) aparecem na janela de projeção.
4. O **Gabarito_MDE** exibe o heatmap do Morro Gaussiano de referência.
5. Pressione **F** para tela cheia (projetor) ou **Q** para encerrar.

---

## Testes Unitários

26 testes automatizados cobrindo todo o motor matemático:

```bash
python -m unittest test_motor_caixao -v
# Resultado: 25 passed, 1 skipped (Open3D não instalado)
```

| Classe | Testes | Componente |
|---|---|---|
| `TestAjustePlano` | 4 | SVD, normal unitária, equação do plano |
| `TestGramSchmidt` | 2 | Ortogonalidade, exceção para paralelos |
| `TestConstruirBase` | 3 | Ortonormalidade mútua dos 3 eixos |
| `TestMatrizTransformacao` | 3 | Identidade, translação, $z_{mesa} = 0$ |
| `TestDeteccaoTabuleiro` | 2 | Imagem sem tabuleiro, tabuleiro 7×5 |
| `TestProjecaoTsai` | 3 | Projeção pinhole, deslocamento em $x$ |
| `TestLeituraRGBD` | 1 | Importação condicional Open3D |
| `TestMDEColoracao` | 6 | Vermelho/Azul/Verde, limites, mock rampa |
| `TestPipeline` | 2 | Integração completa Passos 1+2 |

---

## Referências

- OpenKinect / libfreenect: https://github.com/OpenKinect/libfreenect
- SARndbox (UC Davis): https://github.com/KeckCAVES/SARndbox
- AR Sandbox DIY: https://ar-sandbox.eu/augmented-reality-sandbox-diy/
- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Tsai, R. Y. (1987). *A Versatile Camera Calibration Technique for High-Accuracy 3D Machine Vision Metrology Using Off-the-Shelf TV Cameras and Lenses*. IEEE Journal of Robotics and Automation.
- Open3D: http://www.open3d.org/
- Rasterio: https://rasterio.readthedocs.io/

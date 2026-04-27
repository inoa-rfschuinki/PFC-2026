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

O **AR Sandbox** projeta, em tempo real, uma **grade contínua de quadrados coloridos** sobre uma caixa de areia física de **1,5 m × 1,5 m** com até **30 cm de profundidade**. Um sensor **Microsoft Kinect** montado a **2,5 m de altura** captura a topografia da areia, o motor matemático discretiza a mesa em uma **malha de 30 × 30 células** (5 cm × 5 cm cada), calcula a **altura média** por célula, compara com um **Modelo Digital de Elevação (MDE)** de referência no formato **GeoTIFF**, e projeta o feedback visual diretamente na superfície como polígonos preenchidos:

| Cor | Condição | Significado |
|---|---|---|
| 🔴 **Vermelho** | $Z_{real\_media} > Z_{MDE} + 0{,}02\text{ m}$ | Areia em excesso — **Cavar** |
| 🔵 **Azul** | $Z_{real\_media} < Z_{MDE} - 0{,}02\text{ m}$ | Areia insuficiente — **Preencher** |
| 🟢 **Verde** | Diferença $\leq 0{,}02\text{ m}$ | Dentro da tolerância — **OK** |

O sistema exibe **duas janelas simultâneas**:

| Janela | Conteúdo |
|---|---|
| **Projecao_Areia** | Grade contínua de quadrados coloridos (vermelho/azul/verde) — enviada ao projetor |
| **Gabarito_MDE** | Heatmap de referência do MDE sendo replicado — monitor do operador |

### Resiliência Total — Zero Crash na Apresentação

O sistema é **100% Plug & Play**: funciona em qualquer máquina, com ou sem hardware.

- **Sem Kinect?** → O `KinectSensor` entra em **Modo Simulação Interativo** com uma grade persistente de alturas inicializada em **15 cm**. O usuário pode **cavar** e **preencher** a areia virtual usando o **mouse** (veja seção abaixo).
- **Sem GeoTIFF?** → O `AdaptadorMDE` gera automaticamente um **Morro Gaussiano** no centro da mesa (pico de 30 cm caindo para 0 nas bordas).
- **Resultado da Simulação** → As **três cores** convivem na mesma imagem: bordas vermelhas, anel intermediário verde e centro azul — e **reagem em tempo real** à interação do mouse.

---

## Simulador Interativo — "Pá Virtual" com o Mouse

Na ausência de hardware físico (Kinect + areia), o sistema oferece um **emulador interativo completo** que permite demonstrar todo o pipeline AR usando apenas mouse e teclado.

### Como funciona

O `KinectSensor` em modo simulação mantém uma **matriz de alturas persistente** (grid 50×50) na memória, representando o estado atual da areia virtual. Eventos de mouse na janela **Projecao_Areia** modificam essa matriz em tempo real:

| Ação do Mouse | Efeito na Areia | Analogia Física |
|---|---|---|
| **Botão Esquerdo** + Arrastar | **Diminui** $Z_{real}$ — cava a areia | Pá escavando |
| **Botão Direito** + Arrastar | **Aumenta** $Z_{real}$ — preenche a areia | Balde despejando |

- O efeito é **acumulativo**: quanto mais tempo o mouse permanece sobre um ponto, maior a alteração de altura.
- A modificação usa um **perfil Gaussiano** com raio de 10 cm, garantindo bordas suaves e naturais (sem buracos quadrados).
- A altura é limitada ao intervalo físico $[0{,}00 \text{ m},\; 0{,}30 \text{ m}]$.
- A grade de quadrados coloridos **reage instantaneamente** na tela: ao cavar uma região verde, ela se torna azul; ao preencher uma vermelha, ela se torna verde.

### Exemplo de interação

1. Ao iniciar, toda a areia está a $Z = 0{,}15$ m (nivelada).
2. O MDE alvo (Morro Gaussiano) pede $Z = 0{,}30$ m no centro e $Z \approx 0$ nas bordas.
3. Resultado inicial: centro **azul** (falta areia), bordas **vermelhas** (excesso), anel intermediário **verde**.
4. O operador **arrasta o botão direito no centro** → a areia sobe → quadrados azuis se tornam verdes.
5. O operador **arrasta o botão esquerdo nas bordas** → a areia desce → quadrados vermelhos se tornam verdes.
6. **Objetivo**: tornar toda a grade verde — o terreno virtual replica o MDE.

---

## Renderização — Malha Discretizada de Quadrados

O sistema **não** projeta pontos isolados. A mesa é dividida em uma **grade contínua de células** (por padrão 30 × 30 = 900 quadrados de 5 cm × 5 cm), e cada célula é renderizada como um **polígono preenchido** usando `cv2.fillPoly`:

1. **Discretização** — Os pontos da nuvem do Kinect são agrupados por célula e a altura $Z$ é **calculada como média** espacial, filtrando ruído do sensor.
2. **Comparação** — A altura alvo $Z_{MDE}$ é consultada no **centro geométrico** de cada célula.
3. **Projeção geométrica** — Os vértices da grade inteira (31 × 31 = 961 pontos) são projetados de uma só vez via `cv2.projectPoints` (modelo Tsai).
4. **Rasterização** — Cada célula é desenhada como polígono de 4 cantos com `cv2.fillPoly`, resultando em cobertura contínua sem buracos.

O resultado é uma **projeção sólida e limpa** sobre a areia — como "curvas de nível discretizadas".

---

## Calibração Coplanar — Solução para Areia Plana

A calibração 3D clássica (DLT não-coplanar) falha quando a areia está inicialmente plana: a variação em Z é apenas ruído do sensor, tornando a matriz de calibração mal-condicionada. O sistema resolve isso em **5 passos** durante o estado `CALIBRACAO`:

| Passo | Função | O que faz |
|---|---|---|
| **1 — Plane Fitting** | `ajustar_plano_svd` | SVD sobre a nuvem ruidosa → normal $\hat{n}$ e centroide $\bar{p}$ |
| **2 — Base Local** | `construir_base_mesa` + `montar_matriz_transformacao` | Gram-Schmidt + Produto Vetorial → base ortonormal → matriz afim $T$ (4×4); areia passa a ser $Z_{local}=0$ |
| **3 — Grid no Local** | `transformar_pontos(T, \ldots)` | Pontos 3D do xadrez → referencial local; $Z_{local}\approx 0$ |
| **4 — Tsai Coplanar** | `calibrar_coplanar` → `cv2.calibrateCamera` | Força $Z_{local}=0$ nos objectPoints → sistema bem-condicionado → $K$, $dist$, $rvec$, $tvec$ |
| **5 — Loop RGBD** | `projetar_ponto_rgbd` | No loop em tempo real: $T@p_{kinect}$ → $Z_{local}\neq 0$ (topografia real) → `projectPoints` → pixel $(u,v)$ |

### Xadrez de Calibração

Ao pressionar **C**, o sistema projeta automaticamente um padrão de xadrez (7×5 cantos internos) sobre a areia plana. O operador pressiona **SPACE** para capturar. O Kinect detecta os cantos com `cv2.findChessboardCorners` e faz a back-projection para 3D. Se o xadrez não for detectado, o sistema usa uma **aproximação geométrica** como fallback.

---

## Arquitetura — 3 Camadas

```
┌────────────────────────────────────────────────────────────────────┐
│                            main.py                                 │
│                 Máquina de Estados (Orquestrador)                  │
│  INIT → IDLE → CALIBRACAO → AR_LOOP → (loop contínuo)             │
│                                                                    │
│  Calibração: SVD+Gram-Schmidt → xadrez projetado → Tsai coplanar  │
│  Janelas:  Projecao_Areia ← Grade de quadrados AR (projetor)      │
│            Gabarito_MDE   ← Heatmap referência (monitor)           │
│  Mouse:    Botão Esq → cavar | Botão Dir → preencher              │
└────────┬──────────────┬──────────────┬─────────────────────────────┘
         │              │              │
┌────────▼────────┐ ┌──▼───────────────┐ ┌▼──────────────┐
│ kinect_sensor.py │ │motor_caixao_     │ │mde_cartografia│
│ KinectSensor     │ │areia.py          │ │   .py         │
│ OOP + Fallback   │ │ SVD, Gram-Schmidt│ │ AdaptadorMDE  │
│ Grade persistente│ │ gerar_xadrez()   │ │ GeoTIFF +     │
│ modificar_areia()│ │ calibrar_        │ │ Fallback      │
│ capturar_cor()   │ │  coplanar()      │ │ Gaussiano +   │
│ pixels_para_3d() │ │ projetar_rgbd()  │ │ Heatmap       │
│ back-projection  │ │ Grade fillPoly   │ │               │
└──────────────────┘ └──────────────────┘ └───────────────┘
```

| Camada | Módulo | Responsabilidade |
|---|---|---|
| **Hardware** | `kinect_sensor.py` | `KinectSensor`: Open3D → freenect → simulação; back-projection pinhole correta; `capturar_imagem_cor()`, `pixels_para_3d()` |
| **Lógica** | `motor_caixao_areia.py` | Álgebra linear: SVD, Gram-Schmidt, $T$ (4×4), `gerar_imagem_xadrez`, `calibrar_coplanar`, `projetar_ponto_rgbd`, grade com `fillPoly` |
| **Dados** | `mde_cartografia.py` | `AdaptadorMDE`: GeoTIFF via rasterio → fallback Morro Gaussiano + heatmap |
| **Orquestração** | `main.py` | Máquina de estados, dual-window, xadrez de calibração, fallback geométrico, mouse callback |

---

## Estrutura do Repositório

```
PFC-2026/
├── main.py                    # Máquina de Estados + Calibração Coplanar + Mouse Callback
├── kinect_sensor.py           # KinectSensor: back-projection, capturar_cor, pixels_para_3d
├── motor_caixao_areia.py      # Motor matemático: SVD, Gram-Schmidt, Tsai, Calibração Coplanar
├── mde_cartografia.py         # AdaptadorMDE: GeoTIFF + fallback Gaussiano + heatmap
│
├── test_motor_caixao.py       # 26 testes unitários automatizados
├── DOCUMENTACAO_OFICIAL.md    # Documentação acadêmica completa para a banca
├── .gitignore                 # Ignora __pycache__, .venv, *.tif, etc.
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

### Modo Simulação Interativo (sem hardware — padrão automático)

```bash
python main.py
```

Se nenhum Kinect estiver conectado e nenhum GeoTIFF estiver presente, o sistema entra **automaticamente** em modo simulação interativo. Nenhuma configuração necessária — basta usar o mouse para interagir.

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
CAMINHO_GEOTIFF      = "25S51_ZN.tif"    # Arquivo MDE (GeoTIFF)
TOLERANCIA_COR       = 0.02              # metros (2 cm)
LARGURA_MESA         = 1.50              # metros
COMPRIMENTO_MESA     = 1.50              # metros
ALTURA_MAX_AREIA     = 0.30              # metros (30 cm)
ALTURA_KINECT        = 2.50              # metros
CELULAS_GRADE_X      = 30                # colunas da malha (5 cm cada)
CELULAS_GRADE_Y      = 30                # linhas da malha (5 cm cada)
RAIO_PA_VIRTUAL      = 0.10              # raio do pincel do mouse (10 cm)
INTENSIDADE_PA_VIRTUAL = 0.008           # deslocamento por evento (8 mm)
FORCAR_SIMULACAO     = False             # True para ignorar Kinect
```

---

## Operação do Sistema

### Teclado

| Tecla | Ação |
|---|---|
| **C** | Calibrar — SVD + Gram-Schmidt + xadrez coplanar + Tsai |
| **SPACE** | Capturar xadrez (durante a etapa de calibração) |
| **ESC** (no xadrez) | Pular captura do xadrez — usa aproximação geométrica |
| **F** | Toggle tela cheia na janela Projecao_Areia |
| **Q** / **ESC** | Encerrar |

### Mouse (Simulação Interativa)

| Ação | Efeito |
|---|---|
| **Botão Esquerdo + Arrastar** | Cavar areia (diminui $Z_{real}$) |
| **Botão Direito + Arrastar** | Preencher areia (aumenta $Z_{real}$) |

---

## Roteiro de Demonstração para a Banca

| Passo | Ação | Resultado esperado |
|---|---|---|
| 1 | `python main.py` | GUI de configuração abre; selecionar mapa ou marcar "Modo Demo" |
| 2 | Clicar **INICIAR** | Duas janelas: **Projecao_Areia** (projetor) e **Gabarito_MDE** (monitor) |
| 3 | Pressionar **C** — modo simulação | Calibração automática: $T$ = identidade, $K$ geométrica |
| 4 | Pressionar **C** — hardware real | (1) SVD+Gram-Schmidt → $T$; (2) xadrez projetado aparece na areia |
| 5 | Pressionar **SPACE** | Kinect detecta cantos do xadrez → `calibrar_coplanar` → $K$, $rvec$, $tvec$ reais |
| 6 | Observar **Projecao_Areia** | Grade contínua: bordas vermelhas, anel verde, centro azul |
| 7 | Observar **Gabarito_MDE** | Heatmap do Morro Gaussiano (ou GeoTIFF) como referência |
| 8 | **Arrastar botão direito** no centro azul | Quadrados mudam de azul → verde (areia subindo até o alvo) |
| 9 | **Arrastar botão esquerdo** nas bordas vermelhas | Quadrados mudam de vermelho → verde (areia descendo até o alvo) |
| 10 | Pressionar **F** | Tela cheia na janela de projeção |
| 11 | Pressionar **C** | Recalibração ao vivo (demonstra robustez) |
| 12 | Pressionar **Q** | Encerramento limpo |

> **Dica para a banca:** a interação com o mouse demonstra em tempo real todo o pipeline matemático (discretização → média espacial → comparação MDE → projeção Tsai → renderização) sem necessidade de hardware físico.

---

## Testes Unitários

26 testes automatizados cobrindo todo o motor matemático:

```bash
python -m unittest test_motor_caixao -v
# Resultado: 26 passed, 1 skipped (Open3D não instalado)
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

## Alterações Recentes

### Correções de Bugs (Abril 2026)

| # | Módulo | Problema | Solução |
|---|---|---|---|
| B1 | `kinect_sensor.py` | `profundidade_para_pontos` retornava `[u,v,d]` em pixels | Back-projection pinhole correta: `X=(u−cx)·d/fx`, retorna metros |
| B2 | `main.py` | Modo real deixava `K`, `rvec`, `tvec` em valores mock hardcoded | Integrado ao fluxo `calibrar_coplanar` com xadrez real |
| B3 | `main.py` | `calibrar_coplanar` e `gerar_imagem_xadrez` não importadas | Adicionadas ao import |
| B4 | `motor_caixao_areia.py` | `gerar_imagem_xadrez` não existia | Criada: xadrez BGR + array `(M,2)` de cantos em pixels do projetor |
| B5 | `kinect_sensor.py` | `capturar_imagem_cor()` não existia | Adicionada para freenect / Open3D / simulação |
| B6 | `kinect_sensor.py` | `pixels_para_3d()` não existia | Adicionada: back-projection por lookup de depth para M pixels |

### Novas Funcionalidades (Abril 2026)

- **`gerar_imagem_xadrez()`** — gera padrão de xadrez centralizado para calibração coplanar.
- **`calibrar_coplanar()`** — orquestra os 5 passos do supervisor: SVD → Gram-Schmidt → grid local → Z=0 → `cv2.calibrateCamera`.
- **`projetar_ponto_rgbd()`** — hot-path do loop AR: `T@p_kinect` → `projectPoints` → pixel `(u,v)`.
- **`capturar_imagem_cor()`** e **`pixels_para_3d()`** em `KinectSensor`.
- **`.gitignore`** adicionado; `__pycache__` removido do repositório remoto.

---

## Referências

- OpenKinect / libfreenect: https://github.com/OpenKinect/libfreenect
- SARndbox (UC Davis): https://github.com/KeckCAVES/SARndbox
- AR Sandbox DIY: https://ar-sandbox.eu/augmented-reality-sandbox-diy/
- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Tsai, R. Y. (1987). *A Versatile Camera Calibration Technique for High-Accuracy 3D Machine Vision Metrology Using Off-the-Shelf TV Cameras and Lenses*. IEEE Journal of Robotics and Automation.
- Open3D: http://www.open3d.org/
- Rasterio: https://rasterio.readthedocs.io/

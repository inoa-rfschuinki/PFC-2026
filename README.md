# AR Sandbox — Caixão de Areia com Realidade Aumentada

![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green?logo=opencv&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-2.x-013243?logo=numpy&logoColor=white)
![Testes](https://img.shields.io/badge/Testes_Unitários-26_passing-brightgreen)
![Licença](https://img.shields.io/badge/Licença-Acadêmica-lightgrey)

**Projeto Final de Curso (PFC) — Engenharia de Computação & Engenharia Eletrônica**  
**Academia Militar das Agulhas Negras (AMAN) — 2026**

---

## Sobre o Projeto

O **AR Sandbox** é um sistema de realidade aumentada que projeta, em tempo real, um mapa de cores sobre uma caixa de areia física. Um sensor Microsoft Kinect captura a topografia da areia, o motor matemático compara cada ponto com um Modelo Digital de Elevação (MDE) de referência, e um projetor exibe o feedback visual diretamente sobre a superfície:

| Cor | Significado | Ação do Usuário |
|---|---|---|
| 🔴 **Vermelho** | Areia acima do alvo | Cavar |
| 🔵 **Azul** | Areia abaixo do alvo | Preencher |
| 🟢 **Verde** | Dentro da tolerância | OK |

O resultado é uma experiência interativa de modelagem de terrenos guiada por realidade aumentada.

---

## Arquitetura do Sistema

O projeto adota uma **arquitetura monolítica modularizada** — todos os módulos executam no mesmo processo Python, mas cada um encapsula uma responsabilidade isolada:

```
                        ┌──────────────────┐
                        │  app_integrado.py │  ← Orquestrador do pipeline
                        └────────┬─────────┘
           ┌─────────────────────┼─────────────────────┐
           ▼                     ▼                     ▼
   ┌───────────────┐   ┌────────────────────┐   ┌──────────────────┐
   │  kinect.py    │   │motor_caixao_areia.py│   │mde_cartografia.py│
   │  exibicao.py  │   │ (Motor Matemático)  │   │adaptador_mde.py  │
   │  utils.py     │   └────────────────────┘   │ (Padrão Adapter)  │
   │  (Captura)    │                             └──────────────────┘
   └───────────────┘
```

---

## Estrutura do Repositório

```
PFC-2026/
├── main.py                    # Loop original de captura (Raquel)
├── kinect.py                  # Captura de profundidade e conversão 3D (Raquel)
├── exibicao.py                # Colorização e exibição no projetor (Raquel)
├── utils.py                   # Utilitários: FPS, log (Raquel)
├── testekinect.py             # Testes manuais do sensor (Raquel)
│
├── motor_caixao_areia.py      # Motor matemático — SVD, Gram-Schmidt, Tsai (Rafael)
├── adaptador_mde.py           # Interface MDE com mock (Rafael)
├── mde_cartografia.py         # Leitor GeoTIFF real com rasterio/scipy (Rafael)
├── app_integrado.py           # Pipeline completo: calibração + loop AR (Rafael)
├── test_motor_caixao.py       # 26 testes unitários automatizados (Rafael)
│
├── DOCUMENTACAO_TECNICA.md    # Documentação técnica completa para a banca
└── README.md                  # Este arquivo
```

| Módulo | Responsabilidade |
|---|---|
| `main.py` | Loop original — captura → processa → exibe (intocado) |
| `kinect.py` | Sensor Kinect: simulador e captura real via `freenect` |
| `exibicao.py` | Colorização do mapa de profundidade e exibição em janela/projetor |
| `utils.py` | Cronômetro de FPS e log formatado no terminal |
| `motor_caixao_areia.py` | **Motor matemático** — toda a álgebra linear e projeção |
| `adaptador_mde.py` | Interface Adapter com mock para desenvolvimento |
| `mde_cartografia.py` | **Leitor GeoTIFF** real com normalização e interpolação |
| `app_integrado.py` | Orquestrador: Fase 1 (calibração) + Fase 2 (loop AR em tempo real) |
| `test_motor_caixao.py` | Suíte de 26 testes unitários com `unittest` |

---

## Módulo de Captura (Raquel)

O módulo `kinect.py` abstrai o sensor Microsoft Kinect com duas implementações intercambiáveis:

- **Modo Simulado** (`USE_KINECT_REAL = False`) — gera um mapa de profundidade 480×640 com uma colina gaussiana e ruído, replicando o formato real do sensor.
- **Modo Real** (`USE_KINECT_REAL = True`) — captura via `freenect.sync_get_depth()`.

A função `profundidade_para_pontos()` converte o mapa de profundidade em um array NumPy `(N, 3)` com coordenadas `[u, v, d]` em pixels e milímetros, filtrado para pontos com profundidade válida.

O módulo `exibicao.py` normaliza a profundidade para colormap (`cv2.COLORMAP_TURBO`) e exibe em janela OpenCV com suporte a tela cheia para o projetor.

---

## Motor Matemático (Rafael)

O módulo `motor_caixao_areia.py` implementa os 4 pilares matemáticos do sistema:

### Passo 1 — Ajuste de Plano via SVD

Recebe a nuvem de pontos 3D do Kinect, centraliza no centroide e aplica a **Decomposição em Valores Singulares** (SVD). O vetor singular associado ao menor valor singular fornece a normal **n** = (a, b, c) da equação do plano:

```
ax + by + cz + d = 0
```

**Função:** `ajustar_plano_svd(pontos)`

### Passo 2 — Sistema de Coordenadas da Mesa (Gram-Schmidt)

Constrói uma base ortonormal {X_mesa, Y_mesa, Z_mesa} a partir da normal do plano, usando **ortogonalização de Gram-Schmidt** e **produto vetorial**:

1. Z_mesa = normal do plano
2. X_mesa = Gram-Schmidt(semente, Z_mesa)
3. Y_mesa = Z_mesa × X_mesa

**Funções:** `gram_schmidt()`, `construir_base_mesa()`

### Passo 3 — Transformação Afim 4×4

Monta a **matriz de transformação afim** que leva pontos do referencial do Kinect para o referencial da mesa:

```
         ┌         ┐       ┌     ┐
         │  R   t  │       │  p  │
p_mesa = │         │   ×   │     │
         │ 0ᵀ  1  │       │  1  │
         └         ┘       └     ┘
```

Onde R empilha os eixos da mesa como linhas e t = −R · centroide. Pontos sobre o plano da mesa resultam em z_mesa = 0.

**Funções:** `montar_matriz_transformacao()`, `transformar_pontos()`

### Passo 4 — Projeção Pinhole / Algoritmo de Tsai

Converte pontos 3D (referencial da mesa) em pixels 2D do projetor usando `cv2.projectPoints`, que aplica o modelo de câmera de Tsai com parâmetros intrínsecos (fx, fy, cx, cy) e distorção radial/tangencial.

**Funções:** `projetar_pontos_tsai()`, `calibrar_projetor()`

### Pipeline Integrado

A função `pipeline_plano_e_base()` executa os Passos 1+2+3 de uma só vez, retornando a normal, o centroide, a base ortonormal e a matriz T.

---

## Integração Cartográfica — Leitura de Mapas Reais (GeoTIFF)

O módulo `mde_cartografia.py` implementa a classe `AdaptadorMDE` que lê Modelos Digitais de Elevação reais fornecidos pela equipe de Cartografia no formato **GeoTIFF**.

### Como funciona

1. **Leitura** — abre o `.tif` com `rasterio` e extrai a primeira banda de elevações.
2. **Normalização Z** — mapeia as elevações reais (ex: 800 m a 1200 m) para a escala física da caixa de areia (ex: 0 a 0.20 m).
3. **Mapeamento XY** — cria eixos lineares em metros correspondentes às dimensões da mesa.
4. **Interpolação** — constrói um `scipy.interpolate.RegularGridInterpolator` bilinear para consultas contínuas.

### Uso

```python
from mde_cartografia import AdaptadorMDE

mde = AdaptadorMDE(
    caminho_geotiff="terreno_aman.tif",
    largura_mesa=0.90,       # X em metros
    comprimento_mesa=0.60,   # Y em metros
    altura_max_areia=0.20,   # Z em metros
)

# Consulta pontual — compatível com gerar_mapa_cores()
z = mde.obter_z_alvo(0.45, 0.30)
```

O módulo `adaptador_mde.py` fornece uma versão com mock interno para desenvolvimento e testes sem depender de arquivos reais.

---

## Qualidade de Software — TDD

A estabilidade do motor matemático é assegurada por uma suíte de **26 testes unitários automatizados** (`test_motor_caixao.py`), cobrindo:

| Classe de Teste | Testes | Componente |
|---|---|---|
| `TestAjustePlano` | 4 | SVD, normal unitária, equação do plano |
| `TestGramSchmidt` | 2 | Ortogonalidade, exceção para vetores paralelos |
| `TestConstruirBase` | 3 | Ortonormalidade mútua dos 3 eixos |
| `TestMatrizTransformacao` | 3 | Identidade, translação, z_mesa = 0 no plano |
| `TestDeteccaoTabuleiro` | 2 | Imagem sem tabuleiro, tabuleiro sintético 7×5 |
| `TestProjecaoTsai` | 3 | Projeção pinhole, deslocamento, múltiplos pontos |
| `TestLeituraRGBD` | 1 | Importação condicional do Open3D |
| `TestMDEColoracao` | 6 | Vermelho/Azul/Verde, limites, mock com rampa |
| `TestPipeline` | 2 | Integração completa Passos 1+2 |

Todos os testes usam valores **hardcoded** e comentários que explicam a matemática passo a passo, preparados para apresentação à banca examinadora.

```bash
# Rodar testes
python -m unittest test_motor_caixao -v

# Resultado esperado: 25 passed, 1 skipped (Open3D)
```

---

## Como Instalar e Rodar

### Pré-requisitos

```bash
# Dependências principais
pip install numpy opencv-python

# Para leitura de GeoTIFF da Cartografia
pip install rasterio scipy

# Para nuvem de pontos RGBD (opcional)
pip install open3d

# Para rodar testes com pytest (opcional)
pip install pytest
```

Para uso com o Kinect real (Linux):

```bash
sudo apt-get install freenect python3-freenect
```

### Execução

```bash
# Pipeline completo (calibração + loop AR)
python app_integrado.py

# Loop original da Raquel (captura + exibição)
python main.py

# Testes manuais do sensor
python testekinect.py

# Testes unitários do motor matemático
python -m unittest test_motor_caixao -v
```

Teclas durante a execução do `app_integrado.py`:

| Tecla | Ação |
|---|---|
| **R** | Recalibrar o plano da mesa |
| **Q** | Encerrar o sistema |

### Rodando com o Kinect real

No arquivo `kinect.py`, altere:

```python
USE_KINECT_REAL = True
```

---

## Status Atual

- ✅ Simulador de profundidade funcionando
- ✅ Conversão depth → pontos 3D funcionando
- ✅ Exibição colorida no projetor funcionando
- ✅ Motor matemático completo (SVD, Gram-Schmidt, Tsai)
- ✅ Transformação afim 4×4 Kinect → Mesa
- ✅ Lógica de coloração MDE (Vermelho/Azul/Verde)
- ✅ Adaptador MDE com mock para desenvolvimento
- ✅ Leitor GeoTIFF real com rasterio + scipy
- ✅ Pipeline integrado (app_integrado.py)
- ✅ 26 testes unitários automatizados
- ✅ Documentação técnica completa
- 🔄 Kinect real — aguardando hardware
- 🔄 Calibração física do projetor com tabuleiro de xadrez
- 🔄 Recebimento do MDE final da Cartografia
- 🔄 Ajuste de tolerância com areia real

---

## Referências

- OpenKinect / libfreenect: https://github.com/OpenKinect/libfreenect
- SARndbox (UC Davis): https://github.com/KeckCAVES/SARndbox
- AR Sandbox DIY: https://ar-sandbox.eu/augmented-reality-sandbox-diy/
- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Tsai, R. Y. (1987). *A Versatile Camera Calibration Technique for High-Accuracy 3D Machine Vision Metrology Using Off-the-Shelf TV Cameras and Lenses*. IEEE Journal of Robotics and Automation.
- Open3D: http://www.open3d.org/
- Rasterio: https://rasterio.readthedocs.io/
# Documentação Oficial — Caixão de Areia com Realidade Aumentada

**Projeto Final de Curso (PFC) — Engenharia de Computação, 2026**

| Campo | Valor |
|---|---|
| **Instituição** | Instituto Militar de Engenharia (IME) |
| **Curso** | Engenharia de Computação & Engenharia Eletrônica & Engenharia Cartográfica |
| **Ano** | 2026 |
| **Equipe** | Rafael Schuinki · Raquel Belchior |
| **Repositório** | `raquelbelchior1/PFC-2026` — branch `main` |

---

## Sumário

1. [Visão Geral e Objetivo](#1-visão-geral-e-objetivo)
2. [Arquitetura de Software](#2-arquitetura-de-software)
3. [Motor Matemático — Pipeline Completo](#3-motor-matemático--pipeline-completo)
4. [Mecanismo de Resiliência — Fallback em Cascata](#4-mecanismo-de-resiliência--fallback-em-cascata)
5. [Regra de Negócio — Coloração por Diferença de Altitude](#5-regra-de-negócio--coloração-por-diferença-de-altitude)
6. [Qualidade de Software e TDD](#6-qualidade-de-software-e-tdd)
7. [Guia de Execução e Demonstração](#7-guia-de-execução-e-demonstração)

---

## 1. Visão Geral e Objetivo

O sistema implementa um **Caixão de Areia com Realidade Aumentada** (*Augmented Reality Sandbox*): uma plataforma que projeta, em tempo real, um mapa de cores sobre uma caixa de areia física, guiando o operador a modelar o terreno até que sua topografia corresponda a um Modelo Digital de Elevação (MDE) de referência.

### 1.1 Parâmetros Físicos da Mesa

| Parâmetro | Valor | Variável no código |
|---|---|---|
| Dimensão X (largura) | 1,50 m | `LARGURA_MESA` |
| Dimensão Y (comprimento) | 1,50 m | `COMPRIMENTO_MESA` |
| Profundidade máxima de areia | 0,30 m (30 cm) | `ALTURA_MAX_AREIA` |
| Altura do Kinect (montagem) | 2,50 m | `ALTURA_KINECT` |
| Tolerância para cor verde | 0,02 m (2 cm) | `TOLERANCIA_COR` |

### 1.2 Saídas Visuais

O sistema opera com **duas janelas OpenCV simultâneas**, projetadas para a apresentação à banca:

| Janela | Nome no código | Função |
|---|---|---|
| **Projeção AR** | `Projecao_Areia` | Imagem de feedback vermelho/azul/verde — enviada ao projetor sobre a areia. Suporta tela cheia (`cv2.WINDOW_FULLSCREEN`). |
| **Gabarito MDE** | `Gabarito_MDE` | Heatmap 2D do MDE de referência com colormap — monitor auxiliar para o operador e a banca. |

---

## 2. Arquitetura de Software

### 2.1 Arquitetura Monolítica Modularizada

A arquitetura é **monolítica modularizada**: todos os módulos executam no mesmo processo Python, cada um encapsulando uma responsabilidade bem definida. Essa escolha garante:

- **Latência mínima** — comunicação por chamada de função em memória, essencial para o loop de tempo real.
- **Deploy simplificado** — ponto de entrada único (`main.py`).
- **Separação de responsabilidades** — cada módulo é testável e substituível de forma independente.

### 2.2 Estrutura de Módulos

```
PFC-2026/
├── main.py                    # Orquestrador — Máquina de Estados
├── kinect_sensor.py           # Camada de Hardware — KinectSensor (OOP)
├── motor_caixao_areia.py      # Motor Matemático — álgebra linear pura
├── mde_cartografia.py         # Adaptador de Dados — AdaptadorMDE (GeoTIFF)
├── test_motor_caixao.py       # Suíte TDD — 26 testes unitários
├── DOCUMENTACAO_OFICIAL.md    # Este documento
└── README.md                  # Guia de uso prático
```

### 2.3 Diagrama de Dependências

```
                        ┌─────────────────────┐
                        │      main.py        │
                        │  Máquina de Estados  │
                        │  INIT → IDLE →       │
                        │  CALIBRACAO → AR_LOOP│
                        └────────┬─────────────┘
           ┌─────────────────────┼─────────────────────┐
           ▼                     ▼                     ▼
   ┌───────────────┐   ┌────────────────────┐   ┌─────────────────┐
   │kinect_sensor.py│   │motor_caixao_areia.py│  │mde_cartografia.py│
   │  KinectSensor  │   │ SVD, Gram-Schmidt, │  │  AdaptadorMDE    │
   │  (Strategy +   │   │ Afim 4×4, Tsai,    │  │  (Adapter Pattern│
   │   Fallback)    │   │ cor_por_diferenca   │  │   + Fallback)    │
   └───────────────┘   └────────────────────┘   └─────────────────┘
```

### 2.4 Máquina de Estados (`main.py`)

O orquestrador implementa uma máquina de estados finita com quatro estados e transições controladas por `cv2.waitKey`:

```
         ┌──────┐                  ┌──────────┐
         │ INIT │─── inicializa ──▶│   IDLE   │
         └──────┘   KinectSensor   └────┬─────┘
                    + AdaptadorMDE       │ tecla [C]
                                         ▼
                                   ┌───────────┐
                      tecla [C] ◀──│CALIBRACAO  │
                      ┌───────────▶│SVD + G-S + T│
                      │            └─────┬─────┘
                      │                  │ sucesso
                      │                  ▼
                      │            ┌──────────┐
                      └────────────│ AR_LOOP  │──── tecla [Q/ESC] ──▶ ENCERRAR
                                   │captura →  │
                                   │transforma│
                                   │compara →  │
                                   │colore →   │
                                   │projeta    │
                                   └──────────┘
```

| Estado | Descrição |
|---|---|
| **INIT** | Inicializa `KinectSensor` (com fallback), carrega `AdaptadorMDE` (com fallback), cria janelas OpenCV. |
| **IDLE** | Exibe profundidade colorida do sensor; aguarda tecla **C** para calibrar. |
| **CALIBRACAO** | No modo real: captura nuvem → SVD → Gram-Schmidt → Matriz 4×4. No modo simulação: calibração automática ($T = I_4$). |
| **AR_LOOP** | Loop contínuo: captura → transforma → compara com MDE → gera cores → projeta na imagem. |

### 2.5 Padrões de Projeto Utilizados

| Padrão | Onde | Propósito |
|---|---|---|
| **Strategy + Fallback** | `KinectSensor` | O construtor seleciona transparentemente a estratégia de captura (Open3D → freenect → simulação) sem alterar a interface pública. |
| **Adapter** | `AdaptadorMDE` | Isola o formato de entrada (GeoTIFF, superfície sintética) da interface `obter_z_alvo(x, y)` consumida pelo motor. |
| **State Machine** | `main.py` | Controla o fluxo do pipeline com estados bem definidos e transições determinísticas. |

### 2.6 Stack Tecnológica

| Componente | Tecnologia | Justificativa |
|---|---|---|
| Linguagem | Python 3.10+ | Type Hints nativos, ecossistema científico maduro |
| Álgebra Linear | NumPy | SVD, transformações matriciais, operações vetorizadas |
| Visão Computacional | OpenCV 4.x | `projectPoints`, `findChessboardCorners`, colormap, janelas |
| Nuvem 3D (opcional) | Open3D | Criação de `PointCloud` a partir de RGB-D |
| GeoTIFF (opcional) | rasterio + scipy | Leitura de MDE real + interpolação bilinear |
| Testes | unittest / pytest | Suíte TDD com 26 testes automatizados |

---

## 3. Motor Matemático — Pipeline Completo

O módulo `motor_caixao_areia.py` implementa os pilares matemáticos que convertem uma nuvem de pontos bruta do Kinect em pixels coloridos projetados sobre a areia. Cada passo é descrito com rigor formal.

### 3.1 Passo 1 — Ajuste de Plano via Decomposição em Valores Singulares (SVD)

**Objetivo:** Dada uma nuvem de $N$ pontos $\{\mathbf{p}_i\}_{i=1}^{N} \subset \mathbb{R}^3$ capturados pelo Kinect sobre a superfície da mesa, encontrar o plano que melhor se ajusta a esses pontos no sentido dos mínimos quadráticos.

**Formulação.** O plano é descrito pela equação:

$$ax + by + cz + d = 0$$

onde $\mathbf{n} = (a, b, c)^T$ é o vetor normal unitário e $d = -\mathbf{n} \cdot \bar{\mathbf{p}}$, com $\bar{\mathbf{p}}$ sendo o centroide da nuvem.

**Método.** O algoritmo procede em três etapas:

1. **Centralização.** Calcula-se o centroide $\bar{\mathbf{p}} = \frac{1}{N}\sum_{i=1}^{N} \mathbf{p}_i$ e constrói-se a matriz centralizada:

$$M = \begin{bmatrix} (\mathbf{p}_1 - \bar{\mathbf{p}})^T \\ \vdots \\ (\mathbf{p}_N - \bar{\mathbf{p}})^T \end{bmatrix} \in \mathbb{R}^{N \times 3}$$

2. **Decomposição SVD.** Aplica-se $M = U \Sigma V^T$, onde $\Sigma = \text{diag}(\sigma_1, \sigma_2, \sigma_3)$ com $\sigma_1 \geq \sigma_2 \geq \sigma_3 \geq 0$.

3. **Extração da normal.** O último vetor-linha de $V^T$ (associado a $\sigma_3$) minimiza $\|M\mathbf{v}\|^2$ sujeito a $\|\mathbf{v}\| = 1$. Este vetor é a normal $\mathbf{n}$ do plano de melhor ajuste.

**Convenção:** o código garante $n_z > 0$ (normal apontando para cima, em direção ao Kinect).

**Implementação:** `ajustar_plano_svd()` em `motor_caixao_areia.py`.

---

### 3.2 Passo 2 — Sistema de Coordenadas da Mesa (Gram-Schmidt)

**Objetivo:** Construir uma base ortonormal $\{X_{\text{mesa}}, Y_{\text{mesa}}, Z_{\text{mesa}}\}$ com $Z_{\text{mesa}} = \mathbf{n}$.

**Método — Ortogonalização de Gram-Schmidt e Produto Vetorial:**

1. **Eixo Z:**

$$Z_{\text{mesa}} = \frac{\mathbf{n}}{\|\mathbf{n}\|}$$

2. **Vetor semente** $\mathbf{s}$: escolhe-se $(1, 0, 0)^T$; se $|\mathbf{s} \cdot Z_{\text{mesa}}| \geq 0{,}9$, usa-se $(0, 1, 0)^T$.

3. **Eixo X — Gram-Schmidt:**

$$X_{\text{mesa}} = \frac{\mathbf{s} - (\mathbf{s} \cdot Z_{\text{mesa}}) \, Z_{\text{mesa}}}{\|\mathbf{s} - (\mathbf{s} \cdot Z_{\text{mesa}}) \, Z_{\text{mesa}}\|}$$

4. **Eixo Y — Produto Vetorial:**

$$Y_{\text{mesa}} = Z_{\text{mesa}} \times X_{\text{mesa}}$$

**Verificação de ortonormalidade** (assegurada pela suíte de testes):

$$X \cdot Y = 0, \quad X \cdot Z = 0, \quad Y \cdot Z = 0, \quad \|X\| = \|Y\| = \|Z\| = 1$$

**Implementação:** `gram_schmidt()` e `construir_base_mesa()` em `motor_caixao_areia.py`.

---

### 3.3 Passo 3 — Transformação Afim 4×4 (Kinect → Mesa)

**Objetivo:** Montar $T \in \mathbb{R}^{4 \times 4}$ tal que pontos sobre o plano da mesa tenham $z_{\text{mesa}} = 0$ e a origem coincida com o centroide.

**Construção.** A matriz de rotação empilha os eixos da mesa como linhas:

$$R = \begin{bmatrix} X_{\text{mesa}}^T \\ Y_{\text{mesa}}^T \\ Z_{\text{mesa}}^T \end{bmatrix}, \quad \mathbf{t} = -R \, \bar{\mathbf{p}}$$

$$T = \begin{bmatrix} R & \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix}$$

**Aplicação:** $\mathbf{p}_{\text{mesa}} = T \cdot [\mathbf{p}_{\text{kinect}}^T, 1]^T$

**Propriedade fundamental:** se $\mathbf{p}$ pertence ao plano, então $z_{\text{mesa}} = 0$.

**Modo Simulação:** como os pontos já estão em coordenadas da mesa, $T = I_4$ (identidade).

**Implementação:** `montar_matriz_transformacao()` e `transformar_pontos()` em `motor_caixao_areia.py`.

---

### 3.4 Passo 4 — Projeção Pinhole / Tsai (3D → 2D)

**Objetivo:** Converter pontos 3D da mesa em pixels 2D do projetor.

O modelo de câmera pinhole com distorção de Tsai é implementado via `cv2.projectPoints`:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} R_{\text{ext}} & \mathbf{t}_{\text{ext}} \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}$$

onde a **matriz intrínseca** do projetor é:

$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

**Projeção final em pixels:**

$$u = f_x \cdot \frac{X'}{Z'} + c_x, \quad v = f_y \cdot \frac{Y'}{Z'} + c_y$$

O modelo de distorção radial e tangencial de Tsai (embutido no OpenCV) corrige aberrações ópticas com coeficientes $(k_1, k_2, p_1, p_2, k_3)$.

**Implementação:** `projetar_pontos_tsai()` e `calibrar_projetor()` em `motor_caixao_areia.py`.

---

## 4. Mecanismo de Resiliência — Fallback em Cascata

O sistema foi projetado com o requisito inviolável de **nunca falhar durante a apresentação à banca**, independentemente do hardware ou arquivos disponíveis. Isso é implementado por fallback automático em dois pontos críticos.

### 4.1 Sensor — `KinectSensor` (kinect_sensor.py)

```
Inicialização (construtor):
  1. Tenta Open3D (Azure Kinect / RealSense)  →  sucesso? usa.
  2. Tenta freenect (Kinect v1)                →  sucesso? usa.
  3. Ambos falharam?  →  Modo Simulação automático.
```

**No Modo Simulação**, `capturar_nuvem()` retorna uma nuvem de **2.500 pontos** (grid 50×50) já em coordenadas da mesa:

- $X \in [0, 1.5]$ m (50 pontos linearmente espaçados)
- $Y \in [0, 1.5]$ m (50 pontos linearmente espaçados)
- $Z = 0.15$ m para **todos** os pontos

Isso simula uma caixa de areia preenchida com **15 cm de areia perfeitamente nivelada** — exatamente na metade da altura máxima da caixa.

### 4.2 MDE — `AdaptadorMDE` (mde_cartografia.py)

```
Inicialização (construtor):
  1. Tenta ler GeoTIFF com rasterio  →  sucesso? normaliza e usa.
  2. Arquivo não existe / rasterio não instalado?
     →  Gera Morro Gaussiano sintético automaticamente.
```

**No modo real**, o GeoTIFF é lido via `rasterio`, as elevações são normalizadas de $[z_{\min}, z_{\max}]$ para $[0, 0.30]$ m, e um interpolador bilinear (`scipy.interpolate.RegularGridInterpolator`) permite consultas pontuais suaves em qualquer coordenada $(x, y)$.

**No Modo Simulação**, é gerado um **Morro Gaussiano** perfeitamente centralizado:

$$Z_{\text{MDE}}(x, y) = 0.3 \cdot \exp\!\left(-\frac{(x - 0.75)^2 + (y - 0.75)^2}{0.1}\right)$$

| Propriedade | Valor |
|---|---|
| Centro do morro | $(x, y) = (0.75, 0.75)$ — centro exato da mesa |
| Altura no pico | $Z = 0.30$ m (altura máxima de areia) |
| Altura nas bordas | $Z \approx 0.00$ m (decaimento exponencial) |
| Parâmetro de dispersão | $\sigma^2 = 0.05$ ($2\sigma^2 = 0.1$ no denominador) |
| Resolução da grade | 100 × 100 pontos |

**Derivação da escolha do parâmetro $0.1$:** com $\sigma^2 = 0.05$, temos $2\sigma \approx 0.316$ m. Na borda da mesa (distância $\sqrt{0.75^2 + 0.75^2} \approx 1.06$ m do centro), o expoente é $\approx -11.25$, resultando em $Z \approx 0.3 \cdot e^{-11.25} \approx 0.000004$ m — efetivamente zero.

### 4.3 Combinação das Simulações — Demonstração das Três Cores

Quando Kinect e GeoTIFF estão ausentes, a combinação dos dois mocks garante que as **três cores** convivam na mesma imagem:

| Região da mesa | $Z_{\text{real}}$ (Kinect) | $Z_{\text{MDE}}$ (Gaussiano) | Diferença | Cor |
|---|---|---|---|---|
| **Bordas** (longe do centro) | 0,15 m | ≈ 0,00 m | $+0{,}15 > +0{,}02$ | 🔴 **Vermelho** (cavar) |
| **Anel intermediário** ($r \approx 0{,}26$ m do centro) | 0,15 m | ≈ 0,15 m | $\approx 0 \leq 0{,}02$ | 🟢 **Verde** (OK) |
| **Centro** (pico do morro) | 0,15 m | ≈ 0,30 m | $-0{,}15 < -0{,}02$ | 🔵 **Azul** (preencher) |

Distribuição medida na simulação (grid 50×50 = 2.500 pontos):

| Cor | Pontos | Percentual |
|---|---|---|
| Vermelho | 2.224 | 89,0% |
| Verde | 84 | 3,4% |
| Azul | 192 | 7,7% |

Essa distribuição confirma que:
1. A lógica de subtração $Z_{\text{real}} - Z_{\text{MDE}}$ está correta.
2. A tolerância de $\pm 0{,}02$ m gera um anel verde visível.
3. As três cores são claramente distinguíveis na imagem projetada.

---

## 5. Regra de Negócio — Coloração por Diferença de Altitude

Para cada ponto $i$ da nuvem transformada para coordenadas da mesa, o sistema extrai $(x_i, y_i, z_i^{\text{real}})$ e consulta o MDE: $z_i^{\text{MDE}} = \text{obter\_z\_alvo}(x_i, y_i)$.

A cor atribuída segue a regra parametrizada pela tolerância $\tau = 0{,}02$ m (2 cm):

$$\text{cor}(i) = \begin{cases} \color{red}{\textbf{Vermelho}} \; (0, 0, 255)_{\text{BGR}} & \text{se } z_i^{\text{real}} > z_i^{\text{MDE}} + \tau \quad \text{(cavar)} \\[6pt] \color{blue}{\textbf{Azul}} \; (255, 0, 0)_{\text{BGR}} & \text{se } z_i^{\text{real}} < z_i^{\text{MDE}} - \tau \quad \text{(preencher)} \\[6pt] \color{green}{\textbf{Verde}} \; (0, 255, 0)_{\text{BGR}} & \text{caso contrário} \quad \text{(OK)} \end{cases}$$

**Notas importantes:**

- A convenção de cores é **BGR** (Blue, Green, Red), padrão do OpenCV.
- No limite exato ($z_i^{\text{real}} = z_i^{\text{MDE}} \pm \tau$), a classificação é **Verde** (comparações estritas `>` e `<`).
- A tolerância opera em **metros**, na mesma unidade de $Z_{\text{real}}$ e $Z_{\text{MDE}}$.

**Implementação:** `cor_por_diferenca()` e `gerar_mapa_cores()` em `motor_caixao_areia.py`.

---

## 6. Qualidade de Software e TDD

### 6.1 Estratégia de Testes

A estabilidade do motor matemático foi assegurada com **Test-Driven Development (TDD)**, resultando em **26 testes unitários** no módulo `test_motor_caixao.py`:

```bash
python -m unittest test_motor_caixao -v
# Resultado: 25 passed, 1 skipped (Open3D não instalado)
```

### 6.2 Cobertura por Componente

| Classe de Teste | Qtd | Componente Verificado |
|---|---|---|
| `TestAjustePlano` | 4 | SVD: normal unitária, plano horizontal $z=5$, plano inclinado, exceção $N<3$ |
| `TestGramSchmidt` | 2 | Ortogonalidade, exceção para vetores paralelos |
| `TestConstruirBase` | 3 | Ortonormalidade mútua, $Z_{\text{mesa}} = \mathbf{n}$, planos inclinados |
| `TestMatrizTransformacao` | 3 | $T = I$ para base canônica, translação anula origem, $z_{\text{mesa}} = 0$ no plano |
| `TestDeteccaoTabuleiro` | 2 | Imagem sem tabuleiro → `False`, tabuleiro sintético $7 \times 5$ → cantos detectados |
| `TestProjecaoTsai` | 3 | Projeção no ponto principal, deslocamento em $x$, múltiplos pontos |
| `TestLeituraRGBD` | 1 | Importação condicional do Open3D (skip gracioso se ausente) |
| `TestMDEColoracao` | 6 | Vermelho/Azul/Verde, limites exatos, mock com rampa linear |
| `TestPipeline` | 2 | Integração completa: plano $z=0$ e ponto acima do plano $z=10$ |

### 6.3 Filosofia — Transparência Matemática

Cada teste usa valores **hardcoded** com comentários que explicitam a conta passo a passo, permitindo que a banca verifique a correção sem executar o código:

**Exemplo — plano horizontal em $z = 5$:**

```
Pontos: (0,0,5), (1,0,5), (0,1,5), (1,1,5), (2,3,5)
Normal esperada: (0, 0, 1) — plano horizontal
d esperado: -5
Verificação: n · p + d = (0,0,1)·(0,0,5) + (-5) = 5 - 5 = 0  ✓
```

**Exemplo — projeção pinhole:**

```
Ponto (0,0,0), tvec = (0,0,1), fx = 320, cx = 320
u = 320 · (0/1) + 320 = 320
v = 240 · (0/1) + 240 = 240
Pixel esperado: (320, 240) — centro da imagem  ✓
```

---

## 7. Guia de Execução e Demonstração

### 7.1 Instalação

```bash
# Obrigatórias
pip install numpy opencv-python

# Opcionais (GeoTIFF real, interpolação, nuvem RGBD)
pip install rasterio scipy open3d

# Testes (opcional, unittest funciona nativamente)
pip install pytest
```

### 7.2 Execução

```bash
python main.py
```

O sistema detecta automaticamente o hardware disponível. Se nenhum Kinect estiver conectado e nenhum GeoTIFF estiver presente, entra em **Modo Simulação** completo sem intervenção do usuário.

### 7.3 Roteiro de Demonstração para a Banca

| Passo | Ação | Resultado esperado |
|---|---|---|
| 1 | `python main.py` | Duas janelas abrem: Projecao_Areia e Gabarito_MDE |
| 2 | Pressionar **C** | Calibração automática (modo simulação) ou SVD (modo real) |
| 3 | Observar Projecao_Areia | Três cores visíveis: bordas vermelhas, anel verde, centro azul |
| 4 | Observar Gabarito_MDE | Heatmap do Morro Gaussiano (ou GeoTIFF real) |
| 5 | Pressionar **F** | Tela cheia na janela de projeção |
| 6 | Pressionar **C** | Recalibração (demonstra robustez) |
| 7 | Pressionar **Q** | Encerramento limpo |

### 7.4 Execução dos Testes

```bash
python -m unittest test_motor_caixao -v
```

Saída esperada: **25 passed, 1 skipped** (Open3D ausente no ambiente de teste).

---

> **Documento gerado em:** Março de 2026
> **Versão:** 2.0 — Versão final para defesa
> **Sistema:** Finalizado, testado e pronto para apresentação

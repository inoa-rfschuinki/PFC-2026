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
4. [Renderização por Malha Discretizada](#4-renderização-por-malha-discretizada)
5. [Emulador Interativo de Areia](#5-emulador-interativo-de-areia)
6. [Mecanismo de Resiliência — Fallback em Cascata](#6-mecanismo-de-resiliência--fallback-em-cascata)
7. [Regra de Negócio — Coloração por Diferença de Altitude](#7-regra-de-negócio--coloração-por-diferença-de-altitude)
8. [Qualidade de Software e TDD](#8-qualidade-de-software-e-tdd)
9. [Guia de Execução e Demonstração](#9-guia-de-execução-e-demonstração)

---

## 1. Visão Geral e Objetivo

O sistema implementa um **Caixão de Areia com Realidade Aumentada** (*Augmented Reality Sandbox*): uma plataforma que projeta, em tempo real, uma **grade contínua de quadrados coloridos** sobre uma caixa de areia física, guiando o operador a modelar o terreno até que sua topografia corresponda a um Modelo Digital de Elevação (MDE) de referência.

O pipeline de renderização opera por **discretização em malha**: a mesa é dividida em células regulares, e cada célula é classificada e desenhada como um polígono preenchido, eliminando a projeção ruidosa de pontos isolados e garantindo cobertura visual contínua.

Na ausência de hardware físico, o sistema oferece um **emulador interativo** que permite ao operador cavar e preencher a areia virtual com o mouse, demonstrando todo o fluxo matemático em tempo real.

### 1.1 Parâmetros Físicos da Mesa

| Parâmetro | Valor | Variável no código |
|---|---|---|
| Dimensão X (largura) | 1,50 m | `LARGURA_MESA` |
| Dimensão Y (comprimento) | 1,50 m | `COMPRIMENTO_MESA` |
| Profundidade máxima de areia | 0,30 m (30 cm) | `ALTURA_MAX_AREIA` |
| Altura do Kinect (montagem) | 2,50 m | `ALTURA_KINECT` |
| Tolerância para cor verde | 0,02 m (2 cm) | `TOLERANCIA_COR` |
| Células da grade (eixo X) | 30 | `CELULAS_GRADE_X` |
| Células da grade (eixo Y) | 30 | `CELULAS_GRADE_Y` |
| Tamanho de cada célula | 5 cm × 5 cm | derivado |
| Raio da pá virtual (mouse) | 0,10 m (10 cm) | `RAIO_PA_VIRTUAL` |
| Intensidade da pá virtual | 0,008 m (8 mm/evento) | `INTENSIDADE_PA_VIRTUAL` |

### 1.2 Saídas Visuais

O sistema opera com **duas janelas OpenCV simultâneas**, projetadas para a apresentação à banca:

| Janela | Nome no código | Função |
|---|---|---|
| **Projeção AR** | `Projecao_Areia` | Grade contínua de quadrados coloridos (vermelho/azul/verde) — enviada ao projetor sobre a areia. Suporta tela cheia (`cv2.WINDOW_FULLSCREEN`). Aceita interação via mouse (modo simulação). |
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
├── main.py                    # Orquestrador — Máquina de Estados + Mouse Callback
├── kinect_sensor.py           # Camada de Hardware — KinectSensor (OOP + Grade Persistente)
├── motor_caixao_areia.py      # Motor Matemático — álgebra linear + discretização em grade
├── mde_cartografia.py         # Adaptador de Dados — AdaptadorMDE (GeoTIFF)
├── test_motor_caixao.py       # Suíte TDD — 26 testes unitários
├── DOCUMENTACAO_OFICIAL.md    # Este documento
└── README.md                  # Guia de uso prático
```

### 2.3 Diagrama de Dependências

```
                      ┌──────────────────────────┐
                      │        main.py            │
                      │   Máquina de Estados      │
                      │   INIT → IDLE →           │
                      │   CALIBRACAO → AR_LOOP    │
                      │        +                  │
                      │   cv2.setMouseCallback    │
                      │   (Pá Virtual)            │
                      └──────────┬────────────────┘
         ┌───────────────────────┼───────────────────────┐
         ▼                       ▼                       ▼
┌─────────────────┐   ┌──────────────────────┐   ┌──────────────────┐
│kinect_sensor.py  │   │motor_caixao_areia.py  │  │mde_cartografia.py │
│  KinectSensor    │   │ SVD, Gram-Schmidt,    │  │  AdaptadorMDE     │
│  (Strategy +     │   │ Afim 4×4, Tsai,       │  │  (Adapter Pattern │
│   Fallback +     │   │ discretizar_nuvem_    │  │   + Fallback)     │
│   Grade Persist.)│   │ em_grade,             │  │                   │
│  modificar_areia │   │ gerar_imagem_grade_   │  │                   │
│  ()              │   │ cores (fillPoly)      │  │                   │
└─────────────────┘   └──────────────────────┘   └──────────────────┘
```

### 2.4 Máquina de Estados (`main.py`)

O orquestrador implementa uma máquina de estados finita com quatro estados e transições controladas por `cv2.waitKey`. O callback de mouse opera de forma assíncrona dentro do `AR_LOOP`, modificando o estado da areia entre frames:

```
         ┌──────┐                  ┌──────────┐
         │ INIT │─── inicializa ──▶│   IDLE   │
         └──────┘   KinectSensor   └────┬─────┘
                    + AdaptadorMDE       │ tecla [C]
                    + MouseCallback      ▼
                                   ┌───────────┐
                      tecla [C] ◀──│CALIBRACAO  │
                      ┌───────────▶│SVD + G-S + T│
                      │            └─────┬─────┘
                      │                  │ sucesso
                      │                  ▼
                      │            ┌──────────────────────┐
                      └────────────│      AR_LOOP         │──── tecla [Q/ESC] ──▶ ENCERRAR
                                   │ captura nuvem        │
                                   │ transforma (T)       │
                                   │ discretiza em grade  │
                                   │ compara MDE por célula│
                                   │ projeta vértices (Tsai)│
                                   │ fillPoly por célula  │
                                   │       ↑              │
                                   │ mouse callback       │
                                   │ modifica _grade_areia│
                                   └──────────────────────┘
```

| Estado | Descrição |
|---|---|
| **INIT** | Inicializa `KinectSensor` (com fallback e grade persistente), carrega `AdaptadorMDE` (com fallback), cria janelas OpenCV, registra `cv2.setMouseCallback`. |
| **IDLE** | Exibe profundidade colorida do sensor; aguarda tecla **C** para calibrar. |
| **CALIBRACAO** | No modo real: captura nuvem → SVD → Gram-Schmidt → Matriz 4×4. No modo simulação: calibração automática ($T = I_4$). |
| **AR_LOOP** | Loop contínuo: captura → transforma → discretiza em grade → compara MDE por célula → projeta vértices → `fillPoly`. Mouse modifica a areia entre frames. |

### 2.5 Padrões de Projeto Utilizados

| Padrão | Onde | Propósito |
|---|---|---|
| **Strategy + Fallback** | `KinectSensor` | O construtor seleciona transparentemente a estratégia de captura (Open3D → freenect → simulação interativa) sem alterar a interface pública. |
| **Adapter** | `AdaptadorMDE` | Isola o formato de entrada (GeoTIFF, superfície sintética) da interface `obter_z_alvo(x, y)` consumida pelo motor. |
| **State Machine** | `main.py` | Controla o fluxo do pipeline com estados bem definidos e transições determinísticas. |
| **Observer (Callback)** | `main.py` | O callback de mouse (`cv2.setMouseCallback`) observa eventos de interação e modifica o estado da areia de forma desacoplada do loop principal. |

### 2.6 Stack Tecnológica

| Componente | Tecnologia | Justificativa |
|---|---|---|
| Linguagem | Python 3.10+ | Type Hints nativos, ecossistema científico maduro |
| Álgebra Linear | NumPy | SVD, transformações matriciais, operações vetorizadas |
| Visão Computacional | OpenCV 4.x | `projectPoints`, `fillPoly`, `setMouseCallback`, colormap, janelas |
| Nuvem 3D (opcional) | Open3D | Criação de `PointCloud` a partir de RGB-D |
| GeoTIFF (opcional) | rasterio + scipy | Leitura de MDE real + interpolação bilinear |
| Testes | unittest / pytest | Suíte TDD com 26 testes automatizados |

---

## 3. Motor Matemático — Pipeline Completo

O módulo `motor_caixao_areia.py` implementa os pilares matemáticos que convertem uma nuvem de pontos bruta do Kinect em uma grade contínua de quadrados coloridos projetados sobre a areia. Cada passo é descrito com rigor formal.

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

## 4. Renderização por Malha Discretizada

O pipeline de renderização substitui a abordagem de projeção de pontos isolados por uma **discretização em malha regular** que garante cobertura visual contínua, filtragem de ruído do sensor e projeção geométrica eficiente.

### 4.1 Definição da Grade

O plano da mesa $[0, L_x] \times [0, L_y]$ (onde $L_x = L_y = 1{,}50$ m) é particionado em uma grade uniforme de $N_x \times N_y$ células retangulares (por padrão $N_x = N_y = 30$).

Cada célula $C_{i,j}$ (com $i \in \{0, \ldots, N_y-1\}$ e $j \in \{0, \ldots, N_x-1\}$) ocupa o domínio:

$$C_{i,j} = \left[ j \cdot \Delta x,\; (j+1) \cdot \Delta x \right] \times \left[ i \cdot \Delta y,\; (i+1) \cdot \Delta y \right]$$

onde os tamanhos de célula são:

$$\Delta x = \frac{L_x}{N_x} = \frac{1{,}50}{30} = 0{,}05 \text{ m}, \quad \Delta y = \frac{L_y}{N_y} = \frac{1{,}50}{30} = 0{,}05 \text{ m}$$

A grade possui $(N_y + 1) \times (N_x + 1) = 31 \times 31 = 961$ vértices.

### 4.2 Agregação Espacial — Média de Altura por Célula

Para cada célula $C_{i,j}$, define-se o conjunto de pontos da nuvem do Kinect que pertencem a ela:

$$\mathcal{P}_{i,j} = \left\{ \mathbf{p}_k = (x_k, y_k, z_k) \in \text{nuvem} \;\middle|\; (x_k, y_k) \in C_{i,j} \right\}$$

A altura representativa da célula é a **média aritmética** dos valores $Z$ dos pontos capturados:

$$\bar{Z}_{i,j}^{\text{real}} = \frac{1}{|\mathcal{P}_{i,j}|} \sum_{\mathbf{p}_k \in \mathcal{P}_{i,j}} z_k$$

**Justificativa para a média.** O sensor Kinect apresenta ruído Gaussiano com desvio padrão da ordem de $\sigma \approx 3$ mm na faixa de operação (2–3 m). Ao agregar $n$ pontos por célula, a média reduz o desvio padrão do estimador para:

$$\sigma_{\bar{Z}} = \frac{\sigma}{\sqrt{n}}$$

Com o grid de simulação de 50×50 pontos e 30×30 células, cada célula contém em média $\lfloor 50/30 \rfloor^2 \approx 2\text{–}3$ pontos, proporcionando filtragem significativa. Em modo real com profundidade 640×480, a densidade aumenta para dezenas de pontos por célula.

**Implementação vetorizada.** O binning é realizado via `np.add.at` para acumulação eficiente sem loops Python:

```python
col = np.clip((x / tam_celula_x).astype(np.int32), 0, n_celulas_x - 1)
lin = np.clip((y / tam_celula_y).astype(np.int32), 0, n_celulas_y - 1)
np.add.at(soma_z, (lin, col), z)
np.add.at(contagens, (lin, col), 1)
```

Células sem pontos ($|\mathcal{P}_{i,j}| = 0$) são marcadas como `NaN` e não são renderizadas.

**Implementação:** `discretizar_nuvem_em_grade()` em `motor_caixao_areia.py`.

### 4.3 Classificação de Cor por Célula

O centro geométrico de cada célula $C_{i,j}$ é:

$$\mathbf{c}_{i,j} = \left( \left(j + \tfrac{1}{2}\right) \Delta x,\; \left(i + \tfrac{1}{2}\right) \Delta y \right)$$

A altura alvo do MDE é consultada nesse centro: $Z_{i,j}^{\text{MDE}} = \text{obter\_z\_alvo}(\mathbf{c}_{i,j})$.

A classificação de cor segue a regra definida na Seção 7, comparando $\bar{Z}_{i,j}^{\text{real}}$ com $Z_{i,j}^{\text{MDE}}$.

### 4.4 Projeção em Lote dos Vértices e Rasterização

A grade de $(N_y+1) \times (N_x+1)$ vértices é projetada do referencial 3D da mesa para o plano 2D do projetor em uma **única chamada** a `cv2.projectPoints`:

$$\text{vertices\_2d} = \text{Tsai}\!\left(\left\{ \left(\tfrac{j \cdot L_x}{N_x},\; \tfrac{i \cdot L_y}{N_y},\; 0\right) \right\}_{i=0,\ldots,N_y}^{j=0,\ldots,N_x}\right)$$

Os vértices são projetados com $Z = 0$ (plano da mesa), e a projeção em lote evita $N_x \times N_y$ chamadas individuais.

Cada célula $C_{i,j}$ é então desenhada como um quadrilátero preenchido usando os **quatro vértices projetados** adjacentes:

$$\text{quad}_{i,j} = \left[ V_{i,j},\; V_{i,j+1},\; V_{i+1,j+1},\; V_{i+1,j} \right]$$

A rasterização é feita via `cv2.fillPoly(imagem, [quad], cor)`, garantindo:
- **Cobertura contínua** — sem buracos entre células adjacentes.
- **Projeção geometricamente correta** — os cantos dos quadrados respeitam a perspectiva do projetor.
- **Eficiência** — a projeção de todos os vértices é realizada em $O(1)$ chamadas ao OpenCV.

**Implementação:** `gerar_imagem_grade_cores()` em `motor_caixao_areia.py`.

---

## 5. Emulador Interativo de Areia

Na ausência de hardware físico (sensor Kinect e caixa de areia), o sistema oferece um emulador interativo que permite ao operador modificar a topografia virtual em tempo real, demonstrando o pipeline completo de forma tangível para a banca avaliadora.

### 5.1 Estado Persistente da Areia

A classe `KinectSensor`, quando em modo simulação, inicializa e mantém uma **matriz de alturas persistente** $\mathbf{H} \in \mathbb{R}^{R \times R}$ (onde $R = 50$ é a resolução da grade de simulação):

$$H_{i,j}^{(0)} = \frac{h_{\max}}{2} = 0{,}15 \text{ m} \quad \forall\; i, j$$

onde $h_{\max} = 0{,}30$ m é a altura máxima de areia. Essa inicialização simula uma caixa preenchida com areia nivelada exatamente na metade da profundidade.

Os eixos da grade mapeiam linearmente para as coordenadas físicas da mesa:

$$x_j = \frac{j \cdot L_x}{R - 1}, \quad y_i = \frac{i \cdot L_y}{R - 1}, \quad i,j \in \{0, \ldots, R-1\}$$

A cada frame do loop principal, `capturar_nuvem()` retorna a nuvem de pontos $(x_j, y_i, H_{i,j})$, refletindo o estado atual (potencialmente modificado) da areia.

### 5.2 Modelo Matemático da Escavação/Preenchimento

Ao receber um evento de mouse na posição física $\mathbf{m} = (x_m, y_m)$ na mesa, o sistema aplica a seguinte atualização à matriz de alturas:

$$H_{i,j}^{(t+1)} = \text{clip}\!\left( H_{i,j}^{(t)} \pm \alpha \cdot \exp\!\left( -\frac{(x_j - x_m)^2 + (y_i - y_m)^2}{2\sigma^2} \right),\; 0,\; h_{\max} \right)$$

onde:

| Símbolo | Significado | Valor padrão |
|---|---|---|
| $\alpha$ | Intensidade do deslocamento por evento (metros) | $0{,}008$ m (8 mm) |
| $\sigma$ | Desvio padrão do perfil Gaussiano | $r / 2 = 0{,}05$ m |
| $r$ | Raio de ação da pá virtual | $0{,}10$ m (10 cm) |
| $+$ | Operador ao preencher (botão direito) | — |
| $-$ | Operador ao cavar (botão esquerdo) | — |
| $\text{clip}(v, a, b)$ | $\max(a, \min(v, b))$ | $[0, 0{,}30]$ m |

**Justificativa do perfil Gaussiano.** A distribuição Gaussiana 2D produz uma deformação suave e natural, isenta dos artefatos visuais de escavações retangulares ou cônicas. Com $\sigma = r/2$, aproximadamente 95% da energia do operador concentra-se dentro do raio $r$:

$$\int_0^{r} 2\pi \rho \cdot e^{-\rho^2 / 2\sigma^2}\, d\rho \;\Big/\; \int_0^{\infty} 2\pi \rho \cdot e^{-\rho^2 / 2\sigma^2}\, d\rho = 1 - e^{-r^2 / 2\sigma^2} = 1 - e^{-2} \approx 0{,}865$$

O efeito é **acumulativo**: manter o mouse sobre um ponto por $n$ frames desloca a altura em até $n \cdot \alpha$, convergindo monotonicamente para o limite do `clip`.

### 5.3 Mapeamento Pixel → Coordenada Física

O callback registrado via `cv2.setMouseCallback` converte a posição do cursor $(u, v)$ em pixels para coordenadas físicas $(x, y)$ da mesa:

$$x_{\text{mesa}} = \frac{u}{W_{\text{janela}}} \cdot L_x, \quad y_{\text{mesa}} = \frac{v}{H_{\text{janela}}} \cdot L_y$$

onde $W_{\text{janela}}$ e $H_{\text{janela}}$ são as dimensões atuais da janela (obtidas via `cv2.getWindowImageRect`), garantindo funcionamento correto mesmo com redimensionamento.

**Tratamento de erros:**
- Cliques fora da área da imagem ($u < 0$, $v < 0$, $u \geq W$, $v \geq H$) são ignorados.
- Dimensões de janela inválidas ($\leq 0$) são tratadas com fallback para a resolução nominal do projetor.
- O callback opera apenas em modo simulação; em modo real (Kinect conectado), eventos de mouse são silenciosamente ignorados.

### 5.4 Sincronia com o Loop de Renderização

O callback do mouse executa de forma **assíncrona** entre frames (invocado pelo `cv2.waitKey`). A modificação em `_grade_areia` é imediatamente visível no frame seguinte, pois `capturar_nuvem()` lê diretamente da mesma referência em memória. Não há necessidade de mecanismos de sincronização (mutex), pois o Python GIL garante atomicidade das atribuições NumPy em thread único.

O fluxo temporal é:

```
                    ┌─ frame N ────────────────────────────────┐
                    │ capturar_nuvem() → lê _grade_areia       │
                    │ discretizar → comparar MDE → fillPoly    │
                    │ cv2.imshow() → cv2.waitKey(1)            │
                    │             ↓                            │
                    │   mouse callback modifica _grade_areia   │
                    └──────────────────────────────────────────┘
                    ┌─ frame N+1 ──────────────────────────────┐
                    │ capturar_nuvem() → lê _grade_areia ←(atualizada)
                    │ ...                                      │
```

**Implementação:** `_callback_mouse()` em `main.py` e `modificar_areia()` em `kinect_sensor.py`.

---

## 6. Mecanismo de Resiliência — Fallback em Cascata

O sistema foi projetado com o requisito inviolável de **nunca falhar durante a apresentação à banca**, independentemente do hardware ou arquivos disponíveis. Isso é implementado por fallback automático em dois pontos críticos.

### 6.1 Sensor — `KinectSensor` (kinect_sensor.py)

```
Inicialização (construtor):
  1. Tenta Open3D (Azure Kinect / RealSense)  →  sucesso? usa.
  2. Tenta freenect (Kinect v1)                →  sucesso? usa.
  3. Ambos falharam?  →  Modo Simulação Interativo automático.
```

**No Modo Simulação Interativo**, o sensor mantém uma grade persistente de alturas $\mathbf{H} \in \mathbb{R}^{50 \times 50}$, inicializada em $Z = 0{,}15$ m. A cada chamada de `capturar_nuvem()`, a nuvem é reconstruída a partir do estado atual da grade (incluindo modificações feitas pelo mouse). O método `modificar_areia(x, y, cavar)` permite alterar as alturas em tempo real com perfil Gaussiano.

### 6.2 MDE — `AdaptadorMDE` (mde_cartografia.py)

```
Inicialização (construtor):
  1. Resolve caminho via pathlib (relativo ao diretório do script).
  2. Tenta ler GeoTIFF com rasterio  →  sucesso? normaliza e usa.
  3. Arquivo não existe / rasterio não instalado / erro de leitura?
     →  Log de erro detalhado + gera Morro Gaussiano sintético.
```

**No modo real**, o GeoTIFF é lido via `rasterio`, convertido para `float32`, valores nodata substituídos pelo mínimo válido, e as elevações são normalizadas de $[z_{\min}, z_{\max}]$ para $[0, 0.30]$ m. Um interpolador bilinear (`scipy.interpolate.RegularGridInterpolator`) permite consultas pontuais suaves em qualquer coordenada $(x, y)$.

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

### 6.3 Combinação das Simulações — Demonstração Interativa

Quando Kinect e GeoTIFF estão ausentes, a combinação dos dois mocks define o estado inicial da visualização:

| Região da mesa | $\bar{Z}^{\text{real}}$ (Areia) | $Z^{\text{MDE}}$ (Gaussiano) | Diferença | Cor |
|---|---|---|---|---|
| **Bordas** (longe do centro) | 0,15 m | ≈ 0,00 m | $+0{,}15 > +0{,}02$ | 🔴 **Vermelho** (cavar) |
| **Anel intermediário** ($r \approx 0{,}26$ m do centro) | 0,15 m | ≈ 0,15 m | $\approx 0 \leq 0{,}02$ | 🟢 **Verde** (OK) |
| **Centro** (pico do morro) | 0,15 m | ≈ 0,30 m | $-0{,}15 < -0{,}02$ | 🔵 **Azul** (preencher) |

Com o emulador interativo, o operador pode:
1. **Arrastar botão direito** no centro azul → areia sobe → quadrados se tornam verdes.
2. **Arrastar botão esquerdo** nas bordas vermelhas → areia desce → quadrados se tornam verdes.
3. **Objetivo**: tornar toda a grade verde, replicando o MDE com a areia virtual.

As cores reagem **instantaneamente** a cada evento de mouse, demonstrando em tempo real o pipeline completo: modificação da grade → captura da nuvem → discretização → comparação MDE → projeção Tsai → renderização `fillPoly`.

---

## 7. Regra de Negócio — Coloração por Diferença de Altitude

Para cada célula $C_{i,j}$ da grade com dados do sensor, o sistema calcula a altura média $\bar{Z}_{i,j}^{\text{real}}$ e consulta o alvo $Z_{i,j}^{\text{MDE}} = \text{obter\_z\_alvo}(\mathbf{c}_{i,j})$ no centro geométrico da célula.

A cor atribuída segue a regra parametrizada pela tolerância $\tau = 0{,}02$ m (2 cm):

$$\text{cor}(i,j) = \begin{cases} \color{red}{\textbf{Vermelho}} \; (0, 0, 255)_{\text{BGR}} & \text{se } \bar{Z}_{i,j}^{\text{real}} > Z_{i,j}^{\text{MDE}} + \tau \quad \text{(cavar)} \\[6pt] \color{blue}{\textbf{Azul}} \; (255, 0, 0)_{\text{BGR}} & \text{se } \bar{Z}_{i,j}^{\text{real}} < Z_{i,j}^{\text{MDE}} - \tau \quad \text{(preencher)} \\[6pt] \color{green}{\textbf{Verde}} \; (0, 255, 0)_{\text{BGR}} & \text{caso contrário} \quad \text{(OK)} \end{cases}$$

**Notas importantes:**

- A convenção de cores é **BGR** (Blue, Green, Red), padrão do OpenCV.
- No limite exato ($\bar{Z}_{i,j}^{\text{real}} = Z_{i,j}^{\text{MDE}} \pm \tau$), a classificação é **Verde** (comparações estritas `>` e `<`).
- A tolerância opera em **metros**, na mesma unidade de $\bar{Z}^{\text{real}}$ e $Z^{\text{MDE}}$.
- A classificação opera **por célula**, não por ponto individual, resultando em quadrados de cor uniforme.

**Implementação:** `cor_por_diferenca()` em `motor_caixao_areia.py`, invocada dentro de `gerar_imagem_grade_cores()`.

---

## 8. Qualidade de Software e TDD

### 8.1 Estratégia de Testes

A estabilidade do motor matemático foi assegurada com **Test-Driven Development (TDD)**, resultando em **26 testes unitários** no módulo `test_motor_caixao.py`:

```bash
python -m unittest test_motor_caixao -v
# Resultado: 25 passed, 1 skipped (Open3D não instalado)
```

### 8.2 Cobertura por Componente

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

### 8.3 Filosofia — Transparência Matemática

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

## 9. Guia de Execução e Demonstração

### 9.1 Instalação

```bash
# Obrigatórias
pip install numpy opencv-python

# Opcionais (GeoTIFF real, interpolação, nuvem RGBD)
pip install rasterio scipy open3d

# Testes (opcional, unittest funciona nativamente)
pip install pytest
```

### 9.2 Execução

```bash
python main.py
```

O sistema detecta automaticamente o hardware disponível. Se nenhum Kinect estiver conectado e nenhum GeoTIFF estiver presente, entra em **Modo Simulação Interativo** completo sem intervenção do usuário.

### 9.3 Controles

**Teclado:**

| Tecla | Ação |
|---|---|
| **C** | Calibrar (SVD + Gram-Schmidt + Matriz 4×4) |
| **F** | Toggle tela cheia na janela Projecao_Areia |
| **Q** / **ESC** | Encerrar |

**Mouse (Modo Simulação Interativo):**

| Ação | Efeito | Análogo físico |
|---|---|---|
| **Botão Esquerdo + Arrastar** | Diminui $Z_{real}$ (cava) | Pá escavando |
| **Botão Direito + Arrastar** | Aumenta $Z_{real}$ (preenche) | Balde despejando |

### 9.4 Roteiro de Demonstração para a Banca

| Passo | Ação | Resultado esperado |
|---|---|---|
| 1 | `python main.py` | Duas janelas abrem: **Projecao_Areia** e **Gabarito_MDE** |
| 2 | Pressionar **C** | Calibração automática ($T = I_4$, projeção configurada) |
| 3 | Observar **Projecao_Areia** | Grade contínua de quadrados: bordas vermelhas, anel verde, centro azul |
| 4 | Observar **Gabarito_MDE** | Heatmap do Morro Gaussiano de referência |
| 5 | **Arrastar botão direito** no centro (azul) | Quadrados mudam de azul → verde (areia subindo) |
| 6 | **Arrastar botão esquerdo** nas bordas (vermelho) | Quadrados mudam de vermelho → verde (areia descendo) |
| 7 | Continuar interagindo | Objetivo: toda a grade verde — terreno replica o MDE |
| 8 | Pressionar **F** | Tela cheia (projetor real) |
| 9 | Pressionar **C** | Recalibração (demonstra robustez) |
| 10 | Pressionar **Q** | Encerramento limpo |

### 9.5 Execução dos Testes

```bash
python -m unittest test_motor_caixao -v
```

Saída esperada: **25 passed, 1 skipped** (Open3D ausente no ambiente de teste).

---

> **Documento gerado em:** Março de 2026
> **Versão:** 3.0 — Versão final com Malha Discretizada e Emulador Interativo
> **Sistema:** Finalizado, testado e pronto para defesa

# Documentação Técnica — Caixão de Areia com Realidade Aumentada

**Projeto Final de Curso (PFC) — Engenharia de Computação, 2026**

| Campo | Valor |
|---|---|
| **Instituição** | Engenharia de Computação |
| **Ano** | 2026 |
| **Equipe** | Rafael Schuinki (Motor Matemático, Integração) · Raquel Belchior (Captura, Infraestrutura) |
| **Orientador** | Conforme definição da banca |
| **Repositório** | `raquelbelchior1/PFC-2026` — branch `main` |

---

## Sumário

1. [Visão Geral e Arquitetura do Sistema](#1-visão-geral-e-arquitetura-do-sistema)
2. [Motor Matemático — O Cérebro do Sistema](#2-motor-matemático--o-cérebro-do-sistema)
3. [Integração de Hardware e Sensores](#3-integração-de-hardware-e-sensores)
4. [Lógica de Negócio e Padrão de Projeto (Adapter)](#4-lógica-de-negócio-e-padrão-de-projeto-adapter)
5. [Qualidade de Software e TDD](#5-qualidade-de-software-e-tdd)
6. [Status Atual e Próximos Passos](#6-status-atual-e-próximos-passos)

---

## 1. Visão Geral e Arquitetura do Sistema

### 1.1 Objetivo do Projeto

O projeto implementa um **Caixão de Areia com Realidade Aumentada** (*Augmented Reality Sandbox*): um sistema que projeta, em tempo real, um mapa de cores sobre uma caixa de areia física, guiando o usuário a escavar ou preencher o terreno até que sua topografia corresponda a um Modelo Digital de Elevação (MDE) de referência fornecido pela equipe de Cartografia.

O pipeline opera continuamente:

1. Um sensor **Kinect** (RGB-D), posicionado acima da mesa, captura a superfície da areia como uma nuvem de pontos 3D.
2. O **motor matemático** ajusta o plano da mesa, transforma os pontos para um referencial local e compara cada ponto com o MDE de referência.
3. Um **projetor**, também posicionado acima da mesa, exibe cores diretamente sobre a areia:
   - **Vermelho** — há areia em excesso; é necessário **cavar**.
   - **Azul** — há areia insuficiente; é necessário **preencher**.
   - **Verde** — o relevo está dentro da tolerância especificada.

O resultado é um *feedback* visual imediato e intuitivo, transformando a modelagem de terrenos em uma experiência interativa de realidade aumentada.

### 1.2 Arquitetura Monolítica Modularizada

A arquitetura adotada é **monolítica modularizada**: todos os módulos executam no mesmo processo Python, mas cada um encapsula uma responsabilidade bem definida, comunicando-se exclusivamente por interfaces públicas (funções e classes). Essa escolha é deliberada:

- **Latência mínima** — a comunicação entre módulos é por chamada de função em memória, essencial para o loop de tempo real (~30 FPS).
- **Deploys simplificado** — um único ponto de entrada (`app_integrado.py`) orquestra todo o pipeline.
- **Separação de responsabilidades** — cada módulo pode ser testado, desenvolvido e mantido de forma independente por membros diferentes da equipe.

A estrutura de arquivos do projeto reflete essa separação:

```
PFC-2026/
├── kinect.py                  ← Captura de profundidade (Raquel)
├── exibicao.py                ← Exibição no projetor (Raquel)
├── utils.py                   ← Utilitários gerais (Raquel)
├── main.py                    ← Loop original da Raquel (intacto)
├── motor_caixao_areia.py      ← Motor matemático (Rafael)
├── adaptador_mde.py           ← Interface com Cartografia (Rafael)
├── app_integrado.py           ← Orquestrador do pipeline completo (Rafael)
├── test_motor_caixao.py       ← Suíte de testes unitários (Rafael)
└── testekinect.py             ← Testes manuais do sensor (Raquel)
```

O diagrama de dependências entre módulos é o seguinte:

```
                        ┌──────────────────┐
                        │  app_integrado.py │
                        │  (Orquestrador)   │
                        └────────┬─────────┘
           ┌─────────────────────┼─────────────────────┐
           ▼                     ▼                     ▼
   ┌───────────────┐   ┌────────────────────┐   ┌─────────────────┐
   │  kinect.py    │   │motor_caixao_areia.py│   │adaptador_mde.py │
   │  exibicao.py  │   │ (Motor Matemático)  │   │ (Padrão Adapter)│
   │  utils.py     │   └────────────────────┘   └─────────────────┘
   │  (Raquel)     │
   └───────────────┘
```

> **Princípio de projeto:** os arquivos da colega Raquel (`kinect.py`, `exibicao.py`, `utils.py`, `main.py`) jamais são alterados. O orquestrador (`app_integrado.py`) importa suas funções públicas e as compõe com o motor matemático.

### 1.3 Stack Tecnológica

| Componente | Tecnologia | Justificativa |
|---|---|---|
| Linguagem | **Python 3.10+** | Type Hints nativos (`tuple[int, int]`), ecossistema científico maduro |
| Álgebra Linear | **NumPy** | SVD, transformações matriciais, operações vetorizadas em nuvens de pontos |
| Visão Computacional | **OpenCV (cv2)** | `findChessboardCorners`, `projectPoints`, `calibrateCamera`, colormap |
| Nuvem de Pontos 3D | **Open3D** | Criação de `PointCloud` a partir de imagens RGB-D, substituindo a legada `libpcl` |
| Testes | **unittest** (stdlib) | Compatível com `pytest`, sem dependência externa, integração nativa com CI |
| Controle de Versão | **Git + GitHub** | Repositório `raquelbelchior1/PFC-2026` |

---

## 2. Motor Matemático — O Cérebro do Sistema

O módulo `motor_caixao_areia.py` implementa os quatro pilares matemáticos que convertem uma nuvem de pontos bruta do Kinect em pixels coloridos no projetor. Cada passo é descrito a seguir com rigor formal.

### 2.1 Passo 1 — Ajuste de Plano via Decomposição em Valores Singulares (SVD)

**Objetivo:** Dada uma nuvem de $N$ pontos $\{\mathbf{p}_i\}_{i=1}^{N} \subset \mathbb{R}^3$ capturados pelo Kinect sobre a superfície da mesa, encontrar o plano que melhor se ajusta a esses pontos no sentido dos mínimos quadráticos.

**Formulação.** O plano é descrito pela equação:

$$ax + by + cz + d = 0$$

onde $\mathbf{n} = (a, b, c)^T$ é o vetor normal unitário e $d = -\mathbf{n} \cdot \bar{\mathbf{p}}$, com $\bar{\mathbf{p}}$ sendo o centroide da nuvem.

**Método.** O algoritmo procede em três etapas:

1. **Centralização.** Calcula-se o centroide $\bar{\mathbf{p}} = \frac{1}{N}\sum_{i=1}^{N} \mathbf{p}_i$ e constrói-se a matriz centralizada:

$$M = \begin{bmatrix} (\mathbf{p}_1 - \bar{\mathbf{p}})^T \\ (\mathbf{p}_2 - \bar{\mathbf{p}})^T \\ \vdots \\ (\mathbf{p}_N - \bar{\mathbf{p}})^T \end{bmatrix} \in \mathbb{R}^{N \times 3}$$

2. **Decomposição SVD.** Aplica-se a fatoração $M = U \Sigma V^T$, onde $\Sigma = \text{diag}(\sigma_1, \sigma_2, \sigma_3)$ com $\sigma_1 \geq \sigma_2 \geq \sigma_3 \geq 0$.

3. **Extração da normal.** O vetor singular direito associado ao **menor valor singular** $\sigma_3$ — isto é, a última linha de $V^T$ — é o vetor que minimiza $\|M\mathbf{v}\|^2$ sujeito a $\|\mathbf{v}\| = 1$. Este vetor é precisamente a normal $\mathbf{n}$ do plano de melhor ajuste.

**Convenção de orientação.** O código garante que a componente $z$ da normal seja positiva ($n_z > 0$), de modo que a normal aponte "para cima" (em direção ao Kinect), invertendo o sinal se necessário.

**Coeficiente $d$.** Uma vez obtida a normal, calcula-se:

$$d = -\mathbf{n} \cdot \bar{\mathbf{p}}$$

garantindo que $\mathbf{n} \cdot \bar{\mathbf{p}} + d = 0$ (o centroide pertence ao plano).

**Implementação:** `ajustar_plano_svd()` em `motor_caixao_areia.py`.

---

### 2.2 Passo 2 — Construção do Sistema de Coordenadas da Mesa

**Objetivo:** Construir uma base ortonormal $\{X_{\text{mesa}}, Y_{\text{mesa}}, Z_{\text{mesa}}\}$ ancorada no plano da mesa, com $Z_{\text{mesa}}$ coincidindo com a normal do plano. Isso permitirá referenciar todos os pontos relativamente à superfície da mesa em vez do referencial arbitrário do Kinect.

**Método — Ortogonalização de Gram-Schmidt e Produto Vetorial:**

Seja $\mathbf{n}$ a normal unitária obtida no Passo 1.

1. **Eixo $Z_{\text{mesa}}$:** Define-se diretamente:

$$Z_{\text{mesa}} = \frac{\mathbf{n}}{\|\mathbf{n}\|}$$

2. **Vetor semente.** Escolhe-se um vetor auxiliar $\mathbf{s}$ que não seja (anti)paralelo a $Z_{\text{mesa}}$. Por padrão, $\mathbf{s} = (1, 0, 0)^T$; caso $|\mathbf{s} \cdot Z_{\text{mesa}}| \geq 0{,}9$, usa-se $\mathbf{s} = (0, 1, 0)^T$.

3. **Eixo $X_{\text{mesa}}$ — Gram-Schmidt.** Remove-se a componente de $\mathbf{s}$ na direção de $Z_{\text{mesa}}$:

$$X_{\text{mesa}} = \frac{\mathbf{s} - (\mathbf{s} \cdot Z_{\text{mesa}}) \, Z_{\text{mesa}}}{\|\mathbf{s} - (\mathbf{s} \cdot Z_{\text{mesa}}) \, Z_{\text{mesa}}\|}$$

O resultado é um vetor unitário tal que $X_{\text{mesa}} \cdot Z_{\text{mesa}} = 0$.

4. **Eixo $Y_{\text{mesa}}$ — Produto Vetorial.** Completa-se a base por:

$$Y_{\text{mesa}} = Z_{\text{mesa}} \times X_{\text{mesa}}$$

Como $Z_{\text{mesa}}$ e $X_{\text{mesa}}$ já são unitários e ortogonais, $Y_{\text{mesa}}$ é automaticamente unitário e ortogonal a ambos, formando uma base ortonormal dextrógira.

**Verificação de ortonormalidade.** As três condições são verificadas nos testes unitários:

$$X_{\text{mesa}} \cdot Y_{\text{mesa}} = 0, \quad X_{\text{mesa}} \cdot Z_{\text{mesa}} = 0, \quad Y_{\text{mesa}} \cdot Z_{\text{mesa}} = 0$$

$$\|X_{\text{mesa}}\| = \|Y_{\text{mesa}}\| = \|Z_{\text{mesa}}\| = 1$$

**Implementação:** `gram_schmidt()` e `construir_base_mesa()` em `motor_caixao_areia.py`.

---

### 2.3 Passo 3 — Transformação Afim 4×4 (Kinect → Mesa)

**Objetivo:** Montar uma matriz afim $T \in \mathbb{R}^{4 \times 4}$ que leve qualquer ponto do referencial do Kinect para o referencial da mesa, de forma que:

- A **origem** do novo referencial coincida com o centroide do plano.
- Os **eixos** do novo referencial coincidam com a base $\{X_{\text{mesa}}, Y_{\text{mesa}}, Z_{\text{mesa}}\}$.
- Pontos **sobre o plano** da mesa tenham coordenada $z_{\text{mesa}} = 0$.

**Construção.** A matriz de rotação $R \in \mathbb{R}^{3 \times 3}$ é formada empilhando os eixos da mesa como linhas:

$$R = \begin{bmatrix} X_{\text{mesa}}^T \\ Y_{\text{mesa}}^T \\ Z_{\text{mesa}}^T \end{bmatrix}$$

O vetor de translação é calculado como:

$$\mathbf{t} = -R \, \bar{\mathbf{p}}$$

onde $\bar{\mathbf{p}}$ é o centroide. Isso garante que $T \cdot [\bar{\mathbf{p}}^T, 1]^T = [0, 0, 0, 1]^T$.

A matriz afim completa é:

$$T = \begin{bmatrix} R & \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix} = \begin{bmatrix} X_{\text{mesa}}^T & t_x \\ Y_{\text{mesa}}^T & t_y \\ Z_{\text{mesa}}^T & t_z \\ 0 \; 0 \; 0 & 1 \end{bmatrix}$$

**Aplicação.** Para transformar um ponto $\mathbf{p} = (x, y, z)^T$:

$$\begin{bmatrix} x_{\text{mesa}} \\ y_{\text{mesa}} \\ z_{\text{mesa}} \\ 1 \end{bmatrix} = T \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}$$

**Propriedade fundamental:** se $\mathbf{p}$ pertence ao plano da mesa, então $z_{\text{mesa}} = Z_{\text{mesa}}^T \cdot \mathbf{p} + t_z = 0$. Pontos acima do plano têm $z_{\text{mesa}} > 0$ e pontos abaixo têm $z_{\text{mesa}} < 0$.

**Implementação:** `montar_matriz_transformacao()` e `transformar_pontos()` em `motor_caixao_areia.py`.

---

### 2.4 Passo 4 — Projeção Pinhole e Algoritmo de Tsai

**Objetivo:** Converter pontos 3D no referencial da mesa em coordenadas 2D (pixels) no plano de imagem do projetor, para que as cores de feedback sejam exibidas na posição correta sobre a areia.

**Modelo de Câmera Pinhole.** A projeção de um ponto $\mathbf{P}_{\text{mesa}} = (X, Y, Z)^T$ para o pixel $(u, v)$ é descrita por:

$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} R_{\text{ext}} & \mathbf{t}_{\text{ext}} \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}$$

onde:

- $K$ é a **matriz intrínseca** do projetor:

$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

com $f_x, f_y$ sendo as distâncias focais (em pixels) e $(c_x, c_y)$ o ponto principal.

- $R_{\text{ext}}, \mathbf{t}_{\text{ext}}$ são os **parâmetros extrínsecos** (rotação e translação do referencial da mesa para o referencial do projetor). A rotação é parametrizada pelo vetor de Rodrigues $\mathbf{r} \in \mathbb{R}^3$.

- $s$ é o fator de escala (profundidade do ponto no referencial da câmera).

**Modelo de distorção.** A função `cv2.projectPoints` embute o modelo de distorção radial e tangencial de Tsai:

$$x' = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2(r^2 + 2x^2)$$

$$y' = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2p_2 xy$$

onde $r^2 = x^2 + y^2$ e $(k_1, k_2, p_1, p_2, k_3)$ são os coeficientes de distorção.

**Projeção final em pixels:**

$$u = f_x \cdot x' + c_x, \quad v = f_y \cdot y' + c_y$$

**Exemplo verificado em teste.** Para um ponto na origem $(0, 0, 0)$ com $\mathbf{t}_{\text{ext}} = (0, 0, 1)^T$ (câmera a 1 metro de distância), $R_{\text{ext}} = I_3$ e $f_x = 320$, $c_x = 320$, $f_y = 240$, $c_y = 240$:

$$u = 320 \cdot \frac{0}{1} + 320 = 320, \quad v = 240 \cdot \frac{0}{1} + 240 = 240$$

O ponto projeta exatamente no centro da imagem — resultado confirmado pelo teste `test_ponto_na_origem_projeta_no_ponto_principal`.

**Implementação:** `projetar_pontos_tsai()` e `calibrar_projetor()` em `motor_caixao_areia.py`.

---

## 3. Integração de Hardware e Sensores

### 3.1 Kinect (RGB-D)

O sensor Microsoft Kinect fornece dois fluxos de dados simultâneos:

| Fluxo | Resolução | Tipo | Uso |
|---|---|---|---|
| Profundidade (IR) | 640 × 480 | `uint16` (mm) | Nuvem de pontos 3D |
| Cor (RGB) | 640 × 480 | `uint8` BGR | Texturização da nuvem (Open3D) |

**Modo simulado.** Enquanto o hardware não está disponível, o módulo `kinect.py` (Raquel) gera um mapa de profundidade sintético com uma colina gaussiana no centro e ruído aleatório:

```python
colina = 150 * exp(-((u - 320)² / (2·120²) + (v - 240)² / (2·90²)))
profundidade = 1000 - colina + ruído
```

A flag `USE_KINECT_REAL` em `kinect.py` controla a troca para o sensor real (`freenect.sync_get_depth()`).

**Nuvem de pontos com Open3D.** Quando a imagem colorida do Kinect estiver disponível, a função `criar_nuvem_de_pontos_open3d()` do motor matemático converte o par RGB-D em uma `PointCloud` 3D usando os parâmetros intrínsecos do sensor:

```python
intrinsic = PinholeCameraIntrinsic(W, H, fx=525.0, fy=525.0, cx=319.5, cy=239.5)
rgbd = RGBDImage.create_from_color_and_depth(cor, profundidade,
                                               depth_scale=1000.0,
                                               depth_trunc=3.0)
nuvem = PointCloud.create_from_rgbd_image(rgbd, intrinsic)
```

Estes valores correspondem aos parâmetros intrínsecos padrão do Kinect v1 (Xbox 360). A `depth_scale=1000.0` converte milímetros para metros e `depth_trunc=3.0` descarta pontos além de 3 metros.

### 3.2 Projetor — Calibração via Tabuleiro de Xadrez

Para que as cores projetadas alinhem-se corretamente à superfície da areia, é necessário calibrar o projetor como uma "câmera inversa". O procedimento planejado é:

1. **Projetar** uma imagem de tabuleiro de xadrez ($8 \times 6$ quadrados, $7 \times 5$ cantos internos) sobre a mesa de areia.
2. **Capturar** essa projeção com o Kinect.
3. **Detectar** os cantos com `encontrar_cantos_tabuleiro()`, que internamente chama:
   - `cv2.findChessboardCorners()` — detecção inicial.
   - `cv2.cornerSubPix()` — refinamento para precisão sub-pixel (critério: 30 iterações, $\varepsilon = 0{,}001$).
4. **Calibrar** via `calibrar_projetor()` → `cv2.calibrateCamera()`, obtendo:
   - Matriz intrínseca $K$ (distâncias focais, ponto principal).
   - Coeficientes de distorção $(k_1, k_2, p_1, p_2, k_3)$.
   - Parâmetros extrínsecos (vetor de Rodrigues $\mathbf{r}$, translação $\mathbf{t}$).
5. **Persistir** os parâmetros em um arquivo `.npz` para evitar recalibração a cada inicialização.

Enquanto a calibração física não é realizada, o sistema opera com parâmetros mockados ($f_x = f_y = 500$, $c_x = 320$, $c_y = 240$, distorção zero).

---

## 4. Lógica de Negócio e Padrão de Projeto (Adapter)

### 4.1 O Padrão Adapter — `AdaptadorMDE`

A equipe de Cartografia fornecerá o Modelo Digital de Elevação (MDE) em um formato que ainda não foi definido (GeoTIFF, matriz `.npy`, CSV, etc.). Para isolar completamente o motor matemático do formato de dados externo, foi implementado o **Padrão Adapter** (*Design Pattern — GoF*) no módulo `adaptador_mde.py`.

A classe `AdaptadorMDE` expõe uma interface estável de dois métodos:

| Método | Responsabilidade |
|---|---|
| `carregar_mapa(filepath)` | Lê o arquivo do MDE e popula a grade interna |
| `obter_z_alvo(x, y) → float` | Retorna a elevação esperada nas coordenadas $(x, y)$ da mesa |

O restante do sistema interage **exclusivamente** com esses dois métodos. Quando o arquivo real da Cartografia for entregue, basta alterar a implementação interna de `carregar_mapa()` — nenhum outro módulo precisa ser modificado.

```
┌────────────────────┐        ┌──────────────────┐        ┌──────────────┐
│  Motor Matemático  │───────▶│  AdaptadorMDE    │───────▶│  Arquivo MDE │
│  gerar_mapa_cores  │ chama  │  obter_z_alvo()  │  lê    │  (GeoTIFF,   │
│                    │        │  carregar_mapa()  │        │   .npy, etc) │
└────────────────────┘        └──────────────────┘        └──────────────┘
        ▲                            ▲
        │                            │
   Nunca muda                 Única classe que
                              muda quando o
                              formato do MDE mudar
```

**Mock atual.** Enquanto o MDE real não está disponível, `carregar_mapa()` gera uma grade sintética $100 \times 100$ com uma rampa linear:

$$z_{\text{mock}}(x, y) = 10 + 0{,}2x + 0{,}1y$$

As coordenadas $(x, y)$ da mesa são convertidas para índices da grade por:

$$\text{col} = \text{round}\!\left(\frac{x - x_{\text{origem}}}{\Delta}\right), \quad \text{lin} = \text{round}\!\left(\frac{y - y_{\text{origem}}}{\Delta}\right)$$

com *clamping* nas bordas para evitar acessos fora da grade.

### 4.2 Lógica de Coloração — `gerar_mapa_cores()`

Para cada ponto $i$ da nuvem já transformada para o referencial da mesa, o sistema extrai as coordenadas $(x_i, y_i, z_i^{\text{real}})$ e consulta o MDE para obter $z_i^{\text{MDE}} = \text{obter\_z\_alvo}(x_i, y_i)$.

A cor atribuída ao ponto segue a regra definida pelo orientador, parametrizada por uma tolerância $\tau$ (padrão: $5{,}0$ mm):

$$\text{cor}(i) = \begin{cases} \color{red}{\textbf{Vermelho}} \; (0, 0, 255)_{\text{BGR}} & \text{se } z_i^{\text{real}} > z_i^{\text{MDE}} + \tau \quad \text{(cavar)} \\[6pt] \color{blue}{\textbf{Azul}} \; (255, 0, 0)_{\text{BGR}} & \text{se } z_i^{\text{real}} < z_i^{\text{MDE}} - \tau \quad \text{(preencher)} \\[6pt] \color{green}{\textbf{Verde}} \; (0, 255, 0)_{\text{BGR}} & \text{caso contrário} \quad \text{(OK)} \end{cases}$$

> **Nota sobre a convenção de cores:** o OpenCV utiliza o canal de cores na ordem BGR (*Blue, Green, Red*), e não RGB. Todas as tuplas de cor no código seguem essa convenção.

A condição no limite exato ($z_i^{\text{real}} = z_i^{\text{MDE}} \pm \tau$) é tratada como **Verde** (dentro da tolerância), pois as comparações usam desigualdade estrita (`>` e `<`).

---

## 5. Qualidade de Software e TDD

### 5.1 Estratégia de Testes

A estabilidade do motor matemático foi assegurada utilizando **Test-Driven Development (TDD)**, resultando em uma suíte de **26 testes unitários** no módulo `test_motor_caixao.py`, executável tanto com `unittest` quanto com `pytest`:

```
python -m unittest test_motor_caixao -v
```

A suíte é organizada em **9 classes de teste**, cada uma focada em um componente específico:

| Classe | Testes | Componente Verificado |
|---|---|---|
| `TestAjustePlano` | 4 | SVD, normal unitária, equação do plano, exceção para $N < 3$ |
| `TestGramSchmidt` | 2 | Ortogonalidade, exceção para vetores paralelos |
| `TestConstruirBase` | 3 | Ortonormalidade mútua, $Z_{\text{mesa}} = \mathbf{n}$ |
| `TestMatrizTransformacao` | 3 | Identidade, translação anula origem, $z_{\text{mesa}} = 0$ no plano |
| `TestDeteccaoTabuleiro` | 2 | Imagem sem tabuleiro, tabuleiro sintético $7 \times 5$ |
| `TestProjecaoTsai` | 3 | Projeção no ponto principal, deslocamento em $x$, múltiplos pontos |
| `TestLeituraRGBD` | 1 | Importação condicional do Open3D (skip gracioso) |
| `TestMDEColoracao` | 6 | Vermelho/Azul/Verde, limites exatos, mock com rampa |
| `TestPipeline` | 2 | Integração completa dos Passos 1 + 2 |

### 5.2 Filosofia dos Testes — Transparência Matemática

Cada teste foi projetado para ser **auto-explicativo** perante a banca examinadora. Os valores são **hardcoded** e os comentários detalham a conta passo a passo. Exemplos:

**Teste: plano horizontal em $z = 5$**

```
Pontos: (0,0,5), (1,0,5), (0,1,5), (1,1,5), (2,3,5)
Normal esperada: (0, 0, 1)  — o plano é horizontal
d esperado: -5

Verificação: n · p + d = (0,0,1)·(0,0,5) + (-5) = 5 - 5 = 0  ✓
```

**Teste: translação anula a origem**

```
Base canônica, origem = (10, 20, 30)
t = -R · origem = -I₃ · (10,20,30) = (-10,-20,-30)

Então: T · [10, 20, 30, 1]ᵀ = [0, 0, 0, 1]ᵀ  ✓
```

**Teste: projeção pinhole**

```
Ponto (0,0,0), tvec = (0,0,1), fx = 320, cx = 320
u = fx · (X/Z) + cx = 320 · (0/1) + 320 = 320
v = fy · (Y/Z) + cy = 240 · (0/1) + 240 = 240

Pixel esperado: (320, 240) — centro da imagem  ✓
```

### 5.3 Mock do MDE para Testes

A suíte inclui um mock dedicado que simula um terreno com rampa linear:

$$z_{\text{mock}}(x, y) = 10 + 0{,}5 \cdot x$$

Esse mock permite testar a lógica de coloração com cenários determinísticos:

| Ponto | $z_{\text{real}}$ | $z_{\text{MDE}}$ | Diferença | Cor |
|---|---|---|---|---|
| $(0, 0, 25)$ | 25 | 10 | $+15 > +5$ | Vermelho |
| $(0, 0, 2)$ | 2 | 10 | $-8 < -5$ | Azul |
| $(0, 0, 11)$ | 11 | 10 | $+1 \in [-5, +5]$ | Verde |

---

## 6. Status Atual e Próximos Passos

### 6.1 O que já foi implementado

| Item | Módulo | Status |
|---|---|---|
| Ajuste de plano por SVD | `motor_caixao_areia.py` | Implementado e testado (4 testes) |
| Base ortonormal da mesa (Gram-Schmidt) | `motor_caixao_areia.py` | Implementado e testado (5 testes) |
| Matriz de transformação afim 4×4 | `motor_caixao_areia.py` | Implementado e testado (3 testes) |
| Projeção 3D→2D (Tsai/Pinhole) | `motor_caixao_areia.py` | Implementado e testado (3 testes) |
| Detecção de tabuleiro de xadrez | `motor_caixao_areia.py` | Implementado e testado (2 testes) |
| Leitura RGBD com Open3D | `motor_caixao_areia.py` | Implementado (teste condicional) |
| Lógica de coloração MDE (Verm./Azul/Verde) | `motor_caixao_areia.py` | Implementado e testado (6 testes) |
| Adaptador MDE (Padrão Adapter) | `adaptador_mde.py` | Implementado com mock interno |
| Pipeline integrado (Calibração + Loop AR) | `app_integrado.py` | Implementado, executável com simulador |
| Captura simulada do Kinect | `kinect.py` | Implementado (Raquel) |
| Exibição no projetor | `exibicao.py` | Implementado (Raquel) |
| Suíte de testes unitários | `test_motor_caixao.py` | 26 testes — 25 passam, 1 skip (Open3D) |

### 6.2 Roadmap de Integração — O que falta

#### Hardware

- [ ] **Posicionamento físico do Kinect** sobre a mesa de areia, perpendicular à superfície, a aproximadamente 1–1,5 m de altura.
- [ ] **Montagem e alinhamento do projetor**, apontando para a mesa com resolução e foco adequados.
- [ ] **Conexão USB do Kinect** e ativação da flag `USE_KINECT_REAL = True` em `kinect.py`.

#### Calibração do Projetor (Passos 3 e 4 do Orientador)

- [ ] Imprimir e posicionar tabuleiro de xadrez na mesa de areia.
- [ ] Projetar padrão de calibração do projetor sobre a mesa.
- [ ] Capturar imagem pelo Kinect e executar `encontrar_cantos_tabuleiro()`.
- [ ] Executar `calibrar_projetor()` para gerar $K$, $(k_1, k_2, p_1, p_2, k_3)$, $\mathbf{r}$, $\mathbf{t}$.
- [ ] Salvar parâmetros em `calibracao_projetor.npz` e carregar automaticamente em `app_integrado.py`.

#### Cartografia / MDE

- [ ] **Receber o arquivo final do MDE** da equipe de Cartografia (formato esperado: GeoTIFF, `.npy` ou CSV).
- [ ] **Implementar a leitura real** no método `AdaptadorMDE.carregar_mapa()`, substituindo o mock atual.
- [ ] **Definir a resolução espacial** e o sistema de coordenadas do MDE, compatibilizando com o referencial da mesa.

#### Condições Ambientais

- [ ] **Refinamento das condições de iluminação** — a leitura infravermelha do Kinect é sensível à luz solar direta e fontes IR externas. Recomenda-se operar em ambiente com iluminação controlada.
- [ ] **Ajuste da tolerância $\tau$** com base em testes empíricos com areia real (granulometria, compactação e ruído do sensor afetam o valor ideal).

#### Software — Melhorias Futuras

- [ ] **Instalar Open3D** e ativar o pipeline RGBD completo (Passo 5) quando a imagem colorida do Kinect estiver disponível.
- [ ] **Tela cheia no projetor** — ativar `tela_cheia=True` na chamada a `exibir_profundidade()`.
- [ ] **Suavização temporal** — aplicar filtro de média móvel entre frames para reduzir flickering na projeção.
- [ ] **Persistência de calibração** — salvar/carregar automaticamente os parâmetros do plano da mesa entre sessões.

---

> **Documento gerado em:** Março de 2026
> **Versão:** 1.0
> **Autoria:** Rafael Schuinki — Motor Matemático e Integração de Sistemas

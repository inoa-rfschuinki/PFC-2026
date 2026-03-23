"""
main.py — Orquestrador Principal: Máquina de Estados do AR Sandbox
===================================================================
Projeto Final de Curso — Engenharia de Computação (AMAN, 2026)

Script **Plug & Play** que orquestra todo o pipeline do Caixão de
Areia com Realidade Aumentada.  Funciona com ou sem Kinect conectado.

Dimensões físicas reais
-----------------------
- Caixa de areia: 1,5 m × 1,5 m × 0,3 m de profundidade
- Kinect montado a 2,5 m de altura

Janelas de saída
----------------
- **Projecao_Areia** — feedback AR do projetor (vermelho/azul/verde)
- **Gabarito_MDE** — heatmap de referência do MDE sendo replicado

Máquina de Estados
------------------
INIT
    Inicializa o sensor (``KinectSensor`` com fallback), carrega o MDE
    (com fallback para superfície sintética) e transiciona para IDLE.
IDLE
    Exibe o mapa de profundidade colorido enquanto aguarda o comando
    de calibração.  Tecla **C** → transiciona para CALIBRACAO.
CALIBRACAO
    Captura nuvem de pontos → SVD → Gram-Schmidt → Matriz 4×4.
    Ao concluir, transiciona automaticamente para AR_LOOP.
AR_LOOP
    Loop contínuo: captura → transforma → compara MDE → colore → projeta.
    Tecla **C** → volta para CALIBRACAO.
    Tecla **Q** / ESC → encerra.

Configuração
------------
As variáveis no topo do arquivo controlam caminhos de arquivo,
resolução e tolerância — edite-as antes de rodar.
"""

from __future__ import annotations

import logging
import sys
import time
from enum import Enum, auto
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox

import cv2
import numpy as np

# ── Camada de hardware ────────────────────────────────────────────────
from kinect_sensor import KinectSensor

# ── Motor matemático ──────────────────────────────────────────────────
from motor_caixao_areia import (
    pipeline_plano_e_base,
    transformar_pontos,
    gerar_mapa_cores,
    projetar_pontos_tsai,
    encontrar_cantos_tabuleiro,
    calibrar_projetor,
    gerar_imagem_grade_cores,
)

# ── Adaptador MDE ────────────────────────────────────────────────────
from mde_cartografia import AdaptadorMDE


# =====================================================================
# CONFIGURAÇÃO — edite aqui antes de rodar
# =====================================================================

CAMINHO_GEOTIFF: str = "25S51_ZN.tif"
"""Caminho para o GeoTIFF da Cartografia.  Se não existir, o sistema
gera uma superfície sintética automaticamente."""

RESOLUCAO_PROJETOR: tuple[int, int] = (640, 480)
"""``(largura, altura)`` em pixels da janela de projeção."""

TOLERANCIA_COR: float = 0.02
"""Tolerância em metros (2 cm) para a classificação Vermelho/Azul/Verde."""

LARGURA_MESA: float = 1.50
"""Dimensão X da caixa de areia em metros (1,5 m)."""

COMPRIMENTO_MESA: float = 1.50
"""Dimensão Y da caixa de areia em metros (1,5 m)."""

ALTURA_MAX_AREIA: float = 0.30
"""Espessura máxima de areia mapeável em metros (30 cm)."""

ALTURA_KINECT: float = 2.50
"""Altura de montagem do Kinect acima da mesa, em metros."""

CELULAS_GRADE_X: int = 30
"""Número de colunas da grade de discretização (eixo X).
Com a mesa de 1,5 m, 30 colunas geram quadrados de 5 cm × 5 cm."""

CELULAS_GRADE_Y: int = 30
"""Número de linhas da grade de discretização (eixo Y).
Com a mesa de 1,5 m, 30 linhas geram quadrados de 5 cm × 5 cm."""

RAIO_PA_VIRTUAL: float = 0.10
"""Raio de ação da pá virtual (mouse) em metros (10 cm)."""

INTENSIDADE_PA_VIRTUAL: float = 0.008
"""Deslocamento de areia por evento de mouse no centro do pincel (8 mm)."""

FORCAR_SIMULACAO: bool = False
"""Se ``True``, ignora o Kinect e usa nuvem sintética."""

JANELA_PROJECAO: str = "Projecao_Areia"
"""Nome da janela OpenCV de projeção AR (para o projetor)."""

JANELA_GABARITO: str = "Gabarito_MDE"
"""Nome da janela OpenCV com o heatmap de referência do MDE."""

# =====================================================================
# Logging
# =====================================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")


# =====================================================================
# Máquina de Estados
# =====================================================================

class Estado(Enum):
    """Estados possíveis da aplicação."""
    INIT = auto()
    IDLE = auto()
    CALIBRACAO = auto()
    AR_LOOP = auto()
    ENCERRAR = auto()


class DadosCalibracao:
    """Armazena os resultados de uma calibração completa.

    Attributes
    ----------
    T : np.ndarray | None
        Matriz afim 4×4 Kinect → Mesa.
    normal : np.ndarray | None
        Vetor normal do plano da mesa.
    centroide : np.ndarray | None
        Centroide da nuvem usada na calibração.
    camera_matrix : np.ndarray
        Matriz intrínseca do projetor (3×3).
    dist_coeffs : np.ndarray
        Coeficientes de distorção.
    rvec : np.ndarray
        Vetor de Rodrigues (rotação extrínseca → projetor).
    tvec : np.ndarray
        Translação extrínseca → projetor.
    """

    def __init__(self) -> None:
        self.T: Optional[np.ndarray] = None
        self.normal: Optional[np.ndarray] = None
        self.centroide: Optional[np.ndarray] = None

        # Parâmetros do projetor — defaults realistas (mock)
        # Serão sobrescritos quando a calibração real for feita
        self.camera_matrix: np.ndarray = np.array([
            [500.0,   0.0, 320.0],
            [  0.0, 500.0, 240.0],
            [  0.0,   0.0,   1.0],
        ])
        self.dist_coeffs: np.ndarray = np.zeros(5)
        self.rvec: np.ndarray = np.zeros((3, 1))
        self.tvec: np.ndarray = np.array([[0.0], [0.0], [1.0]])

    @property
    def esta_calibrado(self) -> bool:
        """``True`` se a calibração da mesa (Passos 1+2) foi concluída."""
        return self.T is not None


# =====================================================================
# Callback do Mouse — "Pá Virtual" para simulação interativa
# =====================================================================

# Estado global do mouse (necessário para o callback do OpenCV)
_mouse_botao_esquerdo: bool = False
_mouse_botao_direito: bool = False
_sensor_ref: Optional[KinectSensor] = None


def _callback_mouse(evento: int, x_pixel: int, y_pixel: int,
                    flags: int, param: None) -> None:
    """Callback ``cv2.setMouseCallback`` para cavar/preencher areia.

    Mapeia a coordenada do pixel do mouse para a coordenada física
    da mesa (em metros) e chama ``sensor.modificar_areia()``.

    - **Botão esquerdo** (segurar + arrastar): **cava** a areia
      (diminui Z) com decaimento Gaussiano.
    - **Botão direito** (segurar + arrastar): **preenche** a areia
      (aumenta Z) com decaimento Gaussiano.

    O efeito é acumulativo: quanto mais tempo o mouse é arrastado
    sobre um ponto, maior a alteração de altura.

    Parameters
    ----------
    evento : int
        Tipo de evento OpenCV (``cv2.EVENT_*``).
    x_pixel, y_pixel : int
        Coordenadas do mouse na janela, em pixels.
    flags : int
        Flags de estado (botões pressionados).
    param : None
        Dados do usuário (não utilizado).
    """
    global _mouse_botao_esquerdo, _mouse_botao_direito

    if evento == cv2.EVENT_LBUTTONDOWN:
        _mouse_botao_esquerdo = True
    elif evento == cv2.EVENT_LBUTTONUP:
        _mouse_botao_esquerdo = False
    elif evento == cv2.EVENT_RBUTTONDOWN:
        _mouse_botao_direito = True
    elif evento == cv2.EVENT_RBUTTONUP:
        _mouse_botao_direito = False

    # Processar arraste (MOVE enquanto botão pressionado)
    if not (_mouse_botao_esquerdo or _mouse_botao_direito):
        return
    if _sensor_ref is None or not _sensor_ref.esta_simulando:
        return

    # Obter tamanho real da janela (pode estar redimensionada)
    try:
        largura_janela = cv2.getWindowImageRect(JANELA_PROJECAO)[2]
        altura_janela = cv2.getWindowImageRect(JANELA_PROJECAO)[3]
    except cv2.error:
        largura_janela, altura_janela = RESOLUCAO_PROJETOR

    if largura_janela <= 0 or altura_janela <= 0:
        return

    # Ignorar cliques fora da área da imagem
    if x_pixel < 0 or y_pixel < 0 or x_pixel >= largura_janela or y_pixel >= altura_janela:
        return

    # Mapear pixel → coordenada física da mesa (metros)
    x_mesa = (x_pixel / largura_janela) * LARGURA_MESA
    y_mesa = (y_pixel / altura_janela) * COMPRIMENTO_MESA

    _sensor_ref.modificar_areia(
        x_mesa, y_mesa,
        cavar=_mouse_botao_esquerdo,
        raio=RAIO_PA_VIRTUAL,
        intensidade=INTENSIDADE_PA_VIRTUAL,
    )


# =====================================================================
# Funções auxiliares do pipeline
# =====================================================================

def _executar_calibracao(sensor: KinectSensor) -> DadosCalibracao:
    """Captura a nuvem atual e calcula plano + base + matriz T.

    No modo simulação, os pontos já estão em coordenadas da mesa,
    então T = identidade e os parâmetros de projeção são calculados
    para mapear [0, 1.5] m × [0, 1.5] m → pixels da imagem.

    Parameters
    ----------
    sensor : KinectSensor
        Sensor (real ou simulado).

    Returns
    -------
    DadosCalibracao
        Objeto com os parâmetros de calibração preenchidos.

    Raises
    ------
    RuntimeError
        Se a nuvem de pontos for insuficiente (< 10 pontos).
    """
    dados = DadosCalibracao()

    print("\n" + "=" * 50)
    print("  CALIBRAÇÃO — Passos 1 e 2 (SVD + Gram-Schmidt)")
    print("=" * 50)

    # ── Modo Simulação: pontos já em coordenadas da mesa ──
    if sensor.esta_simulando:
        dados.T = np.eye(4)
        dados.normal = np.array([0.0, 0.0, 1.0])
        dados.centroide = np.array([0.75, 0.75, 0.15])

        # Projeção: mapear mesa [0, L] × [0, C] → pixels do projetor
        largura, altura = RESOLUCAO_PROJETOR
        d_cam = 10.0  # distância virtual da câmera (metros)
        fx = largura * d_cam / LARGURA_MESA
        fy = altura * d_cam / COMPRIMENTO_MESA
        dados.camera_matrix = np.array([
            [fx,  0.0, 0.0],
            [0.0, fy,  0.0],
            [0.0, 0.0, 1.0],
        ])
        dados.dist_coeffs = np.zeros(5)
        dados.rvec = np.zeros((3, 1))
        dados.tvec = np.array([[0.0], [0.0], [d_cam]])

        print("  Modo Simulação — calibração automática.")
        print("  T = Identidade (pontos já em coordenadas da mesa).")
        print(f"  Projeção: fx={fx:.1f}, fy={fy:.1f}, d={d_cam:.1f} m")
        print("  ✓ Calibração concluída (simulação).")
        print("=" * 50 + "\n")
        return dados

    # ── Modo Real: SVD + Gram-Schmidt ──
    pontos = sensor.capturar_nuvem()
    if pontos.shape[0] < 10:
        raise RuntimeError(
            "Nuvem com poucos pontos — verifique a posição do sensor."
        )
    print(f"  Pontos capturados: {pontos.shape[0]:,}")

    normal, d, centroide, X, Y, Z, T = pipeline_plano_e_base(pontos)

    dados.T = T
    dados.normal = normal
    dados.centroide = centroide

    print(f"  Plano: {normal[0]:.4f}x + {normal[1]:.4f}y "
          f"+ {normal[2]:.4f}z + {d:.2f} = 0")
    print(f"  Centroide: ({centroide[0]:.1f}, {centroide[1]:.1f}, {centroide[2]:.1f})")
    print("  ✓ Calibração concluída.")
    print("=" * 50 + "\n")

    return dados


def _processar_frame_ar(
    sensor: KinectSensor,
    calibracao: DadosCalibracao,
    mde: AdaptadorMDE,
    resolucao: tuple[int, int],
    tolerancia: float,
) -> np.ndarray:
    """Processa um frame completo do pipeline AR com discretização em grade.

    Em vez de colorir pontos individuais da nuvem do Kinect (abordagem
    ruidosa e com buracos), a mesa é dividida em uma **malha regular**
    de ``CELULAS_GRADE_Y × CELULAS_GRADE_X`` quadrados (por padrão
    30 × 30 = 5 cm × 5 cm cada).

    Pipeline por frame (abordagem de **grade**):

    1. **Captura** — obtém a nuvem 3D do Kinect.
    2. **Transformação** — aplica T (Kinect → Mesa) nos pontos.
    3. **Discretização** — agrupa os pontos em células da grade e
       calcula :math:`Z_{real\_media}` por célula (filtra ruído).
    4. **Comparação MDE** — consulta :math:`Z_{MDE}` no centro de
       cada célula e classifica a cor (Vermelho / Azul / Verde).
    5. **Projeção Tsai** — projeta os vértices da grade em lote
       via ``cv2.projectPoints``.
    6. **Rasterização** — desenha cada célula como polígono
       preenchido (``cv2.fillPoly``), gerando uma imagem contínua
       sem buracos.

    Parameters
    ----------
    sensor : KinectSensor
    calibracao : DadosCalibracao
    mde : AdaptadorMDE
    resolucao : tuple[int, int]
        ``(largura, altura)`` do projetor.
    tolerancia : float
        Tolerância em metros para a classificação de cores.

    Returns
    -------
    np.ndarray, shape (H, W, 3), dtype uint8
        Imagem BGR com grade contínua de quadrados coloridos.
    """
    largura, altura = resolucao

    # 1. Captura
    pontos = sensor.capturar_nuvem()
    if pontos.shape[0] == 0:
        return np.zeros((altura, largura, 3), dtype=np.uint8)

    # 2. Transformar Kinect → Mesa
    pontos_mesa = transformar_pontos(calibracao.T, pontos)

    # 3–6. Grade discretizada → imagem contínua de quadrados coloridos
    imagem = gerar_imagem_grade_cores(
        pontos_mesa=pontos_mesa,
        funcao_mde=mde.obter_z_alvo,
        tolerancia=tolerancia,
        n_celulas_x=CELULAS_GRADE_X,
        n_celulas_y=CELULAS_GRADE_Y,
        largura_mesa=LARGURA_MESA,
        comprimento_mesa=COMPRIMENTO_MESA,
        rvec=calibracao.rvec,
        tvec=calibracao.tvec,
        camera_matrix=calibracao.camera_matrix,
        dist_coeffs=calibracao.dist_coeffs,
        resolucao=resolucao,
    )

    return imagem


# =====================================================================
# GUI de Configuração Inicial (Tkinter)
# =====================================================================

def _abrir_gui_configuracao() -> Optional[dict]:
    """Exibe janela Tkinter para o operador selecionar o mapa e confirmar
    as dimensões da mesa antes de iniciar o pipeline AR.

    Returns
    -------
    dict | None
        Dicionário com ``caminho_geotiff``, ``largura_mesa``,
        ``comprimento_mesa`` e ``altura_max_areia``.
        ``None`` se o usuário fechou a janela sem iniciar.
    """
    resultado: dict = {}

    root = tk.Tk()
    root.title("Caixão de Areia — Configuração")
    root.resizable(False, False)

    # ── Variáveis de estado ──
    var_caminho = tk.StringVar(value="")
    var_largura = tk.StringVar(value="1.50")
    var_comprimento = tk.StringVar(value="1.50")
    var_altura = tk.StringVar(value="0.30")
    var_demo = tk.BooleanVar(value=False)

    # ── Título ──
    tk.Label(
        root,
        text="Caixão de Areia — AR Sandbox",
        font=("Segoe UI", 16, "bold"),
    ).pack(pady=(18, 2))
    tk.Label(
        root,
        text="PFC Engenharia de Computação — IME 2026",
        font=("Segoe UI", 10),
        fg="#555555",
    ).pack(pady=(0, 15))

    # ── Seção: Mapa Tático ──
    frame_mapa = tk.LabelFrame(
        root,
        text="  Mapa Tático  ",
        font=("Segoe UI", 11, "bold"),
        padx=15,
        pady=10,
    )
    frame_mapa.pack(padx=20, pady=(0, 5), fill="x")

    lbl_caminho = tk.Label(
        frame_mapa,
        text="Nenhum arquivo selecionado",
        font=("Segoe UI", 9),
        fg="#999999",
        wraplength=380,
        anchor="w",
        justify="left",
    )

    def selecionar_mapa():
        caminho = filedialog.askopenfilename(
            title="Selecionar Mapa Tático GeoTIFF",
            filetypes=[
                ("GeoTIFF", "*.tif *.tiff"),
                ("Todos os arquivos", "*.*"),
            ],
        )
        if caminho:
            var_caminho.set(caminho)
            lbl_caminho.config(text=caminho, fg="#1a1a1a")
            if not var_demo.get():
                btn_iniciar.config(state="normal")

    btn_mapa = tk.Button(
        frame_mapa,
        text="Selecionar Mapa Tático (.TIF)",
        command=selecionar_mapa,
        font=("Segoe UI", 11),
        padx=10,
        pady=6,
        cursor="hand2",
    )
    btn_mapa.pack(fill="x")
    lbl_caminho.pack(fill="x", pady=(5, 5))

    def ao_alternar_demo():
        if var_demo.get():
            btn_mapa.config(state="disabled")
            lbl_caminho.config(
                text="Modo demonstração — Morro Gaussiano sintético",
                fg="#2e7d32",
            )
            btn_iniciar.config(state="normal")
        else:
            btn_mapa.config(state="normal")
            if var_caminho.get():
                lbl_caminho.config(text=var_caminho.get(), fg="#1a1a1a")
                btn_iniciar.config(state="normal")
            else:
                lbl_caminho.config(
                    text="Nenhum arquivo selecionado", fg="#999999",
                )
                btn_iniciar.config(state="disabled")

    tk.Checkbutton(
        frame_mapa,
        text="Usar mapa de demonstração (sem arquivo .TIF)",
        variable=var_demo,
        command=ao_alternar_demo,
        font=("Segoe UI", 9),
    ).pack(anchor="w")

    # ── Seção: Dimensões da Mesa ──
    frame_mesa = tk.LabelFrame(
        root,
        text="  Dimensões da Mesa (metros)  ",
        font=("Segoe UI", 11, "bold"),
        padx=15,
        pady=10,
    )
    frame_mesa.pack(padx=20, pady=10, fill="x")

    for i, (rotulo, var) in enumerate([
        ("Largura (X):", var_largura),
        ("Comprimento (Y):", var_comprimento),
        ("Altura Máx. Areia (Z):", var_altura),
    ]):
        tk.Label(
            frame_mesa, text=rotulo, font=("Segoe UI", 10),
        ).grid(row=i, column=0, sticky="w", pady=4)
        tk.Entry(
            frame_mesa,
            textvariable=var,
            font=("Segoe UI", 10),
            width=10,
            justify="center",
        ).grid(row=i, column=1, padx=(15, 0), pady=4)

    # ── Botão INICIAR ──
    def iniciar():
        try:
            largura = float(var_largura.get())
            comprimento = float(var_comprimento.get())
            altura = float(var_altura.get())
        except ValueError:
            messagebox.showerror(
                "Valores Inválidos",
                "As dimensões da mesa devem ser números decimais.\n"
                "Exemplo: 1.50",
            )
            return
        if largura <= 0 or comprimento <= 0 or altura <= 0:
            messagebox.showerror(
                "Valores Inválidos",
                "Todas as dimensões devem ser maiores que zero.",
            )
            return

        resultado["caminho_geotiff"] = (
            "" if var_demo.get() else var_caminho.get()
        )
        resultado["largura_mesa"] = largura
        resultado["comprimento_mesa"] = comprimento
        resultado["altura_max_areia"] = altura
        root.destroy()

    btn_iniciar = tk.Button(
        root,
        text="INICIAR SIMULAÇÃO",
        command=iniciar,
        font=("Segoe UI", 14, "bold"),
        padx=20,
        pady=12,
        state="disabled",
        cursor="hand2",
    )
    btn_iniciar.pack(pady=(5, 20), padx=20, fill="x")

    # ── Centralizar janela na tela ──
    root.update_idletasks()
    w = root.winfo_reqwidth()
    h = root.winfo_reqheight()
    x = (root.winfo_screenwidth() - w) // 2
    y = (root.winfo_screenheight() - h) // 2
    root.geometry(f"+{x}+{y}")

    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()

    return resultado if resultado else None


# =====================================================================
# Loop principal — Máquina de Estados
# =====================================================================

def main() -> None:
    """Ponto de entrada do sistema AR Sandbox.

    Implementa uma máquina de estados com transições controladas
    por ``cv2.waitKey``.  Toda a inicialização de hardware e MDE
    possui fallback resiliente — o sistema nunca crasha por falta
    de hardware ou arquivo.
    """
    # ── GUI de Configuração Inicial ──────────────────────────────
    config = _abrir_gui_configuracao()
    if config is None:
        print("\n✓ Operação cancelada pelo usuário.")
        sys.exit(0)

    global CAMINHO_GEOTIFF, LARGURA_MESA, COMPRIMENTO_MESA, ALTURA_MAX_AREIA
    CAMINHO_GEOTIFF = config["caminho_geotiff"]
    LARGURA_MESA = config["largura_mesa"]
    COMPRIMENTO_MESA = config["comprimento_mesa"]
    ALTURA_MAX_AREIA = config["altura_max_areia"]

    print()
    print("╔══════════════════════════════════════════════════╗")
    print("║     CAIXÃO DE AREIA — AR Sandbox                ║")
    print("║     PFC Engenharia de Computação — AMAN 2026    ║")
    print(f"║     Mesa: {LARGURA_MESA} m × {COMPRIMENTO_MESA} m × {ALTURA_MAX_AREIA} m")
    print("║     Kinect: 2.5 m de altura                     ║")
    print("╚══════════════════════════════════════════════════╝")
    print()

    estado = Estado.INIT
    sensor: Optional[KinectSensor] = None
    mde: Optional[AdaptadorMDE] = None
    calibracao: Optional[DadosCalibracao] = None
    imagem_gabarito: Optional[np.ndarray] = None
    t_anterior = time.time()

    global _sensor_ref  # necessário para o callback do mouse

    while estado != Estado.ENCERRAR:

        # ── INIT ──────────────────────────────────────────
        if estado == Estado.INIT:
            logger.info("Estado: INIT — inicializando hardware e MDE.")

            # Hardware (com fallback automático)
            try:
                sensor = KinectSensor(
                    forcar_simulacao=FORCAR_SIMULACAO,
                    largura_mesa=LARGURA_MESA,
                    comprimento_mesa=COMPRIMENTO_MESA,
                    altura_max_areia=ALTURA_MAX_AREIA,
                )
            except Exception as e:
                logger.error("Falha crítica ao criar KinectSensor: %s", e)
                print(f"[ERRO FATAL] Não foi possível iniciar o sensor: {e}")
                sys.exit(1)

            # MDE (com fallback para superfície sintética)
            mde = AdaptadorMDE(
                caminho_geotiff=CAMINHO_GEOTIFF,
                largura_mesa=LARGURA_MESA,
                comprimento_mesa=COMPRIMENTO_MESA,
                altura_max_areia=ALTURA_MAX_AREIA,
            )

            # Gerar heatmap do MDE (exibido na janela Gabarito)
            imagem_gabarito = mde.gerar_imagem_visualizacao(
                largura=RESOLUCAO_PROJETOR[0],
                altura=RESOLUCAO_PROJETOR[1],
            )

            # Criar janelas OpenCV — dual display
            cv2.namedWindow(JANELA_PROJECAO, cv2.WINDOW_NORMAL)
            cv2.namedWindow(JANELA_GABARITO, cv2.WINDOW_NORMAL)

            # Registrar callback do mouse para simulação interativa
            _sensor_ref = sensor
            cv2.setMouseCallback(JANELA_PROJECAO, _callback_mouse)

            # Exibir heatmap do MDE imediatamente
            cv2.imshow(JANELA_GABARITO, imagem_gabarito)

            print()
            print("Janelas:")
            print(f"  [{JANELA_PROJECAO}] Projeção AR (vermelho/azul/verde)")
            print(f"  [{JANELA_GABARITO}] Heatmap de referência do MDE")
            print()
            print("Teclas:")
            print("  [C] Calibrar plano da mesa (SVD + Gram-Schmidt)")
            print("  [F] Tela cheia ON/OFF (janela de projeção)")
            print("  [Q] ou [ESC] Encerrar")
            if sensor.esta_simulando:
                print()
                print("  Pá Virtual (Simulação Interativa):")
                print("  [Botão Esquerdo + Arrastar] Cavar areia")
                print("  [Botão Direito  + Arrastar] Colocar areia")
            print()
            estado = Estado.IDLE

        # ── IDLE ──────────────────────────────────────────
        elif estado == Estado.IDLE:
            # Exibir profundidade colorida enquanto aguarda calibração
            profundidade = sensor.capturar_profundidade()
            imagem = KinectSensor.profundidade_para_imagem(profundidade)

            # Overlay de instrução
            cv2.putText(
                imagem,
                "Aperte [C] para calibrar",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            cv2.imshow(JANELA_PROJECAO, imagem)
            cv2.imshow(JANELA_GABARITO, imagem_gabarito)

            tecla = cv2.waitKey(30) & 0xFF
            if tecla in (ord("c"), ord("C")):
                estado = Estado.CALIBRACAO
            elif tecla in (ord("q"), ord("Q"), 27):  # 27 = ESC
                estado = Estado.ENCERRAR

        # ── CALIBRACAO ────────────────────────────────────
        elif estado == Estado.CALIBRACAO:
            logger.info("Estado: CALIBRACAO")
            try:
                calibracao = _executar_calibracao(sensor)
                estado = Estado.AR_LOOP
            except RuntimeError as e:
                logger.error("Calibração falhou: %s", e)
                print(f"[ERRO] {e}")
                print("[ERRO] Voltando para IDLE — tente novamente com [C].")
                estado = Estado.IDLE

        # ── AR_LOOP ───────────────────────────────────────
        elif estado == Estado.AR_LOOP:
            try:
                imagem_ar = _processar_frame_ar(
                    sensor, calibracao, mde,
                    resolucao=RESOLUCAO_PROJETOR,
                    tolerancia=TOLERANCIA_COR,
                )
            except Exception as e:
                logger.warning("Erro no frame AR: %s", e)
                continue

            # FPS
            agora = time.time()
            fps = 1.0 / max(agora - t_anterior, 1e-6)
            t_anterior = agora

            # Overlay de FPS e status
            cv2.putText(
                imagem_ar,
                f"FPS: {fps:.1f} | [C] Recalibrar | [Q] Sair",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 200, 200),
                1,
            )
            cv2.imshow(JANELA_PROJECAO, imagem_ar)
            cv2.imshow(JANELA_GABARITO, imagem_gabarito)

            tecla = cv2.waitKey(1) & 0xFF
            if tecla in (ord("c"), ord("C")):
                estado = Estado.CALIBRACAO
            elif tecla in (ord("q"), ord("Q"), 27):
                estado = Estado.ENCERRAR
            elif tecla in (ord("f"), ord("F")):
                # Toggle tela cheia (janela de projeção)
                prop = cv2.getWindowProperty(
                    JANELA_PROJECAO, cv2.WND_PROP_FULLSCREEN
                )
                if prop == cv2.WINDOW_FULLSCREEN:
                    cv2.setWindowProperty(
                        JANELA_PROJECAO,
                        cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_NORMAL,
                    )
                else:
                    cv2.setWindowProperty(
                        JANELA_PROJECAO,
                        cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN,
                    )

    # ── Encerramento ──────────────────────────────────────
    if sensor is not None:
        sensor.liberar()
    cv2.destroyAllWindows()
    print("\n✓ Sistema encerrado com sucesso.")


# =====================================================================
# Entry point
# =====================================================================

if __name__ == "__main__":
    main()
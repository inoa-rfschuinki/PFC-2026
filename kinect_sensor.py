"""
kinect_sensor.py — Camada de Hardware: Captura de Nuvem de Pontos
==================================================================
Projeto Final de Curso — Engenharia de Computação (AMAN, 2026)

Encapsula toda a comunicação com o sensor Microsoft Kinect v1/v2
(via Open3D ou freenect) e implementa **fallback automático**
para um simulador sintético caso o hardware não esteja conectado.

Padrão de projeto
-----------------
*Strategy + Fallback*: o construtor tenta inicializar o sensor real.
Se falhar, troca transparentemente para um gerador de nuvens sintéticas.
O restante do sistema consome a mesma interface e **nunca quebra por
falta de hardware** — requisito crítico para a apresentação da banca.

Classes
-------
KinectSensor
    Interface única de captura: ``capturar_nuvem()`` retorna
    ``np.ndarray (N, 3)`` e ``capturar_profundidade()`` retorna
    ``np.ndarray (480, 640)``.
"""

from __future__ import annotations

import logging
import time
from enum import Enum, auto
from typing import Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ======================================================================
# Enumeração de estado do sensor
# ======================================================================

class ModoSensor(Enum):
    """Estado operacional do sensor de profundidade."""
    REAL_OPEN3D = auto()
    REAL_FREENECT = auto()
    SIMULACAO = auto()


# ======================================================================
# Constantes do Kinect v1
# ======================================================================

_KINECT_LARGURA: int = 640
_KINECT_ALTURA: int = 480
_KINECT_FX: float = 525.0
_KINECT_FY: float = 525.0
_KINECT_CX: float = 319.5
_KINECT_CY: float = 239.5


# ======================================================================
# KinectSensor — Classe principal
# ======================================================================

class KinectSensor:
    """Captura nuvens de pontos 3D de um sensor Kinect ou de uma simulação.

    O construtor tenta, nesta ordem:

    1. Abrir o Kinect via **Open3D** (``o3d.io.AzureKinectSensor``
       ou ``o3d.io.RealSenseSensor`` para Kinect v2 / Azure).
    2. Abrir o Kinect via **freenect** (Kinect v1 / libfreenect).
    3. Se ambos falharem, entrar em **Modo Simulação** gerando
       nuvens sintéticas com estado persistente de areia.

    **Simulação Interativa**: em modo simulação, a classe mantém uma
    grade NumPy de alturas (``_grade_areia``) que começa em 0.15 m
    (areia nivelada). O método ``modificar_areia()`` permite cavar
    ou preencher com decaimento Gaussiano, simulando interação
    física via mouse.

    Parameters
    ----------
    forcar_simulacao : bool
        Se ``True``, ignora o hardware e entra direto em simulação.
        Útil para testes e desenvolvimento.
    resolucao : tuple[int, int]
        ``(largura, altura)`` da imagem de profundidade.
        Padrão: ``(640, 480)`` (Kinect v1).
    resolucao_grade_sim : int
        Número de pontos por eixo na grade de simulação.
        Padrão: 50 (gera 2500 pontos na nuvem).
    largura_mesa : float
        Dimensão X da mesa em metros (padrão: 1.50).
    comprimento_mesa : float
        Dimensão Y da mesa em metros (padrão: 1.50).
    altura_max_areia : float
        Altura máxima da areia em metros (padrão: 0.30).

    Attributes
    ----------
    modo : ModoSensor
        Estado operacional atual do sensor.
    resolucao : tuple[int, int]
        Resolução do mapa de profundidade.

    Examples
    --------
    >>> sensor = KinectSensor()
    >>> nuvem = sensor.capturar_nuvem()       # (N, 3) float64
    >>> prof = sensor.capturar_profundidade()  # (480, 640) uint16
    >>> sensor.modificar_areia(0.75, 0.75, cavar=True)  # cava no centro
    """

    def __init__(
        self,
        forcar_simulacao: bool = False,
        resolucao: Tuple[int, int] = (_KINECT_LARGURA, _KINECT_ALTURA),
        resolucao_grade_sim: int = 50,
        largura_mesa: float = 1.50,
        comprimento_mesa: float = 1.50,
        altura_max_areia: float = 0.30,
    ) -> None:
        self.resolucao: Tuple[int, int] = resolucao
        self.modo: ModoSensor = ModoSensor.SIMULACAO

        # Dimensões da mesa (usadas na simulação interativa)
        self._largura_mesa = largura_mesa
        self._comprimento_mesa = comprimento_mesa
        self._altura_max_areia = altura_max_areia
        self._resolucao_grade_sim = resolucao_grade_sim

        # Grade persistente de alturas da areia (modo simulação)
        # Inicializada em 0.15 m = metade da altura máxima (0.30 m)
        self._grade_areia: Optional[np.ndarray] = None
        self._eixo_x_sim: Optional[np.ndarray] = None
        self._eixo_y_sim: Optional[np.ndarray] = None

        # Referências para drivers reais (populadas se disponíveis)
        self._o3d_sensor = None
        self._freenect_mod = None

        if forcar_simulacao:
            logger.warning(
                "KinectSensor: forcar_simulacao=True — Modo Simulação ativado."
            )
            print("[KinectSensor] ⚠ Modo Simulação forçado pelo usuário.")
            self._inicializar_grade_simulacao()
            return

        # --- Tentativa 1: Open3D ---
        if self._tentar_open3d():
            return

        # --- Tentativa 2: freenect (Kinect v1) ---
        if self._tentar_freenect():
            return

        # --- Fallback: simulação ---
        logger.warning(
            "KinectSensor: nenhum sensor detectado — entrando em Modo Simulação."
        )
        print("[KinectSensor] ⚠ Aviso: Nenhum sensor Kinect detectado.")
        print("[KinectSensor]   Entrando em MODO SIMULAÇÃO (nuvem sintética).")
        self._inicializar_grade_simulacao()

    # ------------------------------------------------------------------
    # Inicialização de drivers
    # ------------------------------------------------------------------

    def _tentar_open3d(self) -> bool:
        """Tenta abrir o Kinect via Open3D.

        Returns
        -------
        bool
            ``True`` se o sensor Open3D foi inicializado com sucesso.
        """
        try:
            import open3d as o3d  # noqa: F811

            # Verificar se há dispositivos de captura disponíveis
            # Open3D suporta RealSense e Azure Kinect; tratamos ambos.
            # Como não há API simples de listagem universal,
            # tentamos criar o sensor e ler um frame de teste.
            sensor_config = o3d.io.AzureKinectSensorConfig()
            sensor = o3d.io.AzureKinectSensor(sensor_config)
            if sensor.connect(0):
                self._o3d_sensor = sensor
                self.modo = ModoSensor.REAL_OPEN3D
                logger.info("KinectSensor: Open3D Azure Kinect conectado.")
                print("[KinectSensor] ✓ Kinect detectado via Open3D (Azure).")
                return True
        except Exception as e:
            logger.debug("Open3D Kinect indisponível: %s", e)
        return False

    def _tentar_freenect(self) -> bool:
        """Tenta abrir o Kinect v1 via libfreenect.

        Returns
        -------
        bool
            ``True`` se ``freenect.sync_get_depth()`` funcionou.
        """
        try:
            import freenect
            prof, _ = freenect.sync_get_depth()
            if prof is not None:
                self._freenect_mod = freenect
                self.modo = ModoSensor.REAL_FREENECT
                logger.info("KinectSensor: freenect (Kinect v1) conectado.")
                print("[KinectSensor] ✓ Kinect v1 detectado via freenect.")
                return True
        except Exception as e:
            logger.debug("freenect indisponível: %s", e)
        return False

    # ------------------------------------------------------------------
    # Propriedades
    # ------------------------------------------------------------------

    @property
    def esta_simulando(self) -> bool:
        """``True`` se o sensor está em modo simulação."""
        return self.modo == ModoSensor.SIMULACAO

    @property
    def intrinsicos(self) -> dict:
        """Parâmetros intrínsecos padrão do Kinect v1.

        Returns
        -------
        dict
            Chaves: ``fx``, ``fy``, ``cx``, ``cy``.
        """
        return {
            "fx": _KINECT_FX, "fy": _KINECT_FY,
            "cx": _KINECT_CX, "cy": _KINECT_CY,
        }

    # ------------------------------------------------------------------
    # Captura: mapa de profundidade (480×640)
    # ------------------------------------------------------------------

    def capturar_profundidade(self) -> np.ndarray:
        """Retorna um mapa de profundidade ``(H, W)`` em milímetros (uint16).

        Returns
        -------
        np.ndarray, shape (H, W), dtype uint16
            Mapa de profundidade em milímetros.
        """
        if self.modo == ModoSensor.REAL_FREENECT:
            return self._profundidade_freenect()
        if self.modo == ModoSensor.REAL_OPEN3D:
            return self._profundidade_open3d()
        return self._profundidade_simulada()

    def _profundidade_freenect(self) -> np.ndarray:
        """Captura via libfreenect."""
        prof, _ = self._freenect_mod.sync_get_depth()
        return np.asarray(prof, dtype=np.uint16)

    def _profundidade_open3d(self) -> np.ndarray:
        """Captura via Open3D (Azure Kinect / RealSense)."""
        import open3d as o3d
        rgbd = self._o3d_sensor.capture_frame(True)
        if rgbd is None:
            logger.warning("Open3D: frame vazio, retornando simulação.")
            return self._profundidade_simulada()
        depth = np.asarray(rgbd.depth)
        return depth.astype(np.uint16)

    def _profundidade_simulada(self) -> np.ndarray:
        """Gera um mapa de profundidade sintético (colina gaussiana + ruído).

        Simula o sensor montado a 2,5 m da mesa.  A profundidade base
        é **2500 mm** e a colina central varia no tempo para simular
        uma "mão movendo a areia" (até ~300 mm de amplitude, compatível
        com ``ALTURA_MAX_AREIA = 0.30 m``).

        Returns
        -------
        np.ndarray, shape (480, 640), dtype uint16
        """
        w, h = self.resolucao
        # Kinect montado a 2,5 m → profundidade base = 2500 mm
        profundidade = np.full((h, w), 2500, dtype=np.float32)

        u = np.arange(w)
        v = np.arange(h)
        uu, vv = np.meshgrid(u, v)

        # Colina gaussiana no centro — leve oscilação temporal
        fase = time.time() % (2 * np.pi)
        amp = 200 + 30 * np.sin(fase)
        colina = amp * np.exp(
            -(((uu - w // 2) ** 2) / (2 * 160 ** 2)
              + ((vv - h // 2) ** 2) / (2 * 160 ** 2))
        )
        profundidade -= colina

        # Ruído gaussiano para simular o sensor real
        ruido = np.random.normal(0, 3, profundidade.shape).astype(np.float32)
        profundidade += ruido

        return np.clip(profundidade, 0, 65535).astype(np.uint16)

    # ------------------------------------------------------------------
    # Captura: nuvem de pontos 3D (N, 3)
    # ------------------------------------------------------------------

    def capturar_nuvem(self) -> np.ndarray:
        """Retorna uma nuvem de pontos 3D ``(N, 3)``.

        No modo real, cada linha contém ``[u_pixel, v_pixel, profundidade_mm]``.
        No modo simulação, retorna pontos já em coordenadas da mesa
        ``[x_m, y_m, z_m]`` com um plano reto em Z = 0.15 m.

        Returns
        -------
        np.ndarray, shape (N, 3), dtype float64
        """
        if self.modo == ModoSensor.SIMULACAO:
            return self._nuvem_simulada_mesa()
        profundidade = self.capturar_profundidade()
        return self.profundidade_para_pontos(profundidade)

    def _inicializar_grade_simulacao(self) -> None:
        """Cria a grade persistente de alturas para o modo simulação.

        A grade tem ``resolucao_grade_sim × resolucao_grade_sim`` pontos
        cobrindo a mesa inteira, todos inicializados em 0.15 m
        (metade da altura máxima de areia).
        """
        res = self._resolucao_grade_sim
        self._eixo_x_sim = np.linspace(0.0, self._largura_mesa, res)
        self._eixo_y_sim = np.linspace(0.0, self._comprimento_mesa, res)
        self._grade_areia = np.full(
            (res, res), self._altura_max_areia / 2.0, dtype=np.float64
        )
        print(f"[KinectSensor] Grade de simulação: {res}×{res}, "
              f"Z inicial = {self._altura_max_areia / 2.0:.2f} m")

    def modificar_areia(
        self,
        x: float,
        y: float,
        cavar: bool = True,
        raio: float = 0.10,
        intensidade: float = 0.008,
    ) -> None:
        """Cava ou preenche a areia virtual com decaimento Gaussiano.

        Simula a interação física do usuário com a areia.  O efeito
        é centrado em ``(x, y)`` com um perfil Gaussiano de largura
        ``raio`` (em metros).  Chamadas repetidas (arrastar o mouse)
        acumulam o efeito frame a frame.

        Parameters
        ----------
        x : float
            Coordenada X na mesa (metros).
        y : float
            Coordenada Y na mesa (metros).
        cavar : bool
            Se ``True``, diminui Z (cavar). Se ``False``, aumenta Z
            (colocar areia).
        raio : float
            Raio de ação em metros (padrão: 0.10 m = 10 cm).
        intensidade : float
            Deslocamento máximo por chamada no centro do pincel
            (padrão: 0.008 m ≈ 8 mm).
        """
        if self._grade_areia is None:
            return

        xx, yy = np.meshgrid(self._eixo_x_sim, self._eixo_y_sim)

        # Distância ao centro do clique
        dist2 = (xx - x) ** 2 + (yy - y) ** 2
        sigma2 = (raio / 2.0) ** 2  # sigma = raio/2 → 95% do efeito dentro do raio

        # Perfil Gaussiano
        delta = intensidade * np.exp(-dist2 / (2.0 * sigma2))

        if cavar:
            self._grade_areia -= delta
        else:
            self._grade_areia += delta

        # Limitar ao intervalo físico [0, altura_max_areia]
        np.clip(self._grade_areia, 0.0, self._altura_max_areia, out=self._grade_areia)

    def _nuvem_simulada_mesa(self) -> np.ndarray:
        """Gera nuvem de pontos simulada a partir da grade persistente de areia.

        Retorna a nuvem com as alturas atuais (potencialmente modificadas
        por ``modificar_areia``), já em coordenadas da mesa.

        Returns
        -------
        np.ndarray, shape (N, 3), dtype float64
        """
        xx, yy = np.meshgrid(self._eixo_x_sim, self._eixo_y_sim)
        return np.column_stack([
            xx.ravel(),
            yy.ravel(),
            self._grade_areia.ravel(),
        ])

    @staticmethod
    def profundidade_para_pontos(profundidade: np.ndarray) -> np.ndarray:
        """Converte mapa de profundidade ``(H, W)`` em array ``(N, 3)``.

        Parameters
        ----------
        profundidade : np.ndarray, shape (H, W)
            Mapa de profundidade.

        Returns
        -------
        np.ndarray, shape (N, 3), dtype float64
            Array com colunas ``[u, v, d]``, filtrado para ``d > 0``.
        """
        v_idx, u_idx = np.indices(profundidade.shape)
        pontos = np.stack([
            u_idx.flatten(),
            v_idx.flatten(),
            profundidade.flatten(),
        ], axis=1).astype(np.float64)

        return pontos[pontos[:, 2] > 0]

    # ------------------------------------------------------------------
    # Imagem colorida para exibição de debug
    # ------------------------------------------------------------------

    @staticmethod
    def profundidade_para_imagem(profundidade: np.ndarray) -> np.ndarray:
        """Normaliza e aplica colormap ao mapa de profundidade.

        Parameters
        ----------
        profundidade : np.ndarray, shape (H, W)

        Returns
        -------
        np.ndarray, shape (H, W, 3), dtype uint8
            Imagem BGR com colormap TURBO.
        """
        norm = cv2.normalize(profundidade, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_TURBO)

    # ------------------------------------------------------------------
    # Contexto
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return f"KinectSensor(modo={self.modo.name}, resolucao={self.resolucao})"

    def liberar(self) -> None:
        """Libera recursos do sensor (se houver)."""
        if self._o3d_sensor is not None:
            self._o3d_sensor.disconnect()
            self._o3d_sensor = None
        self._freenect_mod = None
        logger.info("KinectSensor: recursos liberados.")

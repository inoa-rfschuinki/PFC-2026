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
       nuvens sintéticas (colina gaussiana + ruído).

    Parameters
    ----------
    forcar_simulacao : bool
        Se ``True``, ignora o hardware e entra direto em simulação.
        Útil para testes e desenvolvimento.
    resolucao : tuple[int, int]
        ``(largura, altura)`` da imagem de profundidade.
        Padrão: ``(640, 480)`` (Kinect v1).

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
    """

    def __init__(
        self,
        forcar_simulacao: bool = False,
        resolucao: Tuple[int, int] = (_KINECT_LARGURA, _KINECT_ALTURA),
    ) -> None:
        self.resolucao: Tuple[int, int] = resolucao
        self.modo: ModoSensor = ModoSensor.SIMULACAO

        # Referências para drivers reais (populadas se disponíveis)
        self._o3d_sensor = None
        self._freenect_mod = None

        if forcar_simulacao:
            logger.warning(
                "KinectSensor: forcar_simulacao=True — Modo Simulação ativado."
            )
            print("[KinectSensor] ⚠ Modo Simulação forçado pelo usuário.")
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

        Reproduz o formato que o Kinect v1 entregaria: ``(480, 640)`` uint16
        com valores em milímetros.  A colina central varia no tempo para
        simular uma "mão movendo a areia".

        Returns
        -------
        np.ndarray, shape (480, 640), dtype uint16
        """
        w, h = self.resolucao
        profundidade = np.full((h, w), 1000, dtype=np.float32)

        u = np.arange(w)
        v = np.arange(h)
        uu, vv = np.meshgrid(u, v)

        # Colina gaussiana no centro — leve oscilação temporal
        fase = time.time() % (2 * np.pi)
        amp = 150 + 20 * np.sin(fase)
        colina = amp * np.exp(
            -(((uu - w // 2) ** 2) / (2 * 120 ** 2)
              + ((vv - h // 2) ** 2) / (2 * 90 ** 2))
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
        """Retorna uma nuvem de pontos 3D ``(N, 3)`` em milímetros.

        Cada linha contém ``[u_pixel, v_pixel, profundidade_mm]``.
        Pontos com profundidade zero são descartados.

        Returns
        -------
        np.ndarray, shape (N, 3), dtype float64
            Coordenadas ``[u, v, d]`` dos pontos válidos.
        """
        profundidade = self.capturar_profundidade()
        return self.profundidade_para_pontos(profundidade)

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

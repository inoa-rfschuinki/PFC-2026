"""
mde_cartografia.py — Adaptador de MDE com Fallback Resiliente
==============================================================
Projeto Final de Curso — Engenharia de Computação (AMAN, 2026)

Lê um Modelo Digital de Elevação no formato GeoTIFF (.tif) fornecido
pela equipe de Cartografia, normaliza as elevações para a escala
física da caixa de areia e expõe consultas pontuais interpoladas.

**Fallback Resiliente**: se o arquivo GeoTIFF não existir, as
dependências ``rasterio``/``scipy`` não estiverem instaladas, ou
ocorrer qualquer erro de leitura, o adaptador gera automaticamente
uma **superfície sintética** (cosseno 2D) para que a demonstração
nunca pare.

Dependências externas (opcionais)::

    pip install rasterio scipy numpy

Uso típico::

    mde = AdaptadorMDE("terreno_aman.tif")
    z = mde.obter_z_alvo(0.45, 0.30)
"""

from __future__ import annotations

import os
import numpy as np
from typing import Optional, Tuple

try:
    import rasterio
except ImportError:
    rasterio = None  # type: ignore[assignment]

try:
    from scipy.interpolate import RegularGridInterpolator
except ImportError:
    RegularGridInterpolator = None  # type: ignore[assignment, misc]


class AdaptadorMDE:
    """Lê um GeoTIFF e fornece elevações normalizadas para a caixa de areia.

    A classe executa três etapas na construção:

    1. **Leitura** — abre o GeoTIFF com ``rasterio`` e extrai a primeira
       banda como matriz de elevações (linhas × colunas).
    2. **Normalização Z** — mapeia o intervalo real de elevações
       ``[z_min, z_max]`` do terreno para ``[0, altura_max_areia]``.
    3. **Mapeamento XY** — cria dois eixos lineares que vão de
       ``0`` a ``largura_mesa`` (X) e ``0`` a ``comprimento_mesa`` (Y),
       um por coluna e um por linha da grade.

    A consulta ``obter_z_alvo(x, y)`` usa
    ``scipy.interpolate.RegularGridInterpolator`` para retornar valores
    suaves (interpolação bilinear) em qualquer coordenada dentro da mesa.

    Parameters
    ----------
    caminho_geotiff : str
        Caminho para o arquivo ``.tif`` do MDE.
    largura_mesa : float
        Dimensão X da caixa de areia, em metros.
    comprimento_mesa : float
        Dimensão Y da caixa de areia, em metros.
    altura_max_areia : float
        Espessura máxima de areia mapeável, em metros.
    """

    def __init__(
        self,
        caminho_geotiff: str = "",
        largura_mesa: float = 1.50,
        comprimento_mesa: float = 1.50,
        altura_max_areia: float = 0.30,
    ) -> None:
        # Dimensões físicas da mesa
        self._largura_mesa = largura_mesa
        self._comprimento_mesa = comprimento_mesa
        self._altura_max_areia = altura_max_areia

        # Estado interno
        self._grade_normalizada: Optional[np.ndarray] = None
        self._eixo_x: Optional[np.ndarray] = None
        self._eixo_y: Optional[np.ndarray] = None
        self._interpolador: Optional[RegularGridInterpolator] = None
        self._carregado: bool = False
        self._usando_sintetico: bool = False

        # Metadados do GeoTIFF
        self._z_min_original: float = 0.0
        self._z_max_original: float = 0.0
        self._resolucao_geotiff: Optional[Tuple[float, float]] = None

        # Tentar carregar o GeoTIFF; se falhar, gerar superfície sintética
        if caminho_geotiff:
            try:
                self._carregar(caminho_geotiff)
            except (ImportError, FileNotFoundError, Exception) as e:
                print(f"[MDE] ⚠ Falha ao carregar GeoTIFF: {e}")
                print("[MDE]   Gerando superfície sintética para demonstração.")
                self._gerar_superficie_sintetica()
        else:
            print("[MDE] Nenhum caminho fornecido — usando superfície sintética.")
            self._gerar_superficie_sintetica()

    # ------------------------------------------------------------------ #
    # Leitura e normalização
    # ------------------------------------------------------------------ #

    def _carregar(self, caminho: str) -> None:
        """Lê o GeoTIFF, normaliza e constrói o interpolador."""

        if rasterio is None:
            raise ImportError(
                "O pacote 'rasterio' é necessário para ler GeoTIFF. "
                "Instale com: pip install rasterio"
            )
        if RegularGridInterpolator is None:
            raise ImportError(
                "O pacote 'scipy' é necessário para interpolação. "
                "Instale com: pip install scipy"
            )

        if not os.path.isfile(caminho):
            raise FileNotFoundError(
                f"Arquivo GeoTIFF não encontrado: {caminho}"
            )

        # 1. Leitura da primeira banda
        with rasterio.open(caminho) as src:
            elevacoes = src.read(1).astype(np.float64)  # (linhas, colunas)
            self._resolucao_geotiff = src.res            # (res_x, res_y)
            nodata = src.nodata

        # Tratar pixels sem dado substituindo por NaN e depois pela mediana
        if nodata is not None:
            mascara_nodata = elevacoes == nodata
            if mascara_nodata.any():
                mediana = float(np.nanmedian(elevacoes[~mascara_nodata]))
                elevacoes[mascara_nodata] = mediana

        # 2. Normalização Z: [z_min, z_max] → [0, altura_max_areia]
        self._z_min_original = float(np.nanmin(elevacoes))
        self._z_max_original = float(np.nanmax(elevacoes))

        amplitude = self._z_max_original - self._z_min_original
        if amplitude < 1e-12:
            # Terreno plano — normalizar para metade da altura
            self._grade_normalizada = np.full_like(
                elevacoes, self._altura_max_areia / 2.0
            )
        else:
            self._grade_normalizada = (
                (elevacoes - self._z_min_original) / amplitude
            ) * self._altura_max_areia

        # 3. Mapeamento XY: criar eixos em metros
        n_linhas, n_colunas = self._grade_normalizada.shape
        self._eixo_y = np.linspace(0.0, self._comprimento_mesa, n_linhas)
        self._eixo_x = np.linspace(0.0, self._largura_mesa, n_colunas)

        # 4. Construir interpolador bilinear
        #    RegularGridInterpolator espera (eixo_y, eixo_x) porque a
        #    grade tem shape (n_linhas, n_colunas) = (len(eixo_y), len(eixo_x))
        self._interpolador = RegularGridInterpolator(
            (self._eixo_y, self._eixo_x),
            self._grade_normalizada,
            method="linear",
            bounds_error=False,
            fill_value=None,  # extrapola pela borda mais próxima
        )

        self._carregado = True
        print(f"[MDE Cartografia] GeoTIFF carregado: {caminho}")
        print(f"[MDE Cartografia] Grade: {n_linhas}×{n_colunas}")
        print(f"[MDE Cartografia] Elevação original: "
              f"{self._z_min_original:.1f} m → {self._z_max_original:.1f} m")
        print(f"[MDE Cartografia] Normalizado para: "
              f"0.000 m → {self._altura_max_areia:.3f} m")
        print(f"[MDE Cartografia] Mesa: {self._largura_mesa:.2f} m × "
              f"{self._comprimento_mesa:.2f} m")

    # ------------------------------------------------------------------ #
    # Superfície sintética (fallback)
    # ------------------------------------------------------------------ #

    def _gerar_superficie_sintetica(self, resolucao: int = 100) -> None:
        """Gera uma superfície matemática cosseno 2D como fallback.

        A superfície produzida é:

        .. math::

            z(x, y) = \\frac{h}{2} \\left(1 + \\cos\\!\\left(
                \\frac{2\\pi\\,x}{L_x}\\right) \\cdot
                \\cos\\!\\left(\\frac{2\\pi\\,y}{L_y}\\right)\\right)

        Isso gera uma "colina" suave no centro da mesa que vai de 0 a
        ``altura_max_areia``, ideal para demonstrar a coloração RGB
        na apresentação da banca.

        Parameters
        ----------
        resolucao : int
            Número de pontos por eixo na grade sintética.
        """
        self._eixo_x = np.linspace(0.0, self._largura_mesa, resolucao)
        self._eixo_y = np.linspace(0.0, self._comprimento_mesa, resolucao)
        xx, yy = np.meshgrid(self._eixo_x, self._eixo_y)

        h = self._altura_max_areia
        self._grade_normalizada = (h / 2.0) * (
            1.0
            + np.cos(2 * np.pi * xx / self._largura_mesa)
            * np.cos(2 * np.pi * yy / self._comprimento_mesa)
        )

        if RegularGridInterpolator is not None:
            self._interpolador = RegularGridInterpolator(
                (self._eixo_y, self._eixo_x),
                self._grade_normalizada,
                method="linear",
                bounds_error=False,
                fill_value=None,
            )

        self._z_min_original = 0.0
        self._z_max_original = h
        self._carregado = True
        self._usando_sintetico = True
        print(f"[MDE] Superfície sintética gerada: {resolucao}×{resolucao}")
        print(f"[MDE] Z range: 0.000 m → {h:.3f} m")

    # ------------------------------------------------------------------ #
    # Consulta pontual
    # ------------------------------------------------------------------ #

    def obter_z_alvo(self, x: float, y: float) -> float:
        """Retorna a altura normalizada que a areia deve ter em (x, y).

        Usa interpolação bilinear via ``RegularGridInterpolator``.
        Pontos fora da mesa são clamped à borda mais próxima.

        Este método é compatível com a assinatura esperada por
        ``gerar_mapa_cores(funcao_mde=mde.obter_z_alvo)``.

        Parameters
        ----------
        x : float
            Coordenada X na mesa (metros), de 0 a ``largura_mesa``.
        y : float
            Coordenada Y na mesa (metros), de 0 a ``comprimento_mesa``.

        Returns
        -------
        z_alvo : float
            Altura normalizada em metros, de 0 a ``altura_max_areia``.
        """
        if not self._carregado or self._grade_normalizada is None:
            return 0.0

        # Clamp para os limites da mesa
        x_c = max(0.0, min(x, self._largura_mesa))
        y_c = max(0.0, min(y, self._comprimento_mesa))

        # Caminho rápido: interpolador disponível
        if self._interpolador is not None:
            return float(self._interpolador((y_c, x_c)))

        # Fallback: nearest-neighbor (scipy não instalado)
        n_lin, n_col = self._grade_normalizada.shape
        col = int(round(x_c / self._largura_mesa * (n_col - 1)))
        lin = int(round(y_c / self._comprimento_mesa * (n_lin - 1)))
        col = max(0, min(col, n_col - 1))
        lin = max(0, min(lin, n_lin - 1))
        return float(self._grade_normalizada[lin, col])

    # ------------------------------------------------------------------ #
    # Visualização — heatmap do MDE
    # ------------------------------------------------------------------ #

    def gerar_imagem_visualizacao(
        self,
        largura: int = 480,
        altura: int = 480,
        colormap: int = 4,  # cv2.COLORMAP_TURBO = 20, COLORMAP_JET = 2, COLORMAP_INFERNO = 11
    ) -> np.ndarray:
        """Gera uma imagem BGR com heatmap da grade de elevações normalizada.

        Se o MDE não estiver carregado, retorna uma imagem preta com
        texto informativo.

        Parameters
        ----------
        largura : int
            Largura da imagem de saída em pixels.
        altura : int
            Altura da imagem de saída em pixels.
        colormap : int
            Código ``cv2.COLORMAP_*``.  Padrão: ``cv2.COLORMAP_TURBO`` (20).

        Returns
        -------
        np.ndarray, shape (altura, largura, 3), dtype uint8
            Imagem BGR com o heatmap do MDE.
        """
        import cv2

        if not self._carregado or self._grade_normalizada is None:
            img = np.zeros((altura, largura, 3), dtype=np.uint8)
            cv2.putText(
                img, "MDE nao carregado", (10, altura // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
            )
            return img

        # Normalizar grade para [0, 255]
        grade = self._grade_normalizada.astype(np.float64)
        g_min, g_max = grade.min(), grade.max()
        if g_max - g_min < 1e-12:
            norm = np.full_like(grade, 128, dtype=np.uint8)
        else:
            norm = ((grade - g_min) / (g_max - g_min) * 255).astype(np.uint8)

        # Aplicar colormap
        heatmap = cv2.applyColorMap(norm, colormap)

        # Redimensionar para a resolução desejada
        heatmap = cv2.resize(heatmap, (largura, altura), interpolation=cv2.INTER_LINEAR)

        # Adicionar barra de escala com rótulos
        fonte = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            heatmap,
            f"Z: {self._z_min_original:.1f} m",
            (5, altura - 10),
            fonte, 0.4, (255, 255, 255), 1,
        )
        cv2.putText(
            heatmap,
            f"Z: {self._z_max_original:.1f} m",
            (5, 20),
            fonte, 0.4, (255, 255, 255), 1,
        )

        tipo = "Sintetico" if self._usando_sintetico else "GeoTIFF"
        cv2.putText(
            heatmap,
            f"MDE ({tipo})",
            (largura - 160, 20),
            fonte, 0.4, (255, 255, 255), 1,
        )

        return heatmap

    # ------------------------------------------------------------------ #
    # Utilitários
    # ------------------------------------------------------------------ #

    @property
    def esta_carregado(self) -> bool:
        """Indica se o MDE foi carregado (real ou sintético)."""
        return self._carregado

    @property
    def usando_sintetico(self) -> bool:
        """``True`` se estiver usando superfície sintética (fallback)."""
        return self._usando_sintetico

    @property
    def dimensoes_mesa(self) -> Tuple[float, float, float]:
        """Retorna (largura_X, comprimento_Y, altura_Z) da mesa em metros."""
        return (self._largura_mesa, self._comprimento_mesa, self._altura_max_areia)

    @property
    def elevacao_original(self) -> Tuple[float, float]:
        """Retorna (z_min, z_max) das elevações originais do GeoTIFF em metros."""
        return (self._z_min_original, self._z_max_original)

    @property
    def shape_grade(self) -> Optional[Tuple[int, int]]:
        """Retorna (linhas, colunas) da grade normalizada, ou None."""
        if self._grade_normalizada is not None:
            return self._grade_normalizada.shape
        return None

    def __repr__(self) -> str:
        if self._carregado and self._grade_normalizada is not None:
            h, w = self._grade_normalizada.shape
            return (
                f"AdaptadorMDE(grade={h}×{w}, "
                f"mesa={self._largura_mesa:.2f}×{self._comprimento_mesa:.2f} m, "
                f"z_max={self._altura_max_areia:.3f} m)"
            )
        return "AdaptadorMDE(não carregado)"

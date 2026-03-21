"""
mde_cartografia.py — Adaptador de MDE Real via GeoTIFF (Cartografia)
=====================================================================
Projeto Final de Curso — Engenharia de Computação (2026)

Lê um Modelo Digital de Elevação no formato GeoTIFF (.tif) fornecido
pela equipe de Cartografia, normaliza as elevações para a escala
física da caixa de areia e expõe consultas pontuais interpoladas.

Dependências externas:
    pip install rasterio scipy numpy

Uso típico::

    mde = AdaptadorMDE(
        caminho_geotiff="mapa_terreno.tif",
        largura_mesa=0.90,      # metros (X)
        comprimento_mesa=0.60,  # metros (Y)
        altura_max_areia=0.20,  # metros (Z)
    )
    z = mde.obter_z_alvo(0.45, 0.30)  # altura da areia em (x, y)
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
        caminho_geotiff: str,
        largura_mesa: float = 0.90,
        comprimento_mesa: float = 0.60,
        altura_max_areia: float = 0.20,
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

        # Metadados do GeoTIFF
        self._z_min_original: float = 0.0
        self._z_max_original: float = 0.0
        self._resolucao_geotiff: Optional[Tuple[float, float]] = None

        # Carregar automaticamente
        self._carregar(caminho_geotiff)

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
        if not self._carregado or self._interpolador is None:
            return 0.0

        # Clamp para os limites da mesa
        x_c = max(0.0, min(x, self._largura_mesa))
        y_c = max(0.0, min(y, self._comprimento_mesa))

        # O interpolador recebe (y, x) porque a grade é (linhas, colunas)
        return float(self._interpolador((y_c, x_c)))

    # ------------------------------------------------------------------ #
    # Utilitários
    # ------------------------------------------------------------------ #

    @property
    def esta_carregado(self) -> bool:
        """Indica se o GeoTIFF foi carregado com sucesso."""
        return self._carregado

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

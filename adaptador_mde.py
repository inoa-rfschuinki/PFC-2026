"""
adaptador_mde.py — Interface com o Modelo Digital de Elevação (Cartografia)
============================================================================
Projeto Final de Curso — Engenharia de Computação (2026)

Este módulo encapsula toda a comunicação com o MDE fornecido pela equipe
de Cartografia.  Quando o arquivo real (GeoTIFF, matriz NumPy, etc.)
for entregue, basta alterar os métodos internos desta classe — nenhum
outro arquivo do projeto precisa ser modificado.

╔══════════════════════════════════════════════════════════════════════╗
║  PONTO DE INTEGRAÇÃO COM A CARTOGRAFIA                             ║
║  • O método `carregar_mapa()` deve ler o formato final (GeoTIFF,   ║
║    .npy, .csv, etc.) e armazenar a grade em `self._grade`.         ║
║  • O método `obter_z_alvo(x, y)` deve interpolar a grade para      ║
║    retornar a elevação esperada nas coordenadas da mesa.            ║
║  • Enquanto o mapa não chegar, o mock interno é usado.              ║
╚══════════════════════════════════════════════════════════════════════╝
"""

from __future__ import annotations

import numpy as np
from typing import Optional


class AdaptadorMDE:
    """Adaptador que isola o restante do sistema do formato do MDE.

    Uso típico::

        mde = AdaptadorMDE()
        mde.carregar_mapa("terreno_final.tif")   # quando disponível
        z = mde.obter_z_alvo(x, y)               # consulta pontual
    """

    def __init__(self) -> None:
        self._grade: Optional[np.ndarray] = None
        self._origem_x: float = 0.0
        self._origem_y: float = 0.0
        self._resolucao: float = 1.0   # metros por pixel da grade
        self._carregado: bool = False

    # ------------------------------------------------------------------ #
    # Carga do mapa
    # ------------------------------------------------------------------ #

    def carregar_mapa(self, filepath: str) -> None:
        """Carrega o MDE a partir de um arquivo.

        ┌──────────────────────────────────────────────────────┐
        │  TODO — CARTOGRAFIA                                  │
        │  Substituir o corpo deste método pela leitura real:  │
        │    • GeoTIFF  → rasterio / gdal                     │
        │    • .npy     → np.load(filepath)                    │
        │    • .csv     → np.loadtxt(filepath, delimiter=',')  │
        │  Preencher self._grade, self._origem_x/y,            │
        │  self._resolucao conforme metadados do arquivo.      │
        └──────────────────────────────────────────────────────┘

        Parameters
        ----------
        filepath : str
            Caminho para o arquivo do MDE.
        """
        print(f"[AdaptadorMDE] Aguardando arquivo da Cartografia.")
        print(f"[AdaptadorMDE] Caminho recebido: {filepath}")
        print(f"[AdaptadorMDE] Usando mock interno até a entrega do mapa real.")

        # --- Mock: grade sintética 100×100 com uma rampa suave ----------
        tamanho = 100
        x = np.linspace(0, tamanho - 1, tamanho)
        y = np.linspace(0, tamanho - 1, tamanho)
        xx, yy = np.meshgrid(x, y)
        self._grade = 10.0 + 0.2 * xx + 0.1 * yy     # rampa linear
        self._origem_x = 0.0
        self._origem_y = 0.0
        self._resolucao = 1.0
        self._carregado = True

    # ------------------------------------------------------------------ #
    # Consulta pontual
    # ------------------------------------------------------------------ #

    def obter_z_alvo(self, x: float, y: float) -> float:
        """Retorna a elevação Z esperada pelo MDE nas coordenadas (x, y).

        Se o mapa ainda não foi carregado, usa um mock plano em z = 0.
        Se (x, y) cai fora da grade carregada, retorna o valor da borda
        mais próxima (clamping).

        ┌──────────────────────────────────────────────────────┐
        │  INTEGRAÇÃO CARTOGRAFIA — este é o método que será   │
        │  passado como `funcao_mde` para `gerar_mapa_cores`.  │
        │  Ex.:                                                │
        │    cores = gerar_mapa_cores(                         │
        │        pontos_mesa,                                  │
        │        funcao_mde=mde.obter_z_alvo,                  │
        │        tolerancia=5.0,                               │
        │    )                                                 │
        └──────────────────────────────────────────────────────┘

        Parameters
        ----------
        x, y : float
            Coordenadas no referencial da mesa.

        Returns
        -------
        z_alvo : float
        """
        if not self._carregado or self._grade is None:
            # Fallback: superfície plana em z = 0
            return 0.0

        # Converter coordenadas da mesa → índices da grade
        col = int(round((x - self._origem_x) / self._resolucao))
        lin = int(round((y - self._origem_y) / self._resolucao))

        # Clamping para não sair da grade
        lin = max(0, min(lin, self._grade.shape[0] - 1))
        col = max(0, min(col, self._grade.shape[1] - 1))

        return float(self._grade[lin, col])

    # ------------------------------------------------------------------ #
    # Utilitários
    # ------------------------------------------------------------------ #

    @property
    def esta_carregado(self) -> bool:
        return self._carregado

    def __repr__(self) -> str:
        if self._carregado and self._grade is not None:
            h, w = self._grade.shape
            return f"AdaptadorMDE(carregado=True, grade={h}x{w})"
        return "AdaptadorMDE(carregado=False, usando mock z=0)"

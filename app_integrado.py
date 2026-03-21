"""
app_integrado.py — Pipeline Completo do Caixão de Areia (AR Sandbox)
=====================================================================
Projeto Final de Curso — Engenharia de Computação (2026)

Orquestra o pipeline contínuo unindo:
  • Captura de profundidade  (kinect.py — Raquel)
  • Exibição no projetor     (exibicao.py — Raquel)
  • Utilitários              (utils.py — Raquel)
  • Motor matemático         (motor_caixao_areia.py — Rafael)
  • Adaptador MDE            (adaptador_mde.py — Rafael / Cartografia)

Nenhum arquivo da Raquel é alterado. Este módulo apenas importa e
orquestra as funções já existentes.

╔══════════════════════════════════════════════════════════════════════╗
║  STATUS DE INTEGRAÇÃO / O QUE FALTA                                ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                    ║
║  Hardware                                                          ║
║  --------                                                          ║
║  [ ] Posicionamento físico do Kinect sobre a mesa de areia         ║
║  [ ] Montagem e alinhamento do projetor apontando para a mesa      ║
║  [ ] Conexão USB do Kinect (trocar USE_KINECT_REAL para True       ║
║      em kinect.py quando o sensor estiver disponível)              ║
║                                                                    ║
║  Calibração do Projetor (Passo 3 + 4 do orientador)               ║
║  ---------------------------------------------------               ║
║  [ ] Projetar tabuleiro de xadrez na mesa de areia                 ║
║  [ ] Capturar imagem do tabuleiro pelo Kinect                      ║
║  [ ] Usar encontrar_cantos_tabuleiro() para obter pontos 2D        ║
║  [ ] Usar calibrar_projetor() / cv2.calibrateCamera para gerar     ║
║      camera_matrix, dist_coeffs, rvec, tvec reais                  ║
║  [ ] Salvar parâmetros em .npz para não recalibrar toda vez        ║
║                                                                    ║
║  Cartografia / MDE                                                 ║
║  ----------------                                                  ║
║  [ ] Receber arquivo do MDE (GeoTIFF, .npy, ou matriz CSV)        ║
║  [ ] Implementar leitura real em AdaptadorMDE.carregar_mapa()      ║
║  [ ] Definir resolução espacial e sistema de coordenadas           ║
║      compatível com o referencial da mesa                          ║
║                                                                    ║
║  Software                                                          ║
║  --------                                                          ║
║  [ ] Open3D: instalar e ativar nuvem RGBD (Passo 5) quando        ║
║      imagem colorida do Kinect estiver disponível                  ║
║  [ ] Ajustar tolerância de cor com base em testes com areia real   ║
║  [ ] Tela cheia no projetor (tela_cheia=True em exibir)            ║
║                                                                    ║
╚══════════════════════════════════════════════════════════════════════╝
"""

from __future__ import annotations

import sys
import numpy as np
import cv2

from typing import Optional

# ── Módulos da Raquel (captura / exibição / utilitários) ──────────────
from kinect import capturar_profundidade, profundidade_para_pontos
from exibicao import exibir_profundidade
from utils import Cronometro, log

# ── Motor matemático (Rafael) ────────────────────────────────────────
from motor_caixao_areia import (
    pipeline_plano_e_base,
    transformar_pontos,
    gerar_mapa_cores,
    projetar_pontos_tsai,
)

# ── Adaptador MDE (interface com Cartografia) ────────────────────────
from adaptador_mde import AdaptadorMDE


# =====================================================================
# Parâmetros mockados do projetor (serão substituídos pela calibração)
# =====================================================================

def _parametros_projetor_mock() -> dict:
    """Retorna parâmetros intrínsecos/extrínsecos mockados do projetor.

    TODO — CALIBRAÇÃO FÍSICA
    ─────────────────────────
    Quando a calibração real for feita (Passos 3 e 4 do orientador):
      1. Projetar tabuleiro na areia
      2. Capturar imagem pelo Kinect
      3. encontrar_cantos_tabuleiro() → pontos 2D
      4. calibrar_projetor() → camera_matrix, dist_coeffs, rvec, tvec
      5. Salvar em calibracao_projetor.npz
      6. Carregar aqui com np.load() em vez desses valores mockados.
    """
    camera_matrix = np.array([
        [500.0,   0.0, 320.0],
        [  0.0, 500.0, 240.0],
        [  0.0,   0.0,   1.0],
    ])
    dist_coeffs = np.zeros(5)
    rvec = np.zeros((3, 1))
    tvec = np.array([[0.0], [0.0], [1.0]])   # câmera 1m acima da mesa

    return {
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "rvec": rvec,
        "tvec": tvec,
    }


# =====================================================================
# FASE 1 — Calibração Inicial
# =====================================================================

class Calibracao:
    """Armazena os resultados da calibração (plano + base + projetor)."""

    def __init__(self) -> None:
        self.T: Optional[np.ndarray] = None          # Matriz 4×4
        self.normal: Optional[np.ndarray] = None
        self.centroide: Optional[np.ndarray] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.rvec: Optional[np.ndarray] = None
        self.tvec: Optional[np.ndarray] = None
        self._calibrado: bool = False

    @property
    def esta_calibrado(self) -> bool:
        return self._calibrado

    def calibrar(self) -> None:
        """Executa a calibração: captura frame → ajusta plano → monta base.

        Passos executados:
          1. Captura profundidade (Kinect ou simulador)
          2. Converte para nuvem de pontos 3D
          3. pipeline_plano_e_base() → normal, centroide, T  (Passos 1+2)

        TODO — Passo 3+4 (calibração do projetor):
          4. Projetar tabuleiro → capturar → encontrar_cantos_tabuleiro()
          5. calibrar_projetor() → camera_matrix, dist_coeffs, rvec, tvec
          6. Salvar em .npz para reutilização
        """
        print("[Calibração] Capturando frame de referência...")

        profundidade = capturar_profundidade()
        pontos = profundidade_para_pontos(profundidade).astype(np.float64)

        if pontos.shape[0] < 10:
            raise RuntimeError(
                "[Calibração] Poucos pontos válidos — verifique o sensor Kinect."
            )

        print(f"[Calibração] {pontos.shape[0]:,} pontos capturados.")
        print("[Calibração] Ajustando plano via SVD...")

        normal, d, centroide, X, Y, Z, T = pipeline_plano_e_base(pontos)

        self.T = T
        self.normal = normal
        self.centroide = centroide

        print(f"[Calibração] Plano: {normal[0]:.4f}x + {normal[1]:.4f}y "
              f"+ {normal[2]:.4f}z + {d:.2f} = 0")
        print(f"[Calibração] Centroide: {centroide}")

        # --- Parâmetros do projetor (mock por enquanto) ---
        # TODO: substituir por calibração real (ver docstring acima)
        params = _parametros_projetor_mock()
        self.camera_matrix = params["camera_matrix"]
        self.dist_coeffs = params["dist_coeffs"]
        self.rvec = params["rvec"]
        self.tvec = params["tvec"]

        self._calibrado = True
        print("[Calibração] Concluída com sucesso.\n")


# =====================================================================
# FASE 2 — Loop de Tempo Real
# =====================================================================

def processar_frame(
    calibracao: Calibracao,
    mde: AdaptadorMDE,
    resolucao: tuple[int, int] = (640, 480),
    tolerancia: float = 5.0,
) -> np.ndarray:
    """Processa um frame completo do pipeline AR.

    1. Captura profundidade  →  nuvem 3D
    2. Aplica T (Kinect → Mesa)
    3. Compara com MDE → gera cores por ponto
    4. Projeta cores nos pixels do projetor
    5. Monta imagem BGR para exibição

    Parameters
    ----------
    calibracao : Calibracao
    mde : AdaptadorMDE
    resolucao : (largura, altura) do projetor/janela
    tolerancia : faixa de aceitação (mm) para a coloração

    Returns
    -------
    imagem : np.ndarray, shape (H, W, 3), dtype uint8
    """
    # 1. Captura
    profundidade = capturar_profundidade()
    pontos = profundidade_para_pontos(profundidade).astype(np.float64)

    if pontos.shape[0] == 0:
        # Sem pontos válidos → imagem preta
        return np.zeros((resolucao[1], resolucao[0], 3), dtype=np.uint8)

    # 2. Transformar para referencial da mesa
    pontos_mesa = transformar_pontos(calibracao.T, pontos)

    # 3. Gerar cores (comparação Kinect vs MDE)
    cores = gerar_mapa_cores(
        pontos_mesa,
        funcao_mde=mde.obter_z_alvo,
        tolerancia=tolerancia,
    )

    # 4. Projetar pontos 3D da mesa → pixels 2D do projetor
    pixels = projetar_pontos_tsai(
        pontos_mesa,
        calibracao.rvec,
        calibracao.tvec,
        calibracao.camera_matrix,
        calibracao.dist_coeffs,
    )

    # 5. Montar imagem de saída
    largura, altura = resolucao
    imagem = np.zeros((altura, largura, 3), dtype=np.uint8)

    # Pintar cada pixel projetado com a cor correspondente
    u = np.clip(pixels[:, 0].astype(int), 0, largura - 1)
    v = np.clip(pixels[:, 1].astype(int), 0, altura - 1)
    imagem[v, u] = cores

    return imagem


# =====================================================================
# MAIN — Orquestração
# =====================================================================

def main() -> None:
    print("=" * 60)
    print("  CAIXÃO DE AREIA — AR Sandbox")
    print("  PFC Engenharia de Computação 2026")
    print("=" * 60)
    print()

    # ── Inicializar MDE ───────────────────────────────────────────
    mde = AdaptadorMDE()
    mde.carregar_mapa("terreno_placeholder.tif")
    print()

    # ── Fase 1: Calibração ────────────────────────────────────────
    calibracao = Calibracao()
    calibracao.calibrar()

    # ── Fase 2: Loop de tempo real ────────────────────────────────
    print("Loop AR iniciado. Teclas:")
    print("  [R] Recalibrar plano da mesa")
    print("  [Q] Sair")
    print()

    cronometro = Cronometro()
    RESOLUCAO = (640, 480)
    TOLERANCIA = 5.0  # mm

    while True:
        try:
            imagem_ar = processar_frame(
                calibracao, mde,
                resolucao=RESOLUCAO,
                tolerancia=TOLERANCIA,
            )
        except Exception as e:
            # Tratamento de perda de conexão / erro no sensor
            print(f"\n[ERRO] Falha no frame: {e}")
            print("[ERRO] Tentando próximo frame...")
            continue

        # Exibir usando o módulo da Raquel
        exibir_profundidade(imagem_ar, janela="AR Sandbox", tela_cheia=False)

        # Log de performance
        fps = cronometro.marcar()
        log(f"FPS: {fps:.1f}")

        # Teclado
        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('q'):
            break
        elif tecla == ord('r'):
            print("\n[Recalibrando...]")
            calibracao.calibrar()

    cv2.destroyAllWindows()
    print("\nSistema encerrado.")


if __name__ == "__main__":
    main()

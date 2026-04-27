"""
motor_caixao_areia.py — Motor Matemático do Caixão de Areia (AR Sandbox)
=========================================================================
Projeto Final de Curso — Engenharia de Computação (AMAN, 2026)

Módulo **puro** de álgebra linear e projeção, sem dependência de hardware.

Pipeline Matemático
-------------------
1. **Ajuste de plano** — Mínimos Quadráticos via SVD (``ajustar_plano_svd``).
2. **Referencial da mesa** — Gram-Schmidt + Produto Vetorial
   (``construir_base_mesa``, ``montar_matriz_transformacao``).
3. **Detecção de grid** — ``cv2.findChessboardCorners``
   (``encontrar_cantos_tabuleiro``).
4. **Projeção 3D → 2D** — Modelo de Tsai via ``cv2.projectPoints``
   (``projetar_pontos_tsai``, ``calibrar_projetor``).
4b. **Calibração Coplanar** — Geração do padrão de xadrez para projeção
    (``gerar_imagem_xadrez``) e orquestrador completo de 5 passos
    (``calibrar_coplanar``) que resolve o problema de degeneração quando
    a areia está quase plana, forçando Z_local = 0 nos objectPoints.
    Inclui ``projetar_ponto_rgbd`` para uso no loop RGBD em tempo real.
5. **Nuvem RGBD** — Conversão Open3D (``criar_nuvem_de_pontos_open3d``).
6. **Coloração MDE** — Comparação Z_real vs Z_alvo com tolerância
   (``gerar_mapa_cores``).
7. **Discretização em Grade** — Malha de quadrados coloridos preenchidos
   com ``cv2.fillPoly`` (``discretizar_nuvem_em_grade``,
   ``gerar_imagem_grade_cores``).
"""

from __future__ import annotations

import numpy as np
import cv2

from typing import Tuple, Optional, Callable, List

# ============================================================================
# Tipo auxiliar
# ============================================================================
Cor = Tuple[int, int, int]  # (B, G, R) no padrão OpenCV


# ============================================================================
# 1. AJUSTE DE PLANO — Mínimos Quadráticos via SVD
# ============================================================================

def ajustar_plano_svd(pontos: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
    """Encontra o plano que melhor se ajusta à nuvem de pontos usando SVD.

    O método desloca os pontos para o centroide e aplica SVD na matriz
    centralizada.  O vetor normal é o vetor singular associado ao menor
    valor singular.

    Parameters
    ----------
    pontos : np.ndarray, shape (N, 3)
        Nuvem de pontos 3D (x, y, z).

    Returns
    -------
    normal : np.ndarray, shape (3,)
        Vetor normal unitário do plano (a, b, c).
    d : float
        Coeficiente *d* da equação  ax + by + cz + d = 0.
    centroide : np.ndarray, shape (3,)
        Centroide da nuvem de pontos.
    """
    if pontos.shape[0] < 3:
        raise ValueError("São necessários pelo menos 3 pontos para ajustar um plano.")

    centroide = pontos.mean(axis=0)                     # (3,)
    pontos_centralizados = pontos - centroide            # (N, 3)

    # SVD da matriz centralizada
    # U (N×N), S (3,), Vt (3×3)
    _, _, Vt = np.linalg.svd(pontos_centralizados, full_matrices=False)

    # O último vetor-linha de Vt corresponde ao menor valor singular → normal
    normal = Vt[-1]                                      # (3,)

    # Garantir que a normal aponte "para cima" (componente z positiva)
    if normal[2] < 0:
        normal = -normal

    # d = -n · centroide  (para satisfazer  n · p + d = 0)
    d = -float(np.dot(normal, centroide))

    return normal, d, centroide


# ============================================================================
# 2. SISTEMA DE COORDENADAS — Kinect → Mesa (Gram-Schmidt)
# ============================================================================

def gram_schmidt(v: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Remove a componente de *v* na direção de *ref* e normaliza.

    Retorna o vetor ortogonal unitário resultante.  Usado internamente
    para construir uma base ortonormal a partir da normal do plano.

    Parameters
    ----------
    v : np.ndarray, shape (3,)
        Vetor a ser ortogonalizado.
    ref : np.ndarray, shape (3,)
        Vetor de referência (já normalizado).

    Returns
    -------
    np.ndarray, shape (3,)
        Vetor unitário ortogonal a *ref*.
    """
    proj = np.dot(v, ref) * ref
    ortogonal = v - proj
    norma = np.linalg.norm(ortogonal)
    if norma < 1e-12:
        raise ValueError("O vetor fornecido é (anti)paralelo à referência.")
    return ortogonal / norma


def construir_base_mesa(
    normal: np.ndarray,
    semente: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Constrói 3 eixos ortonormais a partir da normal do plano.

    1. Z_mesa = normal (já unitário).
    2. X_mesa = Gram-Schmidt(semente, Z_mesa).
    3. Y_mesa = Z_mesa × X_mesa   (produto vetorial → garante ortonormalidade).

    Parameters
    ----------
    normal : np.ndarray, shape (3,)
        Vetor normal do plano já unitário.
    semente : np.ndarray | None
        Vetor auxiliar para Gram-Schmidt.  Se ``None``, usa [1, 0, 0]
        (ou [0, 1, 0] quando a normal for quase paralela a x).

    Returns
    -------
    X_mesa, Y_mesa, Z_mesa : np.ndarray, shape (3,)
    """
    Z_mesa = normal / np.linalg.norm(normal)

    if semente is None:
        # Escolhe semente que não seja paralela à normal
        if abs(np.dot(Z_mesa, np.array([1.0, 0.0, 0.0]))) < 0.9:
            semente = np.array([1.0, 0.0, 0.0])
        else:
            semente = np.array([0.0, 1.0, 0.0])

    X_mesa = gram_schmidt(semente, Z_mesa)
    Y_mesa = np.cross(Z_mesa, X_mesa)
    Y_mesa = Y_mesa / np.linalg.norm(Y_mesa)  # segurança numérica

    return X_mesa, Y_mesa, Z_mesa


def montar_matriz_transformacao(
    X_mesa: np.ndarray,
    Y_mesa: np.ndarray,
    Z_mesa: np.ndarray,
    origem: np.ndarray,
) -> np.ndarray:
    """Monta a matriz de transformação afim 4×4 Kinect → Mesa.

    A matriz resultante *T* satisfaz:
        p_mesa = T @ [x_kinect, y_kinect, z_kinect, 1]ᵀ

    A rotação leva os eixos do Kinect para a base da mesa, e a
    translação desloca a origem para o centroide do plano.

    Parameters
    ----------
    X_mesa, Y_mesa, Z_mesa : np.ndarray, shape (3,)
        Eixos ortonormais da mesa.
    origem : np.ndarray, shape (3,)
        Ponto que se tornará a nova origem (centroide do plano).

    Returns
    -------
    T : np.ndarray, shape (4, 4)
        Matriz de transformação afim.
    """
    R = np.vstack([X_mesa, Y_mesa, Z_mesa])          # (3, 3)
    t = -R @ origem                                    # translação

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def transformar_pontos(T: np.ndarray, pontos: np.ndarray) -> np.ndarray:
    """Aplica a transformação afim 4×4 a uma nuvem de pontos.

    Parameters
    ----------
    T : np.ndarray, shape (4, 4)
    pontos : np.ndarray, shape (N, 3)

    Returns
    -------
    np.ndarray, shape (N, 3)
        Pontos no referencial da mesa.
    """
    N = pontos.shape[0]
    homogeneos = np.hstack([pontos, np.ones((N, 1))])   # (N, 4)
    transformados = (T @ homogeneos.T).T                 # (N, 4)
    return transformados[:, :3]


# ============================================================================
# 3. CAPTURA 2D DE GRID — Detecção de tabuleiro de xadrez
# ============================================================================

def encontrar_cantos_tabuleiro(
    imagem: np.ndarray,
    tamanho_tabuleiro: Tuple[int, int] = (7, 5),
    refinar: bool = True,
) -> Tuple[bool, Optional[np.ndarray]]:
    """Detecta os cantos internos de um tabuleiro de xadrez na imagem.

    Usa ``cv2.findChessboardCorners`` e, opcionalmente, refina com
    ``cv2.cornerSubPix`` para precisão sub-pixel.

    Parameters
    ----------
    imagem : np.ndarray
        Imagem BGR ou escala de cinza.
    tamanho_tabuleiro : (colunas, linhas)
        Número de cantos internos do tabuleiro.
    refinar : bool
        Se ``True``, aplica refinamento sub-pixel.

    Returns
    -------
    encontrado : bool
    cantos : np.ndarray | None, shape (N, 1, 2)
        Coordenadas 2D dos cantos encontrados, ou ``None``.
    """
    if len(imagem.shape) == 3:
        cinza = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
    else:
        cinza = imagem

    encontrado, cantos = cv2.findChessboardCorners(cinza, tamanho_tabuleiro, None)

    if encontrado and refinar:
        criterio = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cantos = cv2.cornerSubPix(cinza, cantos, (11, 11), (-1, -1), criterio)

    return encontrado, cantos if encontrado else None


# ============================================================================
# 4. ALGORITMO DE TSAI — Projeção 3D → 2D via cv2.projectPoints
# ============================================================================

def projetar_pontos_tsai(
    pontos_3d: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Projeta pontos 3D (referencial da mesa) nos pixels 2D do projetor.

    Usa ``cv2.projectPoints``, que internamente aplica o modelo de câmera
    de Tsai (rotação, translação, parâmetros intrínsecos e distorção).

    Parameters
    ----------
    pontos_3d : np.ndarray, shape (N, 3) ou (N, 1, 3)
        Pontos no referencial da mesa.
    rvec : np.ndarray, shape (3, 1)
        Vetor de Rodrigues (rotação extrínseca).
    tvec : np.ndarray, shape (3, 1)
        Translação extrínseca.
    camera_matrix : np.ndarray, shape (3, 3)
        Matriz intrínseca do projetor  [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
    dist_coeffs : np.ndarray | None
        Coeficientes de distorção (k1, k2, p1, p2[, k3...]). Se ``None``,
        assume distorção zero.

    Returns
    -------
    pixels : np.ndarray, shape (N, 2)
        Coordenadas (u, v) projetadas.
    """
    if dist_coeffs is None:
        dist_coeffs = np.zeros(5)

    pontos_3d = pontos_3d.reshape(-1, 1, 3).astype(np.float64)
    rvec = rvec.astype(np.float64).reshape(3, 1)
    tvec = tvec.astype(np.float64).reshape(3, 1)

    pixels_2d, _ = cv2.projectPoints(
        pontos_3d, rvec, tvec, camera_matrix, dist_coeffs
    )
    return pixels_2d.reshape(-1, 2)


def calibrar_projetor(
    pontos_3d_mesa: np.ndarray,
    pontos_2d_projetor: np.ndarray,
    tamanho_imagem: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Calibra o projetor usando correspondências 3D↔2D (cv2.calibrateCamera).

    Parameters
    ----------
    pontos_3d_mesa : np.ndarray, shape (N, 3)
        Pontos 3D no referencial da mesa.
    pontos_2d_projetor : np.ndarray, shape (N, 2)
        Pixels correspondentes na imagem do projetor.
    tamanho_imagem : (largura, altura)
        Resolução do projetor.

    Returns
    -------
    camera_matrix, dist_coeffs, rvec, tvec
    """
    obj_pts = [pontos_3d_mesa.astype(np.float32)]
    img_pts = [pontos_2d_projetor.reshape(-1, 1, 2).astype(np.float32)]

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, tamanho_imagem, None, None
    )
    return camera_matrix, dist_coeffs, rvecs[0], tvecs[0]


# ============================================================================
# 4b. CALIBRAÇÃO COPLANAR — Gerador de Xadrez + Orchestrador (5 Passos)
# ============================================================================

def gerar_imagem_xadrez(
    resolucao: Tuple[int, int],
    tamanho_tabuleiro: Tuple[int, int] = (7, 5),
    tamanho_quadrado: int = 60,
) -> Tuple[np.ndarray, np.ndarray]:
    """Gera o padrão de xadrez para projetar sobre a areia durante a calibração.

    O tabuleiro é centralizado na imagem do projetor.  Os cantos internos são
    as interseções entre quadrados pretos e brancos — são exatamente esses
    pontos que ``cv2.findChessboardCorners`` detecta na imagem do Kinect.

    O ordering dos cantos é o mesmo que o OpenCV usa:
    esquerda → direita dentro de cada linha, linhas de cima → baixo.
    Isso garante correspondência direta com os pixels retornados por
    ``cv2.findChessboardCorners``.

    Parameters
    ----------
    resolucao : (largura, altura)
        Resolução do projetor em pixels.
    tamanho_tabuleiro : (colunas, linhas)
        Número de *cantos internos* do tabuleiro.  Ex: ``(7, 5)`` gera um
        tabuleiro de 8×6 quadrados com 7×5 = 35 cantos internos.
    tamanho_quadrado : int
        Lado de cada quadrado em pixels.

    Returns
    -------
    imagem : np.ndarray, shape (H, W, 3), dtype uint8
        Imagem BGR do xadrez para exibir no projetor.
    cantos_projetor : np.ndarray, shape (M, 2), dtype float32
        Posições ``(u, v)`` dos cantos internos em pixels do projetor.
        ``M = colunas × linhas``.
    """
    W, H = resolucao
    cols, rows = tamanho_tabuleiro   # cantos internos
    n_sq_x = cols + 1                # quadrados na direção X
    n_sq_y = rows + 1                # quadrados na direção Y

    img_gray = np.ones((H, W), dtype=np.uint8) * 255  # fundo branco

    # Centralizar o tabuleiro na imagem
    board_w = n_sq_x * tamanho_quadrado
    board_h = n_sq_y * tamanho_quadrado
    x0 = (W - board_w) // 2
    y0 = (H - board_h) // 2

    # Pintar quadrados onde (linha + coluna) é par → preto
    for r in range(n_sq_y):
        for c in range(n_sq_x):
            if (r + c) % 2 == 0:
                x1 = x0 + c * tamanho_quadrado
                y1 = y0 + r * tamanho_quadrado
                img_gray[y1:y1 + tamanho_quadrado, x1:x1 + tamanho_quadrado] = 0

    # Calcular posições dos cantos internos em pixels do projetor.
    # Canto interno (r, c) fica na interseção das linhas divisórias:
    #   u = x0 + (c + 1) * tamanho_quadrado
    #   v = y0 + (r + 1) * tamanho_quadrado
    cantos: List[List[float]] = []
    for r in range(rows):
        for c in range(cols):
            px = float(x0 + (c + 1) * tamanho_quadrado)
            py = float(y0 + (r + 1) * tamanho_quadrado)
            cantos.append([px, py])

    cantos_arr = np.array(cantos, dtype=np.float32)  # (M, 2)
    imagem_bgr = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    return imagem_bgr, cantos_arr




def calibrar_coplanar(
    nuvem_areia: np.ndarray,
    pontos_3d_grid_kinect: np.ndarray,
    pontos_2d_grid_projetor: np.ndarray,
    tamanho_imagem: Tuple[int, int],
    semente: Optional[np.ndarray] = None,
) -> Tuple[
    np.ndarray,  # T          (4×4) — Kinect → Mesa
    np.ndarray,  # camera_matrix (3×3)
    np.ndarray,  # dist_coeffs
    np.ndarray,  # rvec (3×1)
    np.ndarray,  # tvec (3×1)
    np.ndarray,  # normal (3,)
    np.ndarray,  # centroide (3,)
]:
    """Calibração Coplanar completa em 5 passos.

    Resolve o problema de configuração degenerada que ocorre quando a areia
    está quase plana (variação Z ≈ ruído do sensor): ao invés de tentar
    estimar uma transformação 3D não-coplanar mal-condicionada, aproveitamos
    o fato de que a areia É plana para ajustar um plano e trabalhar com
    ``Z_local = 0`` como superfície matemática estrita.

    ─────────────────────────────────────────────────────────────────────────
    Passo 1 — Ajuste de Plano por SVD (Mínimos Quadrados)
    ─────────────────────────────────────────────────────────────────────────
    Recebe a nuvem ruidosa (N, 3) e encontra o plano ótimo por SVD:

        1. Centraliza: ``P̃ = P − centroide``
        2. Decompõe: ``P̃ = U · S · Vᵀ``  (SVD economy)
        3. Normal:   ``n = última linha de Vᵀ``  (vetor singular mínimo)

    O menor valor singular ``σ_min`` representa a espessura da "laje"
    de pontos em torno do plano — quanto menor, mais plana a areia.
    A equação canônica do plano é ``n·p + d = 0``, com ``d = −n·centroide``.

    ─────────────────────────────────────────────────────────────────────────
    Passo 2 — Sistema de Coordenadas Local (Gram-Schmidt + Produto Vetorial)
    ─────────────────────────────────────────────────────────────────────────
    Constrói a base ortonormal da "mesa" a partir da normal ``n``:

        Z_mesa = n / ‖n‖                         ← perpendicular à areia
        X_mesa = GramSchmidt(semente, Z_mesa)    ← eixo no plano
        Y_mesa = Z_mesa × X_mesa / ‖…‖           ← produto vetorial → ⊥ garantido

    Gram-Schmidt aplicado à semente ``v`` em relação a ``ref``:
        ``X_mesa = (v − (v·ref)·ref) / ‖v − (v·ref)·ref‖``

    Monta a matriz afim 4×4:
        ``T = [ R | t ]``,  ``R = [X_mesa; Y_mesa; Z_mesa]``,  ``t = −R·centroide``
              ``[ 0 | 1 ]``

    Após ``T``, qualquer ponto da superfície plana da areia satisfaz
    estritamente ``Z_local = 0``.

    ─────────────────────────────────────────────────────────────────────────
    Passo 3 — Pontos do Grid no Referencial Local
    ─────────────────────────────────────────────────────────────────────────
    Aplica ``T`` aos ``M`` pontos 3D das interseções do grid (lidas pelo
    Kinect). O resultado é ``(X_local, Y_local, Z_local)`` onde
    ``Z_local ≈ 0`` (residual de ruído).

    ─────────────────────────────────────────────────────────────────────────
    Passo 4 — Tsai Coplanar via cv2.calibrateCamera
    ─────────────────────────────────────────────────────────────────────────
    Força ``Z_local = 0`` em todos os objectPoints antes de passar ao OpenCV:

        ``obj_pts[i] = (X_local[i], Y_local[i], 0)``

    Esta é a hipótese coplanar de Tsai. ``cv2.calibrateCamera`` usa DLT
    seguido de refinamento Levenberg-Marquardt para estimar:

        - **K** = ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``  (intrínseca)
        - **dist** = ``(k1, k2, p1, p2, k3)``               (distorção)
        - **rvec** (Rodrigues) e **tvec**                    (extrínseca)

    O sistema é bem condicionado porque os pontos têm boa distribuição
    em X e Y (o grid calibrador), e a hipótese Z=0 remove a degeneração.

    ─────────────────────────────────────────────────────────────────────────
    Passo 5 — Transição para Loop RGBD
    ─────────────────────────────────────────────────────────────────────────
    Retorna ``T, K, dist, rvec, tvec``. No loop em tempo real, para
    projetar um novo ponto ``(X_real, Y_real, Z_real)`` do Kinect em
    pixels ``(u, v)`` do projetor use :func:`projetar_ponto_rgbd`.

    Parameters
    ----------
    nuvem_areia : np.ndarray, shape (N, 3)
        Nuvem de pontos 3D do Kinect da areia quase plana (m).
    pontos_3d_grid_kinect : np.ndarray, shape (M, 3)
        Posições 3D (X, Y, Z) das interseções do grid calibrador,
        lidas pelo Kinect (mesmo referencial de ``nuvem_areia``).
    pontos_2d_grid_projetor : np.ndarray, shape (M, 2)
        Coordenadas de pixel (u, v) correspondentes às interseções
        do grid na imagem do projetor.
    tamanho_imagem : (largura, altura)
        Resolução do projetor em pixels, ex.: ``(1280, 720)``.
    semente : np.ndarray | None
        Vetor semente para a ortogonalização Gram-Schmidt.
        Se ``None``, escolhe automaticamente (veja :func:`construir_base_mesa`).

    Returns
    -------
    T : np.ndarray, shape (4, 4)
        Matriz afim Kinect → Mesa.
    camera_matrix : np.ndarray, shape (3, 3)
        Parâmetros intrínsecos do projetor.
    dist_coeffs : np.ndarray
        Coeficientes de distorção.
    rvec : np.ndarray, shape (3, 1)
        Vetor de Rodrigues (rotação extrínseca Mesa → Projetor).
    tvec : np.ndarray, shape (3, 1)
        Translação extrínseca Mesa → Projetor.
    normal : np.ndarray, shape (3,)
        Vetor normal do plano ajustado (referencial Kinect).
    centroide : np.ndarray, shape (3,)
        Centroide da nuvem usada no ajuste de plano.

    Raises
    ------
    ValueError
        Se M < 4 (mínimo para cv2.calibrateCamera coplanar) ou se os
        dois arrays de correspondência tiverem tamanhos incompatíveis.
    """
    M = pontos_3d_grid_kinect.shape[0]
    if M < 4:
        raise ValueError(
            f"São necessários pelo menos 4 pontos de correspondência; recebidos: {M}."
        )
    if pontos_2d_grid_projetor.shape[0] != M:
        raise ValueError(
            "pontos_3d_grid_kinect e pontos_2d_grid_projetor devem ter o mesmo "
            f"número de linhas (recebidos {M} e {pontos_2d_grid_projetor.shape[0]})."
        )

    # ── Passo 1: Ajuste de plano por SVD ──────────────────────────────────
    # ajustar_plano_svd centraliza a nuvem, aplica SVD e extrai a normal
    # como o vetor singular associado ao menor valor singular σ_min.
    normal, d, centroide = ajustar_plano_svd(nuvem_areia)

    # ── Passo 2: Base ortonormal da mesa (Gram-Schmidt + Produto Vetorial) ─
    # Z_mesa = normal;  X_mesa via Gram-Schmidt;  Y_mesa = Z×X (cross product)
    X_mesa, Y_mesa, Z_mesa = construir_base_mesa(normal, semente)
    T = montar_matriz_transformacao(X_mesa, Y_mesa, Z_mesa, centroide)

    # ── Passo 3: Transformar pontos do grid para o referencial local ───────
    # Após T, a superfície plana da areia satisfaz Z_local ≈ 0.
    grid_local = transformar_pontos(T, pontos_3d_grid_kinect)  # (M, 3)

    # ── Passo 4: Tsai Coplanar — forçar Z_local = 0 (hipótese coplanar) ───
    # Esta é a chave da calibração coplanar de Tsai: substituímos o Z residual
    # (ruído do sensor) por 0 exato, tornando o sistema bem condicionado.
    # cv2.calibrateCamera recebe uma *lista* de vistas; passamos apenas uma.
    obj_pts_coplanar = np.column_stack([
        grid_local[:, 0],                    # X_local (variação real XY do grid)
        grid_local[:, 1],                    # Y_local (variação real XY do grid)
        np.zeros(M, dtype=np.float64),       # Z_local = 0  ← hipótese coplanar
    ]).astype(np.float32)                    # cv2 exige float32

    img_pts = pontos_2d_grid_projetor.reshape(-1, 1, 2).astype(np.float32)

    largura, altura = tamanho_imagem
    _ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        [obj_pts_coplanar],   # lista de objectPoints por vista
        [img_pts],            # lista de imagePoints por vista
        (largura, altura),    # resolução do projetor
        None,                 # sem estimativa inicial de K
        None,                 # sem estimativa inicial de distorção
    )

    # ── Passo 5: Retornar todos os parâmetros para o loop RGBD ────────────
    return T, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], normal, centroide


def projetar_ponto_rgbd(
    ponto_kinect: np.ndarray,
    T: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: Optional[np.ndarray] = None,
) -> Tuple[float, float]:
    """Projeta um ponto 3D do Kinect em pixels do projetor (hot-path RGBD).

    Encadeia os dois passos do loop em tempo real descrito no Passo 5 da
    calibração coplanar:

        **Passo A — Transformação de referencial:**
            ``p_local = T @ [X_real, Y_real, Z_real, 1]ᵀ``

            Leva o ponto do referencial do Kinect para o referencial da mesa.
            Aqui, ``Z_local ≠ 0`` representa a topografia REAL da areia —
            montanhas e vales esculpidos pelo usuário.

        **Passo B — Projeção Tsai:**
            ``(u, v) = projectPoints(p_local[:3], rvec, tvec, K, dist)``

            Usa os parâmetros obtidos na calibração coplanar. O modelo de Tsai
            lida corretamente com Z_local ≠ 0: o ponto saiu do plano de
            calibração, mas a projeção perspectiva ainda é válida, pois
            ``rvec``/``tvec``/``K`` descrevem a câmera completa em 3D.

    Parameters
    ----------
    ponto_kinect : np.ndarray, shape (3,) ou (1, 3)
        Ponto ``(X_real, Y_real, Z_real)`` no referencial do Kinect, em metros.
    T : np.ndarray, shape (4, 4)
        Matriz afim 4×4 retornada por :func:`calibrar_coplanar`.
    rvec : np.ndarray, shape (3, 1)
        Vetor de Rodrigues retornado por :func:`calibrar_coplanar`.
    tvec : np.ndarray, shape (3, 1)
        Translação extrínseca retornada por :func:`calibrar_coplanar`.
    camera_matrix : np.ndarray, shape (3, 3)
        Matriz intrínseca retornada por :func:`calibrar_coplanar`.
    dist_coeffs : np.ndarray | None
        Coeficientes de distorção. Se ``None``, assume distorção zero.

    Returns
    -------
    u : float
        Coluna do pixel no projetor.
    v : float
        Linha do pixel no projetor.
    """
    # Passo A: Kinect → referencial local da mesa (T é 4×4 afim)
    homo = np.array([*ponto_kinect.ravel()[:3], 1.0], dtype=np.float64)
    p_local = (T @ homo)[:3]  # (3,) — Z_local representa a topografia real

    # Passo B: referencial local → pixels do projetor (modelo de Tsai completo)
    px = projetar_pontos_tsai(
        p_local.reshape(1, 3), rvec, tvec, camera_matrix, dist_coeffs
    )  # (1, 2)
    return float(px[0, 0]), float(px[0, 1])


# ============================================================================
# 5. LEITURA RGBD — Nuvem de pontos com Open3D
# ============================================================================

def criar_nuvem_de_pontos_open3d(
    imagem_cor: np.ndarray,
    mapa_profundidade: np.ndarray,
    intrinsicos: Optional[dict] = None,
):
    """Cria uma nuvem de pontos Open3D a partir de imagem RGB-D.

    Parameters
    ----------
    imagem_cor : np.ndarray, shape (H, W, 3)
        Imagem BGR (será convertida para RGB internamente).
    mapa_profundidade : np.ndarray, shape (H, W)
        Mapa de profundidade em milímetros (uint16 ou float).
    intrinsicos : dict | None
        Dicionário com chaves ``fx, fy, cx, cy``.  Se ``None``, usa
        valores padrão do Kinect v1.

    Returns
    -------
    nuvem : open3d.geometry.PointCloud
    """
    import open3d as o3d

    if intrinsicos is None:
        intrinsicos = {"fx": 525.0, "fy": 525.0, "cx": 319.5, "cy": 239.5}

    H, W = mapa_profundidade.shape[:2]

    intrinsic_o3d = o3d.camera.PinholeCameraIntrinsic(
        W, H,
        intrinsicos["fx"], intrinsicos["fy"],
        intrinsicos["cx"], intrinsicos["cy"],
    )

    # Open3D espera RGB e profundidade como o3d.Image
    rgb = cv2.cvtColor(imagem_cor, cv2.COLOR_BGR2RGB)
    cor_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
    prof_o3d = o3d.geometry.Image(mapa_profundidade.astype(np.float32))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        cor_o3d, prof_o3d,
        depth_scale=1000.0,      # mm → m
        depth_trunc=3.0,         # descartar > 3 m
        convert_rgb_to_intensity=False,
    )

    nuvem = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic_o3d)
    return nuvem


def nuvem_para_numpy(nuvem) -> np.ndarray:
    """Converte uma PointCloud Open3D para array NumPy (N, 3).

    Parameters
    ----------
    nuvem : open3d.geometry.PointCloud

    Returns
    -------
    np.ndarray, shape (N, 3)
    """
    import open3d as o3d  # noqa: F811
    return np.asarray(nuvem.points)


# ============================================================================
# 6. INTEGRAÇÃO COM MDE (Modelo Digital de Elevação) — Interface + Coloração
# ============================================================================

def ler_mde_placeholder(x: float, y: float) -> float:
    """Interface/placeholder para o MDE da Cartografia.

    Quando o mapa real for entregue, substituir o corpo desta função
    pela leitura efetiva (ex.: interpolação do GeoTIFF).

    Parameters
    ----------
    x, y : float
        Coordenadas (no referencial da mesa) do ponto consultado.

    Returns
    -------
    z_esperado : float
        Altura Z esperada pelo MDE.
    """
    # Placeholder: superfície plana em z = 0
    return 0.0


def cor_por_diferenca(
    z_real: float,
    z_mde: float,
    tolerancia: float = 0.02,
) -> Cor:
    """Retorna a cor (BGR) de feedback conforme a diferença de alturas.

    Regra:
      - z_real > z_mde + tolerância  →  Vermelho  (precisa cavar)
      - z_real < z_mde - tolerância  →  Azul      (precisa preencher)
      - caso contrário               →  Verde     (OK)

    Parameters
    ----------
    z_real : float
        Altura medida pelo Kinect (referencial da mesa), em metros.
    z_mde : float
        Altura esperada pelo MDE, em metros.
    tolerancia : float
        Faixa de aceitação em metros (padrão: 0.02 m = 2 cm).

    Returns
    -------
    cor : Cor
        Tupla (B, G, R) no padrão OpenCV.
    """
    COR_VERMELHA: Cor = (0, 0, 255)
    COR_AZUL:     Cor = (255, 0, 0)
    COR_VERDE:    Cor = (0, 255, 0)

    if z_real > z_mde + tolerancia:
        return COR_VERMELHA
    elif z_real < z_mde - tolerancia:
        return COR_AZUL
    else:
        return COR_VERDE


def gerar_mapa_cores(
    pontos_mesa: np.ndarray,
    funcao_mde: Callable[[float, float], float] = ler_mde_placeholder,
    tolerancia: float = 0.02,
) -> np.ndarray:
    """Gera um array de cores (N, 3) BGR comparando Kinect vs MDE.

    Parameters
    ----------
    pontos_mesa : np.ndarray, shape (N, 3)
        Pontos já no referencial da mesa (x, y, z) em metros.
    funcao_mde : Callable[[float, float], float]
        Função que recebe (x, y) e retorna z_esperado em metros.
    tolerancia : float
        Tolerância em metros (padrão: 0.02 m = 2 cm).

    Returns
    -------
    cores : np.ndarray, shape (N, 3), dtype uint8
        Cada linha é a cor BGR do ponto correspondente.
    """
    N = pontos_mesa.shape[0]
    cores = np.empty((N, 3), dtype=np.uint8)

    for i in range(N):
        x, y, z_real = pontos_mesa[i]
        z_mde = funcao_mde(float(x), float(y))
        cores[i] = cor_por_diferenca(float(z_real), z_mde, tolerancia)

    return cores


# ============================================================================
# 7. DISCRETIZAÇÃO EM GRADE — Malha de quadrados coloridos
# ============================================================================

def discretizar_nuvem_em_grade(
    pontos_mesa: np.ndarray,
    n_celulas_x: int,
    n_celulas_y: int,
    largura_mesa: float,
    comprimento_mesa: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Agrupa os pontos da nuvem em uma grade regular e calcula a altura média por célula.

    A mesa é dividida em ``n_celulas_y × n_celulas_x`` quadrados iguais.
    Para cada célula, acumula as alturas Z dos pontos cujas coordenadas
    (X, Y) caem dentro dos limites da célula e retorna a média.

    Isso substitui a abordagem ponto-a-ponto, filtrando o ruído do
    Kinect ao agregar múltiplas leituras por célula.

    Parameters
    ----------
    pontos_mesa : np.ndarray, shape (N, 3)
        Nuvem de pontos no referencial da mesa (X, Y, Z) em metros.
    n_celulas_x : int
        Número de células (colunas) no eixo X.
    n_celulas_y : int
        Número de células (linhas) no eixo Y.
    largura_mesa : float
        Dimensão X da mesa em metros.
    comprimento_mesa : float
        Dimensão Y da mesa em metros.

    Returns
    -------
    alturas : np.ndarray, shape (n_celulas_y, n_celulas_x), dtype float64
        Altura Z média por célula.  ``NaN`` onde não há pontos.
    contagens : np.ndarray, shape (n_celulas_y, n_celulas_x), dtype int32
        Quantidade de pontos do Kinect que caíram em cada célula.
    """
    tam_celula_x = largura_mesa / n_celulas_x
    tam_celula_y = comprimento_mesa / n_celulas_y

    soma_z = np.zeros((n_celulas_y, n_celulas_x), dtype=np.float64)
    contagens = np.zeros((n_celulas_y, n_celulas_x), dtype=np.int32)

    x = pontos_mesa[:, 0]
    y = pontos_mesa[:, 1]
    z = pontos_mesa[:, 2]

    # Índices de célula para cada ponto (clamp para limites da grade)
    col = np.clip((x / tam_celula_x).astype(np.int32), 0, n_celulas_x - 1)
    lin = np.clip((y / tam_celula_y).astype(np.int32), 0, n_celulas_y - 1)

    # Acumulação vetorizada
    np.add.at(soma_z, (lin, col), z)
    np.add.at(contagens, (lin, col), 1)

    alturas = np.full((n_celulas_y, n_celulas_x), np.nan, dtype=np.float64)
    mascara = contagens > 0
    alturas[mascara] = soma_z[mascara] / contagens[mascara]

    return alturas, contagens


def gerar_imagem_grade_cores(
    pontos_mesa: np.ndarray,
    funcao_mde: Callable[[float, float], float],
    tolerancia: float,
    n_celulas_x: int,
    n_celulas_y: int,
    largura_mesa: float,
    comprimento_mesa: float,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    resolucao: Tuple[int, int],
) -> np.ndarray:
    """Gera a imagem AR usando discretização em malha de quadrados coloridos.

    Pipeline completo por frame (abordagem de **grade**):

    1. **Discretização** — agrupa a nuvem de pontos do Kinect em uma
       grade regular de ``n_celulas_y × n_celulas_x`` células e calcula
       a altura média :math:`Z_{real\_media}` por célula.
    2. **Comparação com MDE** — para cada célula com dados, consulta
       ``funcao_mde(x_centro, y_centro)`` para obter :math:`Z_{MDE}` e
       aplica a classificação de cores (Vermelho / Azul / Verde).
    3. **Projeção Tsai** — projeta todos os vértices da grade (malha
       de :math:`(N_y+1) \times (N_x+1)` pontos) de uma só vez via
       ``cv2.projectPoints``, evitando chamadas repetidas.
    4. **Rasterização** — desenha cada célula como um polígono
       preenchido (``cv2.fillPoly``) com a cor correspondente,
       garantindo cobertura contínua sem buracos.

    O resultado é uma grade contínua de quadrados coloridos — como
    "curvas de nível discretizadas" — projetada sobre a areia.

    Parameters
    ----------
    pontos_mesa : np.ndarray, shape (N, 3)
        Nuvem de pontos no referencial da mesa (X, Y, Z) em metros.
    funcao_mde : Callable[[float, float], float]
        Função que recebe (x, y) e retorna a altura alvo Z_mde.
    tolerancia : float
        Tolerância em metros para classificação de cores.
    n_celulas_x : int
        Número de colunas da grade.
    n_celulas_y : int
        Número de linhas da grade.
    largura_mesa : float
        Dimensão X da mesa em metros.
    comprimento_mesa : float
        Dimensão Y da mesa em metros.
    rvec : np.ndarray
        Vetor de Rodrigues (rotação extrínseca → projetor).
    tvec : np.ndarray
        Translação extrínseca → projetor.
    camera_matrix : np.ndarray
        Matriz intrínseca do projetor (3×3).
    dist_coeffs : np.ndarray
        Coeficientes de distorção.
    resolucao : tuple[int, int]
        ``(largura, altura)`` em pixels da imagem de saída.

    Returns
    -------
    np.ndarray, shape (altura, largura, 3), dtype uint8
        Imagem BGR com a grade de quadrados coloridos.
    """
    largura_img, altura_img = resolucao
    imagem = np.zeros((altura_img, largura_img, 3), dtype=np.uint8)

    if pontos_mesa.shape[0] == 0:
        return imagem

    # ── 1. Discretização: agrupar pontos em células ──
    alturas, contagens = discretizar_nuvem_em_grade(
        pontos_mesa, n_celulas_x, n_celulas_y,
        largura_mesa, comprimento_mesa,
    )

    tam_celula_x = largura_mesa / n_celulas_x
    tam_celula_y = comprimento_mesa / n_celulas_y

    # ── 2. Projeção em lote dos vértices da grade ──
    # A grade tem (n_celulas_y + 1) × (n_celulas_x + 1) vértices.
    xs = np.linspace(0.0, largura_mesa, n_celulas_x + 1)
    ys = np.linspace(0.0, comprimento_mesa, n_celulas_y + 1)
    xx, yy = np.meshgrid(xs, ys)
    vertices_3d = np.column_stack([
        xx.ravel(), yy.ravel(), np.zeros(xx.size)
    ])  # (V, 3) com Z = 0 (plano da mesa)

    vertices_2d = projetar_pontos_tsai(
        vertices_3d, rvec, tvec, camera_matrix, dist_coeffs,
    )  # (V, 2)
    vertices_2d = vertices_2d.reshape(
        n_celulas_y + 1, n_celulas_x + 1, 2
    )  # indexável por [lin, col]

    # ── 3. Coloração e rasterização por célula ──
    for i in range(n_celulas_y):
        for j in range(n_celulas_x):
            if contagens[i, j] == 0:
                continue  # sem dados do Kinect nesta célula

            z_real_media = float(alturas[i, j])

            # Centro da célula para consulta ao MDE
            x_centro = (j + 0.5) * tam_celula_x
            y_centro = (i + 0.5) * tam_celula_y
            z_mde = funcao_mde(x_centro, y_centro)

            cor = cor_por_diferenca(z_real_media, z_mde, tolerancia)

            # 4 cantos da célula já projetados em 2D
            pts = np.array([
                vertices_2d[i,     j    ],
                vertices_2d[i,     j + 1],
                vertices_2d[i + 1, j + 1],
                vertices_2d[i + 1, j    ],
            ], dtype=np.int32).reshape((-1, 1, 2))

            cv2.fillPoly(imagem, [pts], cor)

    return imagem


# ============================================================================
# Pipeline completo — atalho de conveniência
# ============================================================================

def pipeline_plano_e_base(
    pontos: np.ndarray,
    semente: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, float, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Executa os Passos 1 e 2 de uma só vez (ajuste de plano + base + matriz).

    Returns
    -------
    normal, d, centroide, X_mesa, Y_mesa, Z_mesa, T
    """
    normal, d, centroide = ajustar_plano_svd(pontos)
    X_mesa, Y_mesa, Z_mesa = construir_base_mesa(normal, semente)
    T = montar_matriz_transformacao(X_mesa, Y_mesa, Z_mesa, centroide)
    return normal, d, centroide, X_mesa, Y_mesa, Z_mesa, T

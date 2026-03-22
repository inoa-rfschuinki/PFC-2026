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
5. **Nuvem RGBD** — Conversão Open3D (``criar_nuvem_de_pontos_open3d``).
6. **Coloração MDE** — Comparação Z_real vs Z_alvo com tolerância
   (``gerar_mapa_cores``).
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

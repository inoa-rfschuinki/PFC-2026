"""
test_motor_caixao.py — Testes Unitários do Motor Matemático
=============================================================
Projeto Final de Curso — Engenharia de Computação (2026)

Cada teste usa valores **hardcoded** e comentários que explicam a
matemática por trás dos asserts, para facilitar a apresentação à
banca examinadora.

Executar:
    python -m pytest test_motor_caixao.py -v
    python -m unittest test_motor_caixao -v
"""

import unittest
import numpy as np

from motor_caixao_areia import (
    ajustar_plano_svd,
    gram_schmidt,
    construir_base_mesa,
    montar_matriz_transformacao,
    transformar_pontos,
    encontrar_cantos_tabuleiro,
    projetar_pontos_tsai,
    cor_por_diferenca,
    gerar_mapa_cores,
    pipeline_plano_e_base,
)


# =====================================================================
# Helpers para comparação numérica
# =====================================================================

def arrays_proximos(a: np.ndarray, b: np.ndarray, tol: float = 1e-6) -> bool:
    return np.allclose(a, b, atol=tol)


# =====================================================================
# 1. TESTES — Ajuste de Plano (SVD)
# =====================================================================

class TestAjustePlano(unittest.TestCase):
    """Verifica que o SVD encontra corretamente a equação do plano."""

    def test_plano_horizontal_z5(self):
        """
        Pontos dispostos em z = 5 (plano horizontal).
        A normal esperada é (0, 0, 1) e d = -5.

        Equação: 0·x + 0·y + 1·z + (-5) = 0  →  z = 5  ✓
        """
        pontos = np.array([
            [0.0, 0.0, 5.0],
            [1.0, 0.0, 5.0],
            [0.0, 1.0, 5.0],
            [1.0, 1.0, 5.0],
            [2.0, 3.0, 5.0],
        ])
        normal, d, centroide = ajustar_plano_svd(pontos)

        # A normal deve ser (0, 0, ±1). Nosso código garante z > 0 → (0, 0, 1).
        self.assertTrue(
            arrays_proximos(np.abs(normal), [0, 0, 1]),
            f"Normal deveria ser (0,0,1), obteve {normal}",
        )

        # Verificação algébrica:  n · p + d = 0  para qualquer ponto do plano.
        # Ex.: (0,0,1)·(0,0,5) + d = 5 + d = 0  →  d = -5
        self.assertAlmostEqual(d, -5.0, places=4,
                               msg=f"d deveria ser -5, obteve {d}")

    def test_plano_inclinado(self):
        """
        Plano  x + y + z = 3  →  normal proporcional a (1,1,1),
        n unitário = (1,1,1)/√3 ≈ (0.577, 0.577, 0.577).

        Verificação: o produto escalar de qualquer ponto com a normal
        deve ser constante (= -d).
        """
        # Pontos que satisfazem x + y + z = 3
        pontos = np.array([
            [3.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [0.0, 0.0, 3.0],
            [1.0, 1.0, 1.0],
            [2.0, 0.5, 0.5],
        ])
        normal, d, centroide = ajustar_plano_svd(pontos)

        # Cada ponto deve satisfazer  n · p + d ≈ 0
        for p in pontos:
            residuo = np.dot(normal, p) + d
            self.assertAlmostEqual(residuo, 0.0, places=4,
                                   msg=f"Resíduo = {residuo} para ponto {p}")

    def test_normal_e_unitaria(self):
        """A norma do vetor normal retornado deve ser 1 (unitário)."""
        pontos = np.array([
            [0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
        ], dtype=float)
        normal, _, _ = ajustar_plano_svd(pontos)
        self.assertAlmostEqual(np.linalg.norm(normal), 1.0, places=8)

    def test_poucos_pontos_levanta_excecao(self):
        """Com menos de 3 pontos não é possível definir um plano."""
        with self.assertRaises(ValueError):
            ajustar_plano_svd(np.array([[0, 0, 0], [1, 0, 0]], dtype=float))


# =====================================================================
# 2. TESTES — Sistema de Coordenadas (Gram-Schmidt + Base)
# =====================================================================

class TestGramSchmidt(unittest.TestCase):
    """Testa a ortogonalização de Gram-Schmidt e a construção da base."""

    def test_ortogonalidade_simples(self):
        """
        Dado ref = (0, 0, 1) e v = (1, 0, 1):
          proj = (v · ref) × ref = 1 × (0,0,1) = (0,0,1)
          ortogonal = v - proj = (1, 0, 0)
          |ortogonal| = 1  →  resultado = (1, 0, 0)

        O produto escalar resultado · ref deve ser 0.
        """
        ref = np.array([0.0, 0.0, 1.0])
        v = np.array([1.0, 0.0, 1.0])
        resultado = gram_schmidt(v, ref)

        dot = np.dot(resultado, ref)
        self.assertAlmostEqual(dot, 0.0, places=10,
                               msg="Produto escalar deve ser 0 (ortogonalidade)")
        self.assertAlmostEqual(np.linalg.norm(resultado), 1.0, places=10,
                               msg="Vetor deve ser unitário")

    def test_vetores_paralelos_levanta_excecao(self):
        """Se v é paralelo a ref, Gram-Schmidt resulta em vetor nulo → erro."""
        ref = np.array([0.0, 0.0, 1.0])
        v = np.array([0.0, 0.0, 5.0])
        with self.assertRaises(ValueError):
            gram_schmidt(v, ref)


class TestConstruirBase(unittest.TestCase):
    """Testa a construção dos 3 eixos ortonormais da mesa."""

    def test_base_plano_horizontal(self):
        """
        Normal = (0, 0, 1)  →  plano horizontal.
        Esperamos Z_mesa = (0,0,1), X_mesa ⊥ Z_mesa, Y_mesa ⊥ ambos.

        Verificações:
          • X · Z = 0,  Y · Z = 0,  X · Y = 0   (ortogonalidade mútua)
          • |X| = |Y| = |Z| = 1                   (todos unitários)
        """
        normal = np.array([0.0, 0.0, 1.0])
        X, Y, Z = construir_base_mesa(normal)

        # Ortogonalidade: todos os produtos escalares cruzados devem ser 0
        self.assertAlmostEqual(np.dot(X, Y), 0.0, places=10,
                               msg="X · Y deve ser 0")
        self.assertAlmostEqual(np.dot(X, Z), 0.0, places=10,
                               msg="X · Z deve ser 0")
        self.assertAlmostEqual(np.dot(Y, Z), 0.0, places=10,
                               msg="Y · Z deve ser 0")

        # Unitários
        for eixo, nome in [(X, "X"), (Y, "Y"), (Z, "Z")]:
            self.assertAlmostEqual(np.linalg.norm(eixo), 1.0, places=10,
                                   msg=f"|{nome}| deve ser 1")

    def test_base_plano_inclinado(self):
        """Base construída a partir de normal (1,1,1)/√3 também é ortonormal."""
        normal = np.array([1.0, 1.0, 1.0])
        normal = normal / np.linalg.norm(normal)
        X, Y, Z = construir_base_mesa(normal)

        self.assertAlmostEqual(np.dot(X, Y), 0.0, places=10)
        self.assertAlmostEqual(np.dot(X, Z), 0.0, places=10)
        self.assertAlmostEqual(np.dot(Y, Z), 0.0, places=10)

    def test_z_mesa_igual_normal(self):
        """O eixo Z da mesa deve coincidir com a normal fornecida."""
        normal = np.array([0.0, 0.0, 1.0])
        _, _, Z = construir_base_mesa(normal)
        self.assertTrue(arrays_proximos(Z, normal))


# =====================================================================
# 2b. TESTES — Matriz de Transformação 4×4
# =====================================================================

class TestMatrizTransformacao(unittest.TestCase):
    """Testa a montagem e aplicação da matriz afim 4×4."""

    def test_identidade_quando_base_canonica_e_origem_zero(self):
        """
        Se a base da mesa coincide com a base canônica e a origem é (0,0,0),
        a matriz deve ser a identidade 4×4  →  pontos não mudam.

        R = I₃,  t = -I₃ · (0,0,0) = (0,0,0)  →  T = I₄
        """
        X = np.array([1.0, 0.0, 0.0])
        Y = np.array([0.0, 1.0, 0.0])
        Z = np.array([0.0, 0.0, 1.0])
        origem = np.array([0.0, 0.0, 0.0])

        T = montar_matriz_transformacao(X, Y, Z, origem)
        self.assertTrue(arrays_proximos(T, np.eye(4)),
                        "Deveria ser identidade quando base = canônica e origem = 0")

    def test_translacao_anula_origem(self):
        """
        Base canônica com origem em (10, 20, 30).
        A translação deve deslocar essa origem para (0, 0, 0) no novo referencial.

        t = -R · origem = -I₃ · (10,20,30) = (-10,-20,-30)
        Então: T · [10, 20, 30, 1]ᵀ = [0, 0, 0, 1]ᵀ
        """
        X = np.array([1.0, 0.0, 0.0])
        Y = np.array([0.0, 1.0, 0.0])
        Z = np.array([0.0, 0.0, 1.0])
        origem = np.array([10.0, 20.0, 30.0])

        T = montar_matriz_transformacao(X, Y, Z, origem)

        # Transformar a própria origem → deve virar (0,0,0)
        p_origem = np.array([[10.0, 20.0, 30.0]])
        resultado = transformar_pontos(T, p_origem)
        self.assertTrue(
            arrays_proximos(resultado[0], [0, 0, 0]),
            f"Origem transformada deveria ser (0,0,0), obteve {resultado[0]}",
        )

    def test_ponto_no_plano_tem_z_zero(self):
        """
        Pontos sobre o plano z = 5 transformados para o referencial da mesa
        devem ter Z_mesa ≈ 0, já que o plano se torna o "chão" da mesa.

        Passo a passo:
          1. Ajustar plano em z = 5  →  normal = (0,0,1), centroide ≈ (*, *, 5)
          2. Construir base → Z_mesa = (0,0,1)
          3. Montar T  →  t_z = -1·5 = -5
          4. T · [x, y, 5, 1]ᵀ  →  z_mesa = 5 - 5 = 0  ✓
        """
        pontos = np.array([
            [0, 0, 5], [1, 0, 5], [0, 1, 5], [1, 1, 5],
            [3, 2, 5], [-1, -1, 5],
        ], dtype=float)

        normal, d, centroide = ajustar_plano_svd(pontos)
        X, Y, Z = construir_base_mesa(normal)
        T = montar_matriz_transformacao(X, Y, Z, centroide)

        transformados = transformar_pontos(T, pontos)

        # Todos os z no referencial da mesa devem ser ≈ 0
        for i, pt in enumerate(transformados):
            self.assertAlmostEqual(
                pt[2], 0.0, places=4,
                msg=f"Ponto {i}: z_mesa = {pt[2]}, deveria ser 0 (ponto no plano)",
            )


# =====================================================================
# 3. TESTES — Captura 2D de Grid (Chessboard)
# =====================================================================

class TestDeteccaoTabuleiro(unittest.TestCase):
    """Testa a detecção de cantos de tabuleiro de xadrez."""

    def test_imagem_sem_tabuleiro(self):
        """Uma imagem totalmente branca não contém tabuleiro → encontrado = False."""
        imagem_branca = np.ones((480, 640, 3), dtype=np.uint8) * 255
        encontrado, cantos = encontrar_cantos_tabuleiro(imagem_branca, (7, 5))
        self.assertFalse(encontrado)
        self.assertIsNone(cantos)

    def test_imagem_sintetica_com_tabuleiro(self):
        """
        Gera uma imagem sintética de tabuleiro 8×6 (7×5 cantos internos)
        e verifica que os cantos são detectados corretamente.
        """
        # Criar tabuleiro sintético
        tamanho_quadrado = 40
        colunas, linhas = 8, 6  # quadrados  →  cantos internos = (7, 5)
        img_h = linhas * tamanho_quadrado + 80
        img_w = colunas * tamanho_quadrado + 80
        imagem = np.ones((img_h, img_w), dtype=np.uint8) * 255

        offset_x, offset_y = 40, 40
        for r in range(linhas):
            for c in range(colunas):
                if (r + c) % 2 == 0:
                    x0 = offset_x + c * tamanho_quadrado
                    y0 = offset_y + r * tamanho_quadrado
                    imagem[y0:y0 + tamanho_quadrado, x0:x0 + tamanho_quadrado] = 0

        encontrado, cantos = encontrar_cantos_tabuleiro(imagem, (7, 5))

        self.assertTrue(encontrado, "Deveria detectar o tabuleiro sintético")
        self.assertIsNotNone(cantos)
        # 7 × 5 = 35 cantos internos
        self.assertEqual(cantos.shape[0], 35,
                         f"Esperados 35 cantos, obteve {cantos.shape[0]}")


# =====================================================================
# 4. TESTES — Projeção Tsai (cv2.projectPoints)
# =====================================================================

class TestProjecaoTsai(unittest.TestCase):
    """Testa a projeção 3D → 2D pelo modelo de câmera via cv2.projectPoints."""

    def test_ponto_na_origem_projeta_no_ponto_principal(self):
        """
        Ponto 3D na origem (0,0,0) do referencial da mesa,
        com rotação = identidade e translação = (0,0,1):

        O ponto na câmera fica em (0,0,1).
        Projeção pinhole:
            u = fx * (X/Z) + cx = 320 * (0/1) + 320 = 320
            v = fy * (Y/Z) + cy = 240 * (0/1) + 240 = 240

        → pixel esperado = (320, 240)  (centro da imagem)
        """
        ponto_3d = np.array([[0.0, 0.0, 0.0]])
        rvec = np.zeros((3, 1))                # sem rotação
        tvec = np.array([[0.0, 0.0, 1.0]])     # translação 1m em z

        camera_matrix = np.array([
            [320.0,   0.0, 320.0],
            [  0.0, 240.0, 240.0],
            [  0.0,   0.0,   1.0],
        ])

        pixels = projetar_pontos_tsai(ponto_3d, rvec, tvec, camera_matrix)

        self.assertAlmostEqual(pixels[0, 0], 320.0, places=2,
                               msg=f"u deveria ser 320, obteve {pixels[0, 0]}")
        self.assertAlmostEqual(pixels[0, 1], 240.0, places=2,
                               msg=f"v deveria ser 240, obteve {pixels[0, 1]}")

    def test_ponto_deslocado_em_x(self):
        """
        Ponto (1, 0, 0) com tvec = (0, 0, 2):
        No referencial da câmera:  (1, 0, 2).
        Projeção:
            u = fx * (1/2) + cx = 320 * 0.5 + 320 = 480
            v = fy * (0/2) + cy = 240 * 0   + 240 = 240

        → pixel esperado = (480, 240)
        """
        ponto_3d = np.array([[1.0, 0.0, 0.0]])
        rvec = np.zeros((3, 1))
        tvec = np.array([[0.0, 0.0, 2.0]])

        camera_matrix = np.array([
            [320.0,   0.0, 320.0],
            [  0.0, 240.0, 240.0],
            [  0.0,   0.0,   1.0],
        ])

        pixels = projetar_pontos_tsai(ponto_3d, rvec, tvec, camera_matrix)

        self.assertAlmostEqual(pixels[0, 0], 480.0, places=2,
                               msg=f"u deveria ser 480, obteve {pixels[0, 0]}")
        self.assertAlmostEqual(pixels[0, 1], 240.0, places=2,
                               msg=f"v deveria ser 240, obteve {pixels[0, 1]}")

    def test_multiplos_pontos(self):
        """Projetar vários pontos de uma vez e verificar formas do resultado."""
        pontos_3d = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        rvec = np.zeros((3, 1))
        tvec = np.array([[0.0, 0.0, 5.0]])
        camera_matrix = np.eye(3) * 500
        camera_matrix[2, 2] = 1.0
        camera_matrix[0, 2] = 320.0
        camera_matrix[1, 2] = 240.0

        pixels = projetar_pontos_tsai(pontos_3d, rvec, tvec, camera_matrix)
        self.assertEqual(pixels.shape, (3, 2),
                         "Resultado deve ter shape (N, 2)")


# =====================================================================
# 5. TESTES — Leitura RGBD (Open3D) — importação condicional
# =====================================================================

class TestLeituraRGBD(unittest.TestCase):
    """Verifica a criação de nuvem de pontos (pula se open3d não instalado)."""

    def test_importacao_open3d(self):
        """Testa apenas que o import open3d não falha."""
        try:
            import open3d  # noqa: F401
        except ImportError:
            self.skipTest("Open3D não instalado — teste pulado")


# =====================================================================
# 6. TESTES — Integração MDE + Coloração
# =====================================================================

class TestMDEColoracao(unittest.TestCase):
    """
    Testa a lógica de coloração comparando Z_real (Kinect) com Z_mde.
    Usa um MOCK simples do MDE para provar à banca que a lógica funciona.

    Cores no padrão BGR (OpenCV):
        Vermelho = (0, 0, 255)  →  precisa cavar
        Azul     = (255, 0, 0)  →  precisa colocar areia
        Verde    = (0, 255, 0)  →  OK
    """

    VERMELHO = (0, 0, 255)
    AZUL     = (255, 0, 0)
    VERDE    = (0, 255, 0)

    # ----- Mock do MDE -----
    @staticmethod
    def mde_mock(x: float, y: float) -> float:
        """
        Mock do MDE da Cartografia.
        Simula um terreno com uma rampa suave: z = 10 + 0.5·x
        Em x=0  → z=10,  x=20 → z=20.
        """
        return 10.0 + 0.5 * x

    # ----- Testes de cor_por_diferenca -----

    def test_vermelho_precisa_cavar(self):
        """
        z_mde = 10,  z_real = 20,  tolerância = 5.

        Diferença = 20 - 10 = 10  >  tolerância (5)
        → Há areia DEMAIS → precisa CAVAR → cor VERMELHA.
        """
        cor = cor_por_diferenca(z_real=20.0, z_mde=10.0, tolerancia=5.0)
        self.assertEqual(cor, self.VERMELHO,
                         "z_real muito acima do MDE → deveria ser VERMELHO (cavar)")

    def test_azul_precisa_colocar_areia(self):
        """
        z_mde = 10,  z_real = 3,  tolerância = 5.

        Diferença = 3 - 10 = -7  <  -tolerância (-5)
        → Falta areia → precisa COLOCAR → cor AZUL.
        """
        cor = cor_por_diferenca(z_real=3.0, z_mde=10.0, tolerancia=5.0)
        self.assertEqual(cor, self.AZUL,
                         "z_real muito abaixo do MDE → deveria ser AZUL (colocar areia)")

    def test_verde_zona_ok(self):
        """
        z_mde = 10,  z_real = 12,  tolerância = 5.

        Diferença = 12 - 10 = 2  →  |2| < 5
        → Dentro da faixa de tolerância → cor VERDE.
        """
        cor = cor_por_diferenca(z_real=12.0, z_mde=10.0, tolerancia=5.0)
        self.assertEqual(cor, self.VERDE,
                         "z_real dentro da tolerância → deveria ser VERDE")

    def test_verde_no_limite_exato(self):
        """
        z_real = z_mde + tolerância = 15  →  NÃO é estritamente maior.
        A condição é z_real > z_mde + tol (estritamente), então
        z_real == z_mde + tol  →  cai no 'else' → VERDE.
        """
        cor = cor_por_diferenca(z_real=15.0, z_mde=10.0, tolerancia=5.0)
        self.assertEqual(cor, self.VERDE,
                         "No limite exato (z_real == z_mde + tol) → VERDE")

    # ----- Testes com o Mock do MDE -----

    def test_gerar_mapa_cores_com_mock(self):
        """
        Usa o mde_mock (z = 10 + 0.5·x) com 3 pontos:

          Ponto A: (0, 0, 25)  →  z_mde=10, diff=+15 > 5  → VERMELHO
          Ponto B: (0, 0,  2)  →  z_mde=10, diff=-8  < -5  → AZUL
          Ponto C: (0, 0, 11)  →  z_mde=10, diff=+1  ∈[-5,5] → VERDE
        """
        pontos = np.array([
            [0.0, 0.0, 25.0],   # muito acima → VERMELHO
            [0.0, 0.0,  2.0],   # muito abaixo → AZUL
            [0.0, 0.0, 11.0],   # OK → VERDE
        ])

        cores = gerar_mapa_cores(pontos, funcao_mde=self.mde_mock, tolerancia=5.0)

        np.testing.assert_array_equal(cores[0], list(self.VERMELHO),
                                      err_msg="Ponto A deveria ser VERMELHO")
        np.testing.assert_array_equal(cores[1], list(self.AZUL),
                                      err_msg="Ponto B deveria ser AZUL")
        np.testing.assert_array_equal(cores[2], list(self.VERDE),
                                      err_msg="Ponto C deveria ser VERDE")

    def test_mde_mock_rampa(self):
        """
        Verificar que o mock segue a fórmula z = 10 + 0.5·x.

        Em x = 20:  z_mde = 10 + 0.5×20 = 20.
        Ponto (20, 0, 30):  diff = 30 - 20 = 10 > 5  → VERMELHO
        Ponto (20, 0, 20):  diff = 20 - 20 = 0  ∈ [-5,5]  → VERDE
        Ponto (20, 0, 10):  diff = 10 - 20 = -10 < -5  → AZUL
        """
        pontos = np.array([
            [20.0, 0.0, 30.0],
            [20.0, 0.0, 20.0],
            [20.0, 0.0, 10.0],
        ])

        cores = gerar_mapa_cores(pontos, funcao_mde=self.mde_mock, tolerancia=5.0)

        np.testing.assert_array_equal(cores[0], list(self.VERMELHO))
        np.testing.assert_array_equal(cores[1], list(self.VERDE))
        np.testing.assert_array_equal(cores[2], list(self.AZUL))


# =====================================================================
# TESTES — Pipeline Integrado (Passos 1 + 2)
# =====================================================================

class TestPipeline(unittest.TestCase):
    """Teste de integração: plano → base → transformação."""

    def test_pipeline_plano_z0(self):
        """
        Pontos em z = 0  →  plano horizontal.
        Após transformação, todos os z_mesa devem ser ≈ 0.
        """
        pontos = np.array([
            [0, 0, 0], [5, 0, 0], [0, 5, 0],
            [5, 5, 0], [2, 3, 0],
        ], dtype=float)

        normal, d, centroide, X, Y, Z, T = pipeline_plano_e_base(pontos)

        # Verificar que a normal é (0,0,1)
        self.assertTrue(arrays_proximos(np.abs(normal), [0, 0, 1]))

        # Transformar e verificar z ≈ 0
        transformados = transformar_pontos(T, pontos)
        for pt in transformados:
            self.assertAlmostEqual(pt[2], 0.0, places=4)

    def test_pipeline_plano_z10_ponto_acima(self):
        """
        Plano em z = 10.  Ponto (0, 0, 15) está 5 unidades ACIMA do plano.
        Após transformação, z_mesa ≈ 5.

        Conta:
          centroide_z ≈ 10,  normal ≈ (0,0,1)
          z_mesa = 1·(15) + t_z = 15 - 10 = 5  ✓
        """
        plano = np.array([
            [0, 0, 10], [1, 0, 10], [0, 1, 10],
            [1, 1, 10], [2, 2, 10],
        ], dtype=float)

        _, _, centroide, X, Y, Z, T = pipeline_plano_e_base(plano)

        ponto_acima = np.array([[0.0, 0.0, 15.0]])
        resultado = transformar_pontos(T, ponto_acima)

        self.assertAlmostEqual(
            resultado[0, 2], 5.0, places=3,
            msg=f"z_mesa deveria ser 5 (15-10), obteve {resultado[0, 2]}",
        )


# =====================================================================

if __name__ == "__main__":
    unittest.main(verbosity=2)

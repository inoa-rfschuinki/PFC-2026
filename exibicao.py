import cv2
import numpy as np

# Módulo responsável pela exibição da imagem no projetor.
# Recebe o mapa de profundidade ou a imagem processada
# pelo parceiro e exibe em tela cheia na segunda tela.

def profundidade_para_imagem(profundidade):
    # Normalizar a profundidade para o intervalo 0-255
    profundidade_normalizada = cv2.normalize(profundidade, None, 0, 255, cv2.NORM_MINMAX)
    profundidade_normalizada = profundidade_normalizada.astype(np.uint8)

    return cv2.applyColorMap(profundidade_normalizada, cv2.COLORMAP_TURBO)

def exibir_profundidade(imagem, janela="projetor",tela_cheia=False):
    cv2.namedWindow(janela, cv2.WINDOW_NORMAL)

    if tela_cheia:
        cv2.setWindowProperty(janela, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    cv2.imshow(janela, imagem)
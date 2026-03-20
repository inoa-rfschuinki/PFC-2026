import cv2
import numpy as np
from kinect import simular_kinect,profundidade_para_pontos
from exibicao import profundidade_para_imagem, exibir_profundidade
# Arquivo de testes manuais.
# Rode esse arquivo para verificar se cada módulo
# está funcionando antes de rodar o sistema completo.

def testar_simulador():
    print("Testando Simulador...")
    profundidade = simular_kinect()
    print(f"Profundidade capturada: {profundidade.shape}, tipo: {profundidade.dtype}")
    print(f"Valor mínimo: {profundidade.min()}, valor máximo: {profundidade.max()}")
    print("Centro da colina (320,240):", profundidade[240,320])
    print("Teste do simulador concluído.\n")

def testar_profundidade_para_pontos(profundidade):
    print("Testando conversão de profundidade para pontos...")
    pontos = profundidade_para_pontos(profundidade)
    print(f"Número de pontos válidos: {len(pontos)}")
    print(f"Formato dos pontos: {pontos.shape}, tipo: {pontos.dtype}")
    print("Pontos de exemplo (primeiros 5):")
    print(pontos[:5])
    print("Teste de conversão concluído.\n")

def testar_exibicao(profundidade):
    print("Testando exibição de profundidade...")
    imagem_profundidade = profundidade_para_imagem(profundidade)
    print(f"Imagem de profundidade: {imagem_profundidade.shape}, tipo: {imagem_profundidade.dtype}")
    exibir_profundidade(imagem_profundidade, janela="Teste de Exibição", tela_cheia=False)
    print("Feche a janela para continuar...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("Teste de exibição concluído.\n")

if __name__ == "__main__":
    testar_simulador()
    profundidade = simular_kinect()
    testar_profundidade_para_pontos(profundidade)
    testar_exibicao(profundidade)
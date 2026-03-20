import cv2
from kinect import capturar_profundidade, profundidade_para_pontos
from exibicao import profundidade_para_imagem, exibir_profundidade
from utils import Cronometro, log

# Ponto de entrada do sistema.
# Orquestra o loop principal: captura → processa → exibe.
# Não contém lógica própria — apenas chama os módulos.

def main():
    print("Sistema Iniciado. Pressione Q para sair.")
    cronometro= Cronometro()
    while True:
        #1. capturar profundidade
        profundidade= capturar_profundidade()
        #2. converter para pontos 3D
        pontos= profundidade_para_pontos(profundidade)
        #3. exibir imagem de profundidade
        imagem_profundidade= profundidade_para_imagem(profundidade)
        exibir_profundidade(imagem_profundidade)
        #4. log de fps e pontos válidos
        fps=cronometro.marcar()
        log(f"FPS: {fps:.2f}", pontos)
        #sai se o usuário pressionar Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    print("\nSistema Encerrado.")

if __name__ == "__main__":
    main()
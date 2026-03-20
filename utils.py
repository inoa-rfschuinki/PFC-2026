import time

# Módulo de funções auxiliares gerais do projeto.
# Contém utilitários de tempo, log e formatação
# que são usados por mais de um módulo.

class Cronometro:
    def __init__(self):
        self.inicio = time.time()

    def marcar(self):
        agora=time.time()
        fps=1.0/(agora-self.inicio)
        self.inicio=agora
        return fps
    
    def parar(self):
        self.fim = time.time()

    def tempo_decorrido(self):
        if self.inicio is None or self.fim is None:
            raise ValueError("Cronômetro não foi iniciado ou parado corretamente.")
        return self.fim - self.inicio
    
    
def log(mensagem, pontos=None):
    """Imprime status do sistema na mesma linha."""
    if pontos is not None:
        print(f"{mensagem} | Pontos válidos: {len(pontos)}", end="\r")
    else:
        print(mensagem, end="\r")
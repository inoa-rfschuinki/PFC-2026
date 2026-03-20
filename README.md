# PFC 2026 — Caixão de Areia com Realidade Aumentada

**Projeto Final de Curso — Engenharia Eletrônica**  
**Academia Militar das Agulhas Negras (AMAN)**

---

## Sobre este repositório

Este repositório contém o código desenvolvido para a parte de **captura e exibição** do sistema de caixão de areia interativo com realidade aumentada.

O sistema lê o relevo modelado na areia por meio do sensor Microsoft Kinect, converte os dados em uma nuvem de pontos 3D e exibe uma projeção topográfica colorida diretamente sobre a superfície da areia.

---

## Estrutura do repositório

```
.
├── main.py                 # loop principal — ponto de entrada do sistema
├── kinect.py               # captura de profundidade e conversão para pontos
├── exibicao.py             # exibição da imagem no projetor
├── utils.py                # funções auxiliares (FPS, log)
├── testekinect.py          # testes manuais de cada módulo
│
├── matlab/
│   ├── script01_gravidade.m
│   ├── script02_ajuste_polinomial_2d.m
│   ├── script03_calibracao_3d.m
│   └── script04_recursivo.m
│
├── relatorio/
│   └── PFC__Projeto_Final.pdf
│
├── estudos/
│   └── ESTUDOS.md
│
└── README.md
```

---

## O que cada arquivo faz

| Arquivo | Responsabilidade |
|---|---|
| `main.py` | Orquestra o loop principal — captura, processa e exibe |
| `kinect.py` | Tudo relacionado ao sensor: simulador e captura real |
| `exibicao.py` | Tudo relacionado à tela: colorização e exibição |
| `utils.py` | Funções de apoio: medição de FPS e log no terminal |
| `testekinect.py` | Testes isolados de cada módulo |

---

## Como rodar

### Pré-requisitos

```bash
pip install numpy opencv-python
```

Para uso com o Kinect real (Linux):

```bash
sudo apt-get install freenect python3-freenect
```

### Rodando o simulador

```bash
python testekinect.py    # testa cada módulo isoladamente
python main.py           # roda o sistema completo
```

Pressione **Q** para encerrar.

### Rodando com o Kinect real

No arquivo `kinect.py`, mude a linha:

```python
USE_KINECT_REAL = False
# para:
USE_KINECT_REAL = True
```

---

## Interface com o Eng. de Computação

A variável `pontos` no loop da `main.py` contém o array entregue para o parceiro:

```
shape: (N, 3)
cada linha: [u, v, d]

u → coluna em pixels  (0 a 639)
v → linha em pixels   (0 a 479)
d → profundidade em mm
```

O formato de entrega (socket, arquivo, memória compartilhada) será definido em conjunto conforme o projeto avança.

---

## Modo atual

- ✅ Simulador funcionando
- ✅ Conversão depth → pontos funcionando  
- ✅ Exibição colorida funcionando
- 🔄 Kinect real — aguardando hardware
- 🔄 Interface com Eng. de Computação — a definir

---

## Referências

- OpenKinect / libfreenect: https://github.com/OpenKinect/libfreenect
- SARndbox: https://github.com/KeckCAVES/SARndbox
- AR Sandbox DIY: https://ar-sandbox.eu/augmented-reality-sandbox-diy/
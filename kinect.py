import numpy as np
import cv2

# Módulo responsável pela captura de dados do sensor Kinect.
# Em modo simulado, gera um mapa de profundidade falso com
# o mesmo formato que o Kinect real entregaria (480x640, uint16).
# Quando o hardware chegar, basta adicionar a função real
# sem alterar o restante do projeto.

USE_KINECT_REAL = False
def simular_kinect():
    profundidade= np.full((480,640),1000,dtype=np.float32)
    
    #simular uma colina no centro
    u=np.arange(640)
    v=np.arange(480)
    uu,vv=np.meshgrid(u,v)

    colina = 150 * np.exp(
            -(((uu - 320)**2) / (2 * 120**2) +
            ((vv - 240)**2) / (2 * 90**2))
        )
    profundidade -= colina

    #adicionar ruido como o sensor real teria
    ruido = np.random.normal(0, 3, profundidade.shape)

    return profundidade.astype(np.uint16)


def capturar_profundidade():
    if USE_KINECT_REAL:
        import freenect
        profundidade,_=freenect.sync_get_depth()
        return profundidade
    else:
        return simular_kinect()
    
def profundidade_para_pontos(profundidade):
    v_idx,u_idx = np.indices(profundidade.shape)
    pontos=np.stack([
        u_idx.flatten(),
        v_idx.flatten(),
        profundidade.flatten()
    ], axis=1)

    return pontos[pontos[:,2]>0]  #filtrar pontos com profundidade válida


import cv2
import numpy as np

# Função vazia para o trackbar
def nothing(x):
    pass

# Carrega a imagem
imagem = cv2.imread('cavalo.png')
print(imagem.shape)
hsv = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)

# Cria uma janela
cv2.namedWindow('Controle de Cor')

# Cria trackbars para ajustar os limites HSV
cv2.createTrackbar('H Min','Controle de Cor',0,179,nothing)
cv2.createTrackbar('H Max','Controle de Cor',179,179,nothing)
cv2.createTrackbar('S Min','Controle de Cor',0,255,nothing)
cv2.createTrackbar('S Max','Controle de Cor',255,255,nothing)
cv2.createTrackbar('V Min','Controle de Cor',0,255,nothing)
cv2.createTrackbar('V Max','Controle de Cor',255,255,nothing)

while True:
    # Lê os valores das trackbars
    h_min = cv2.getTrackbarPos('H Min','Controle de Cor')
    h_max = cv2.getTrackbarPos('H Max','Controle de Cor')
    s_min = cv2.getTrackbarPos('S Min','Controle de Cor')
    s_max = cv2.getTrackbarPos('S Max','Controle de Cor')
    v_min = cv2.getTrackbarPos('V Min','Controle de Cor')
    v_max = cv2.getTrackbarPos('V Max','Controle de Cor')

    # Define limites HSV
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Cria máscara e resultado
    mask = cv2.inRange(hsv, lower, upper)
    resultado = cv2.bitwise_and(imagem, imagem, mask=mask)

    # Mostra resultados
    cv2.imshow('Controle de Cor', resultado)

    # Sai com ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()

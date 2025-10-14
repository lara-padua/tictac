import cv2
import numpy as np
import imutils

# Inicializa câmera
cap = cv2.VideoCapture(0)

def detectar_forma(contorno):
    forma = "Desconhecida"
    perimetro = cv2.arcLength(contorno, True)
    approx = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
    vertices = len(approx)

    if vertices == 3:
        forma = "Triângulo"
    elif vertices == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        aspect_ratio = w / float(h)
        forma = "Quadrado" if 0.95 <= aspect_ratio <= 1.05 else "Retângulo"
    elif vertices > 4:
        forma = "Círculo"

    return forma

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = imutils.resize(frame, width=600)

    # Aplicar blur para reduzir ruído
    blur = cv2.GaussianBlur(frame, (5, 5), 0)

    # Usar Canny diretamente na imagem colorida
    edges = cv2.Canny(blur, 50, 150)

    # Encontrar contornos
    cnts, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        area = cv2.contourArea(c)
        if area > 500:  # ignora ruídos pequenos
            forma = detectar_forma(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                cv2.putText(frame, forma, (cx - 20, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("Reconhecimento de Formas", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

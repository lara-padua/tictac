#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
import imutils
import cv2
import time

# Funções auxiliares
def criar_mascara(hsv, lower, upper, kernel):
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    return mask

def detect_color(mask, frame, color_name, color_bgr, offset, area_min=1000):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < area_min:
        return None

    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.intp(box + offset)

    M = cv2.moments(c)
    if M["m00"] != 0:
        center = (int(M["m10"] / M["m00"]) + offset[0],
                  int(M["m01"] / M["m00"]) + offset[1])
        cv2.drawContours(frame, [box], 0, color_bgr, 2)
        cv2.putText(frame, color_name, center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
        return color_name
    
    return None

def corrigir_iluminacao(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    l_eq = clahe.apply(l)
    lab_eq = cv2.merge((l_eq, a, b))
    frame_clahe = cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)
    frame_norm = cv2.normalize(frame_clahe, None, 0, 255, cv2.NORM_MINMAX)
    return frame_norm

# Limites de cores
color_ranges = {
    "Verde":    ((31, 40, 100), (85, 255, 255), (0, 255, 0)),
    "Amarelo":  ((21, 50, 50),  (30, 255, 255), (0, 255, 255)),
    "Azul":     ((86, 100, 50), (125, 225, 255), (255, 0, 0)),
    "Vermelho":((0, 120, 70),  (10, 255, 255), (0, 0, 255)),
    "Vermelho":((170, 120, 70),(180, 255, 255), (0, 0, 255)),
    "Laranja":  ((11, 100, 50), (20, 255, 255), (0, 165, 255)),
    "Rosa":     ((140, 100, 50),(169, 255, 255),(255, 0, 255)),
}

def color_node():
    rospy.init_node('color_node', anonymous=True)
    pub = rospy.Publisher('color_detected', String, queue_size=10)
    rate = rospy.Rate(10)

    camera = cv2.VideoCapture(0)
    cor_em_potencial = None
    tempo_potencial = None
    cor_fixa = None
    tempo_inicio = None
    tempo_espera = 5
    tempo_pre_detec = 3
    kernel = np.ones((5, 5), np.uint8)

    while not rospy.is_shutdown():
        grabbed, frame = camera.read()
        if not grabbed:
            continue

        frame = imutils.resize(frame, width=600)
        (h, w) = frame.shape[:2]
        roi_size = 200
        cx, cy = w // 2, h // 2
        x1, y1 = cx - roi_size // 2, cy - roi_size // 2
        x2, y2 = cx + roi_size // 2, cy + roi_size // 2
        roi = frame[y1:y2, x1:x2]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1)

        roi_corrigido = corrigir_iluminacao(roi)
        hsv = cv2.cvtColor(roi_corrigido, cv2.COLOR_BGR2HSV)

        cores_detectadas = set()

        if cor_fixa is None:
            for nome, (lower, upper, bgr) in color_ranges.items():
                mask = criar_mascara(hsv, np.array(lower), np.array(upper), kernel)
                cor = detect_color(mask, frame, nome, bgr, (x1, y1))
                if cor:
                    cores_detectadas.add(cor)

            if cores_detectadas:
                cor_atual = list(cores_detectadas)[0]
                if cor_em_potencial != cor_atual:
                    cor_em_potencial = cor_atual
                    tempo_potencial = time.time()
                else:
                    if time.time() - tempo_potencial >= tempo_pre_detec:
                        cor_fixa = cor_em_potencial
                        tempo_inicio = time.time()
                        rospy.loginfo(f"Cor fixada: {cor_fixa}")
                        pub.publish(cor_fixa)  # 🚀 Aqui publicamos no ROS
                        cor_em_potencial = None
                        tempo_potencial = None
            else:
                cor_em_potencial = None
                tempo_potencial = None
        else:
            if time.time() - tempo_inicio >= tempo_espera:
                cor_fixa = None
                tempo_inicio = None
            else:
                cv2.putText(frame, f"Cor fixa: {cor_fixa}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        rate.sleep()

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        color_node()
    except rospy.ROSInterruptException:
        pass

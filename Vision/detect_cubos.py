import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while True:
    # Leer un frame desde la webcam
    ret, frame = cap.read()


    # Convertir a escala de grises
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplicar suavizado Gaussiano
    gauss = cv2.GaussianBlur(gris, (5,5), 0)

    # Detectar bordes con Canny
    canny = cv2.Canny(gauss, 50, 150)
    canny = cv2.dilate(canny, None, iterations=1)
    
    # Buscar contornos
    (contornos,_) = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujar los contornos en la imagen original
    for c in contornos:
        area = cv2.contourArea(c)
        #x,y,w,h = cv2.boundingRect(c)
        epsilon = 0.09*cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,epsilon,True)
        if (len(approx)==6) and area > 1000:
            print("aprox",approx)
            print("Area:", area)
            cv2.drawContours(frame, [c], 0,(0,255,0),2)
            #print("Contornos:", c)
            tl, tr, br, bl,cr,cl = approx
            print("tl", tl[0][0],tl[0][1])
            print("tr", tr[0][0],tr[0][1])
            print("br", br[0][0],br[0][1])
            print("bl", bl[0][0],bl[0][1])
            print("cr", cr[0][0],cr[0][1])
            print("cl", cl[0][0],cl[0][1])
            tl = (int(tl[0][0]), int(tl[0][1]))
            tr = (int (tr[0][0]), int(tr[0][1]))
            br = (int (br[0][0]), int(br[0][1]))
            bl = (int (bl[0][0]), int(bl[0][1]))
            cr = (int (cr[0][0]), int(cr[0][1]))
            cl = (int (cl[0][0]), int(cl[0][1]))
            xmayor = max(tl[0], tr[0], br[0], bl[0], cr[0], cl[0])
            ymayor = max(tl[1], tr[1], br[1], bl[1], cr[1], cl[1])
            xmenor = min(tl[0], tr[0], br[0], bl[0], cr[0], cl[0])
            ymenor = min(tl[1], tr[1], br[1], bl[1], cr[1], cl[1])
            
            cubo = frame[ymenor:ymayor,xmenor:xmayor]
            cv2.imshow("Cubo", cubo)
            # cubo = frame[y:y+h,x:x+w]
            # cv2.imshow("Cubo", cubo)
        if (len(approx)==4) and area > 9000:
            print("aprox",approx)
            print("Area:", area)
            cv2.drawContours(frame, [c], 0,(0,255,0),2)
            #print("Contornos:", c)
            tl, tr, br, bl = approx
            print("tl", tl[0][0],tl[0][1])
            print("tr", tr[0][0],tr[0][1])
            print("br", br[0][0],br[0][1])
            print("bl", bl[0][0],bl[0][1])
            tl = (int(tl[0][0]), int(tl[0][1]))
            tr = (int (tr[0][0]), int(tr[0][1]))
            br = (int (br[0][0]), int(br[0][1]))
            bl = (int (bl[0][0]), int(bl[0][1]))

            xmayor = max(tl[0], tr[0], br[0], bl[0])
            ymayor = max(tl[1], tr[1], br[1], bl[1])
            xmenor = min(tl[0], tr[0], br[0], bl[0])
            ymenor = min(tl[1], tr[1], br[1], bl[1])
            
            cubo = frame[ymenor:ymayor,xmenor:xmayor]
            cv2.imshow("Cubo", cubo)
            # cubo = frame[y:y+h,x:x+w]
            # cv2.imshow("Cubo", cubo)    

    # Mostrar el resultado en una ventana
    #frame = cv2.resize(frame, (0, 0), fx = 0.3, fy  = 0.3)
    cv2.imshow('webcam', frame)

# Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
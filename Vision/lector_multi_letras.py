#!/usr/bin/env python3

import cv2
import easyocr
import numpy as np
import time

import warnings
warnings.filterwarnings("ignore", category=UserWarning)

def ordenar_puntos(puntos):
    n_puntos = np.concatenate([puntos[0],puntos[1], puntos[2],puntos[3]]).tolist()

    y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])

    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])

    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])

    return [x1_order[0], x1_order[1],x2_order[0], x2_order[1]]

def cleanup_text(text): 
    return "".join([c if ord(c) < 128 else "" for c in text]).strip()

reader = easyocr.Reader(["en"],gpu=True)

cap = cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()
    
    if ret == False:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray,300,150)
    canny = cv2.dilate(canny,None, iterations=1)
    cv2.imshow("Canny", canny)
    cnts = cv2.findContours(canny,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts = sorted(cnts, key = cv2.contourArea, reverse=True)[:5]

    for c in cnts:
        epsilon = 0.01*cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,epsilon,True)

        if len(approx)==4:
            cv2.drawContours(frame, [approx], 0,(0,255,255),2)
            puntos = ordenar_puntos(approx)

            cv2.circle(frame, tuple(puntos[0]), 7, (255,0,0),2)
            cv2.circle(frame, tuple(puntos[1]), 7, (0,255,0),2)
            cv2.circle(frame, tuple(puntos[2]), 7, (0,0,255),2)
            cv2.circle(frame, tuple(puntos[3]), 7, (255,255,0),2)
            pts1 = np.float32(puntos)
            pts2 = np.float32([[0,0],[270,0],[0,270],[270,270]])
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(gray,M,(270,310))
            dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
            cv2.imshow('dst',dst)
            result = reader.readtext(dst)

            # for res in result:
            #     print(res[1])

            for bounding_box, text, prob in result:

                text = cleanup_text(text)
                if text >= "A" and text <= "I" or text >="a" and text <= "i":
                    print(f'{prob:0.4f} : {text}')
                    tl, tr, br, bl = bounding_box
                    tl = (int(tl[0]), int(tl[1]))
                    tr = (int (tr[0]), int(tr[1]))
                    br = (int (br[0]), int(br[1]))
                    bl = (int (bl[0]), int(bl[1]))
                    cv2.circle(frame, tuple(puntos[0]), 7, (255,0,0),2)
                    cv2.circle(frame, tuple(puntos[1]), 7, (0,255,0),2)


    frame = cv2.resize(frame, (0, 0), fx = 0.3, fy = 0.3)
    cv2.imshow('Frame',frame)
    k = cv2.waitKey(1) & 0xFF
    if k==27:
        break
cap.release()
cv2.destroyAllWindows()
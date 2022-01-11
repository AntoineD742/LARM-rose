import cv2
import numpy as np

#Constantes seuils
lowBadge=np.array([0, 35, 100]) 
hiBadge=np.array([6, 255, 255])

lowRuban = np.array([0, 0, 230]) 
hiRuban = np.array([255, 100, 255])

#Init cam
cap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')

while cap.isOpened():
    
    #Recup Frame
    ret, frame=cap.read()
    #Transformation en hsv
    img_hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Mask Badge (Double seuillage puis amelioration)
    maskBadge=cv2.inRange(img_hsv, lowBadge, hiBadge)
    maskBadge=cv2.erode(maskBadge, None, iterations=1)
    maskBadge=cv2.dilate(maskBadge, None, iterations=1)

    #Mask Ruban (Double seuillage puis amelioration)
    maskRuban=cv2.inRange(img_hsv, lowRuban, hiRuban)
    maskRuban=cv2.erode(maskRuban, None, iterations=1)
    maskRuban=cv2.dilate(maskRuban, None, iterations=1)

    #Combinaison des masks

    mask = cv2.add(maskBadge, maskRuban)

    #Extraction des zones d'interets
    img_result=cv2.bitwise_and(frame, frame, mask= mask)

    #FILLING HOLE ET ELIMINATION DES AUTRES FORMES ???

    #Affichage
    cv2.imshow('Camera',frame) 
    cv2.waitKey(1)  
    cv2.imshow('Result',img_result)  
    cv2.waitKey(1)
cap.release()
cv2.destroyAllWindows()
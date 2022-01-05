import cv2
import numpy as np
#import os
#from sklearn.svm import LinearSVC
#from scipy.cluster.vq import *
#from sklearn.preprocessing import StandardScaler
#from sklearn import preprocessing

#cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")



#Detecteur
object_cascade=cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

print(cv2.data.haarcascades)

#Init cam
cap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')


while cap.isOpened():
    #Recup Frame
    ret, frame=cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #Detection
    object=object_cascade.detectMultiScale(gray, scaleFactor=1.10, minNeighbors=4)
    
    #Affichage detection
    for x, y, w, h in object:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
    
    #Affichage
    cv2.imshow('Camera', frame)
   # cv2.imshow('GrayScale', gray)

    #Break
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
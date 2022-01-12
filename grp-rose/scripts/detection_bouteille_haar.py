import os
import cv2
import numpy
def generate_negative_description_file():
    with open('neg.txt', 'w') as f:
        for filename in os.listdir('./src/data/negatifs/'):
            f.write('negatifs/' + filename + '\n')

#cascade_name=cv2.CascadeClassifier('cascade.xml')
object_cascade=cv2.CascadeClassifier('bottle.xml')
print(os.path.exists('bottle.xml'))
#object_cascade=cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
#if not object_cascade.load(cascade_name):
#    print('--(!)Error loading face cascade')
#    exit(0)


#Init cam
cap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')

while cap.isOpened():
    
    #Recup Frame
    ret, frame=cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    cv2.imshow('Camera', frame)
    cv2.waitKey(1)
    #cv2.imshow('GrayScale', gray)
    #cv2.waitKey(1)

    #Affichage
    #cv2.imshow('GrayScale', gray)
    #Detection
    objecttest=object_cascade.detectMultiScale(gray, scaleFactor=1.10, minNeighbors=1)

    #Affichage detection
    for x, y, w, h in objecttest:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

    cv2.imshow('Camera', frame)
    cv2.waitKey(1)
    #Break
    #if cv2.waitKey(1)&0xFF==ord('q'):
    #    break
cap.release()
cv2.destroyAllWindows()


import cv2
import numpy as np




#Constantes

#color=223

#lo=np.array([0, 200, color-20])
#hi=np.array([20, 255,color+20])
lowBadge=np.array([0, 35, 100]) #5 51 238
hiBadge=np.array([6, 255, 255])

#lowRuban = np.array([0, 0, 230]) 
#hiRuban = np.array([255, 100, 255])


lowRuban = np.array([0, 0, 230]) 
hiRuban = np.array([255, 100, 255])


#Recup image
img_color = cv2.imread("../data/bouteilles/bouteille3.jpg")


#calculs

#print(img_hsv[401][550])#Y X --> BGR






while True:
    img_hsv=cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)
    #Badge
    #mask=cv2.inRange(img_hsv, lowBadge, hiBadge)
    #mask=cv2.erode(mask, None, iterations=1)
    #mask=cv2.dilate(mask, None, iterations=1)
    #img_result=cv2.bitwise_and(img_color, img_color, mask= mask)

    #Ruban
    #mask2=cv2.inRange(img_hsv, lowRuban, hiRuban)
    #mask2=cv2.erode(mask2, None, iterations=1)
    #mask2=cv2.dilate(mask2, None, iterations=1)
    #img_result2=cv2.bitwise_and(img_color, img_color, mask= mask2)
    

    #Mask Badge 
    maskBadge=cv2.inRange(img_hsv, lowBadge, hiBadge)
    maskBadge=cv2.erode(maskBadge, None, iterations=1)
    maskBadge=cv2.dilate(maskBadge, None, iterations=1)


    #Mask Ruban
    maskRuban=cv2.inRange(img_hsv, lowRuban, hiRuban)
    maskRuban=cv2.erode(maskRuban, None, iterations=1)
    maskRuban=cv2.dilate(maskRuban, None, iterations=1)

    #Add des mask

    mask = cv2.add(maskBadge, maskRuban)

    img_result=cv2.bitwise_and(img_color, img_color, mask= mask)

    #Affichage
    cv2.imshow('color image',img_color)  
    cv2.imshow('HSV image',img_hsv)  
    cv2.imshow('Mask Ruban',maskRuban)  
    cv2.imshow('Mask Badge',maskBadge)  
    cv2.imshow('Mask Resultat',mask)  
    cv2.imshow('Result',img_result)  
    #cv2.imshow('Result2',img_result2)  
    
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
# Destroys all the windows created
cv2.destroyAllwindows() 

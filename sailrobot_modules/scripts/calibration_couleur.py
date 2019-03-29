#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 22 14:32:27 2018

@author: corazzal
"""

import cv2

global colorLower_sonde, colorUpper_sonde

############################################
#HSV de la sonde et de l aiguille
colorLower_sonde = (10,150,60)  #10,150,60  16, 114, 175
colorUpper_sonde = (30,255,255) #jaune 30,255,255   33, 255, 255
    
############################################

def nothing(x):
    pass


if __name__ == '__main__':
        
    cap0 = cv2.VideoCapture(0)
    
    known_distance = 0.3
    known_width = 0.04
    
###########################################
    #Selectionner la couleur a calibrer:
  
    #Sonde
    colorLower, colorUpper = colorLower_sonde, colorUpper_sonde

###########################################
    
    Hl = 0
    Sl = 0
    Vl = 0
    Hu = 0
    Su = 0
    Vu = 0
    
    
# =============================================================================
#     Trackbar pour le HSV

    cv2.namedWindow('frame')
    
    cv2.createTrackbar('Hlow','frame',colorLower[0],255,nothing)
    cv2.createTrackbar('Slow','frame',colorLower[1],255,nothing)
    cv2.createTrackbar('Vlow','frame',colorLower[2],255,nothing)
    
    cv2.createTrackbar('Hup','frame',colorUpper[0],255,nothing)
    cv2.createTrackbar('Sup','frame',colorUpper[1],255,nothing)
    cv2.createTrackbar('Vup','frame',colorUpper[2],255,nothing)

    
# =============================================================================
    

    
    while(cap0.isOpened()):
    
        colorLower = (Hl, Sl, Vl)
        colorUpper = (Hu, Su, Vu)
        
        #Lecture image par image
        ret, frame = cap0.read()
        
        #Pré-traitements
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv.copy(), colorLower, colorUpper)
        
        mask = cv2.erode(mask.copy(), None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        
        #Detection des contours
        im, contours, hierarchy = cv2.findContours(mask.copy(),  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)        
        cv2.drawContours(frame, contours, -1, (0,0,255), 3)
        
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            #print("Rayon du cercle en pixels: ", radius)
            cv2.circle(frame,center,radius,(0,255,0),2)
            #print(cv2.minEnclosingCircle(cnt))
            
        if ret:
            # Affichage des images
            cv2.imshow('frame', frame)
            cv2.imshow('hsv', hsv)
            #cv2.imshow('blur', blur)
            cv2.imshow('mask', mask)
            
# =============================================================================
#             Trackbar

            Hl =  cv2.getTrackbarPos('Hlow', 'frame')
            Sl =  cv2.getTrackbarPos('Slow', 'frame')
            Vl =  cv2.getTrackbarPos('Vlow', 'frame')
            
            Hu =  cv2.getTrackbarPos('Hup', 'frame')
            Su =  cv2.getTrackbarPos('Sup', 'frame')
            Vu =  cv2.getTrackbarPos('Vup', 'frame')

# =============================================================================
            
            key = cv2.waitKey(1)
            if  key == ord('q'):
                 break
        else:
            break
        
      
    # Fermeture lorsque la touche 'q' est appuyee
    cap0.release()
    cv2.destroyAllWindows()

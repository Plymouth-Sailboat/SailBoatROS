#!/usr/bin/env python
# license removed for brevity

from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
import numpy as np
from python_class import Interval as py
from python_class import utilities

def Fct_update_rudder(List_dist,thetab,theta,phi,S2):


        if (S2.h==0)&(len(List_dist) > 0)&(np.cos(theta - thetab)< np.cos(pi/3)):

            dmin = List_dist[0][0]
            alpha00 = List_dist[0][1]
            for i in range(0,len(List_dist)):   # define the closest obstacle
                if List_dist[i][0] < dmin:
                    dmin = List_dist[i][0]
                    alpha00 = List_dist[i][1]

            if dmin < S2.dsecuObs: # if obstacle is too close to turn in this direction
                S2.deltarh = -np.abs(pi/4)*np.sign(np.sin(theta-alpha00))
                deltar = S2.deltarh
                S2.h = 1

        elif (S2.h==1)&(np.cos(theta - thetab)< np.cos(pi/3)):
            deltar = S2.deltarh

        else:   # if no obstacle close to the sailboat
            S2.h = 0
            deltar = S2.update_rudder(thetab,theta,phi)

	return deltar


#####################

def Obstacle_avoidance(x,y,thetab,psi,psi_tw,delta,dsecuObs,param2,LObs):


            # test if obstacles
            x2 = array([x,y])
            List_DA2, List_dist = Liste_dead_area2(x2,LObs,psi_tw,delta,dsecuObs,param2)


            if (List_DA2[0][1]-List_DA2[0][0]) > 2*pi:
                List_DA2 = [py.Interval(-pi+0.005,pi-0.005)]

            thetab0 = thetab
            Critmin = 10

            for i in range(0,len(List_DA2)):

               # if thetab in  List_DA2[i] :
                if py.cos(thetab -  List_DA2[i])[1] == 1 :


                    for j in range(0,2):


                            theta1 = List_DA2[i][j]    # test the following angle

                            # evaluation of the criterium
                            Crit = -np.cos(theta1-thetab0) -np.cos(theta1-psi)

                            # test of the criterium
                            if Crit < Critmin:
                                Critmin = Crit
                                if j == 1:
                                    eps1 =  pi/72
                                else:
                                    eps1 = - pi/72
                                thetab = theta1 + eps1


            return thetab, List_dist


###########################



def d_Projection(x,A,B):

    if A[0]==B[0]:
        xh = A[0]
        yh = x[1]

        if  not(yh in py.Interval(np.min([A[1],B[1]]),np.max([A[1],B[1]]))):

            if np.abs(yh - A[1]) > np.abs(yh - B[1]) :
                yh = B[1]
            else:
                yh = A[1]

    if A[1]==B[1]:
        xh = x[0]
        yh = A[1]

        if  not(xh in py.Interval(np.min([A[0],B[0]]),np.max([A[0],B[0]]))):

            if np.abs(xh - A[0]) > np.abs(xh - B[0]) :
                xh = B[0]
            else:
                xh = A[0]

    dh =  utilities.GPSDist(xh, yh, x[0], x[1])  #(((xh-x[0])**2 + (yh - x[1])**2)**(1/2))


    return dh



######

def Liste_dead_area2(x2,LObs,psy,delta,dsecuObs,param2):


    List_DA = []
    List_dist = []


    for i in range(0,len(LObs)):
        Obs = LObs[i]
        DA, Ldist = Test_Dead_area(x2,Obs,dsecuObs,param2)
        List_DA = List_DA + DA
        List_dist = List_dist + Ldist


     # prise en compte du vent
    s = sign(psy) + (1 - np.abs(sign(psy)))
    psyT = psy - s*pi + py.Interval(-delta,delta)
    if psyT[1]> pi:
        psyT1 = py.Interval(psyT[0],pi)
        psyT2 = py.Interval(-pi, psyT[1]-2*pi)
    elif psyT[0]< -pi:
        psyT1 = py.Interval(-pi,psyT[1])
        psyT2 = py.Interval(psyT[0]+2*pi, pi)
    else:
        psyT1 = psyT
        psyT2 = py.Interval(1,0)


    # union of area
    if psyT2 == py.Interval(1,0):
        L = List_DA + [psyT1]
    else:
        L = List_DA + [psyT1] + [psyT2]


    List_DA2 = []

    k = 0
    kmax = 20
    while (not(L == []))&(k < kmax):  # union of area
        k = k+1

        B = L[0]   # take a box of the list
        L = L[1:len(L)]
        test = 0


        for i in range(0,len(L)):
            B2 = L[i]

            if (B.is_subset(B2))|(B.is_empty()):
                test = 2

                break

            if (not(((B)&(B2)).is_empty()))&(not((B).is_empty()))&(not((B2).is_empty())):
                B = B|B2
                test = 1


        if test == 1:
            L = L + [B]

        elif test == 0:
            if not(B.is_empty()):
                List_DA2 = List_DA2 + [B]


    #################

    # To avoid problem at -pi and pi
    for i in range(len(List_DA2)):
            if List_DA2[i][0] < -pi:
                List_DA2[i] = 2*pi + List_DA2[i]

    L = List_DA2
    List_DA2 = []
    k = 0
    kmax = 20
    while (not(L == []))&(k < kmax):   # fusion of interval again
        k = k+1

        B = L[0]   # take a box of the list
        L = L[1:len(L)]
        test = 0


        for i in range(0,len(L)):
            B2 = L[i]

            if (B.is_subset(B2))|(B.is_empty()):
                test = 2
                break

            if (not(((B)&(B2)).is_empty()))&(not((B).is_empty()))&(not((B2).is_empty())):
                B = B|B2
                test = 1


        if test == 1:
            L = L + [B]

        elif test == 0:
            if not(B.is_empty()):
                List_DA2 = List_DA2 + [B]

    ###################


    return List_DA2, List_dist



###########


def Test_Dead_area(x2,Obs,dsecuObs,param2):


    r = param2[2]*1.1


    # edges of obstacle
    A = [Obs[0][0], Obs[1][0]]
    B = [Obs[0][0], Obs[1][1]]
    C = [Obs[0][1], Obs[1][1]]
    D = [Obs[0][1], Obs[1][0]]
#    A = [Obs[0][0], Obs[0][1]]
#    B = [Obs[0][0], Obs[1][1]]
#    C = [Obs[1][0], Obs[1][1]]
#    D = [Obs[1][0], Obs[0][1]]



    # calcule des distances de l obstacle
    d11 = utilities.GPSDist(A[0], B[0], A[1], B[1])  #  ( ((A[0] - x2[0])**2 +  (A[1] - x2[1])**2 )**(1/2) )
    d22 = utilities.GPSDist(B[0], C[0], B[1], C[1])  # ( ((B[0] - x2[0])**2 +  (B[1] - x2[1])**2 )**(1/2) )
    d33 = utilities.GPSDist(C[0], D[0], C[1], D[1])  # ( ((C[0] - x2[0])**2 +  (C[1] - x2[1])**2 )**(1/2) )
    d44 = utilities.GPSDist(D[0], A[0], D[1], A[1])  # ( ((D[0] - x2[0])**2 +  (D[1] - x2[1])**2 )**(1/2) )


    d1 = d_Projection(x2,A,B)
    d2 = d_Projection(x2,B,C)
    d3 = d_Projection(x2,C,D)
    d4 = d_Projection(x2,D,A)

    dmin = np.min([d1,d2,d3,d4, d11,d22,d33,d44]) # shorter distance to the obstacle
    
   
    # center of the obstacle
    xobs = (Obs[0][0] + Obs[0][1])/2
    yobs = (Obs[1][0] + Obs[1][1])/2
 
    
    # obstacle relative orientation
    x20 = array([x2[0],x2[1]])
    if (x20[0] in py.Interval(A[0],D[0]))&(x20[1]<=A[1]):
        alpha10 = pi/2
    elif (x20[0] in py.Interval(A[0],D[0]))&(x20[1]>=B[1]):
        alpha10 = -pi/2
    elif (x20[1] in py.Interval(A[1],B[1]))&(x20[0]<=D[0]):
        alpha10 = 0
    elif (x20[1] in py.Interval(A[1],B[1]))&(x20[0]>=D[0]):
        alpha10 = pi 
    elif (x20[0]>=D[0])&(x20[1]<=D[1]):
        alpha10 = pi*3/4
    elif (x20[0]>=C[0])&(x20[1]>=C[1]):
        alpha10 = -pi*3/4    
    elif (x20[0]<=B[0])&(x20[1]>=B[1]):
        alpha10 = -pi*1/4      
    elif (x20[0]<=A[0])&(x20[1]<=A[1]):
        alpha10 = pi*1/4

    alpha100 = alpha10    


    #### calcul maximal cone 
    alpha = np.arccos(np.min([dmin/dsecuObs,1])) 
    beta = np.arcsin(np.min([r/dsecuObs,1]))
    
    Lmax = alpha100 + (alpha+beta)*py.Interval(-1.1,1.1) 
       
    alpha0 = np.mod(Lmax[0],2*pi)
    alpha1 = np.mod(Lmax[1],2*pi)
    if alpha0 > pi:
        alpha0 = alpha0 - 2*pi
    elif alpha0 < - pi:
        alpha0 = alpha0 + 2*pi
    if alpha1 > pi:
        alpha1 = alpha1 - 2*pi
    elif alpha1 < -pi:
        alpha1 = alpha1 + 2*pi

        
    if alpha0 <= alpha1:
        Lmax = [py.Interval(alpha0,alpha1)] 
        nLmax = 1
    else:
        Lmax = [py.Interval(alpha0,pi+0.005)] + [py.Interval(-pi-0.005,alpha1)]
        nLmax = 2
     
    ###############################
    
    # calcul du cone

    L = []  # creation d une liste vide
    
    
    
    alpha00 =  np.mod( utilities.GPSBearing(x2[0], x2[1], xobs, yobs),2*pi)   # np.mod(np.angle((xobs-x2[0]) +1j*(yobs - x2[1])),2*pi)
    
    # possible dead zone
    alpha1 = np.mod( utilities.GPSBearing(x2[0], x2[1], Obs[0][0], Obs[1][0]),2*pi)   #   np.mod(np.angle((Obs[0][0]-x2[0]) +1j*(Obs[1][0] - x2[1])),2*pi)
    alpha2 =  np.mod( utilities.GPSBearing(x2[0], x2[1], Obs[0][1], Obs[1][0]),2*pi)   #  np.mod(np.angle((Obs[0][1]-x2[0]) +1j*(Obs[1][0] - x2[1])),2*pi) 
    alpha3 =  np.mod( utilities.GPSBearing(x2[0], x2[1], Obs[0][0], Obs[1][1]),2*pi)   #  np.mod(np.angle((Obs[0][0]-x2[0]) +1j*(Obs[1][1] - x2[1])),2*pi) 
    alpha4 =  np.mod( utilities.GPSBearing(x2[0], x2[1], Obs[0][1], Obs[1][1]),2*pi)   #  np.mod(np.angle((Obs[0][1]-x2[0]) +1j*(Obs[1][1] - x2[1])),2*pi)
    

    # cone a eviter autour de la dead zone
    d01 =  utilities.GPSDist(x2[0], x2[1], Obs[0][0] , Obs[1][0] )  #   ( (Obs[0][0] - x2[0])**2 +  (Obs[1][0] - x2[1])**2 )**(1/2) 
    beta1 = np.arcsin(np.min([r/np.max([d01,0.01]),1]))
    d02 =  utilities.GPSDist(x2[0], x2[1], Obs[0][1] , Obs[1][0] )  #( (Obs[0][1] - x2[0])**2 +  (Obs[1][0] - x2[1])**2 )**(1/2) 
    beta2 = np.arcsin(np.min([r/np.max([d02,0.01]),1]))
    d03 =  utilities.GPSDist(x2[0], x2[1], Obs[0][0] , Obs[1][1] )  # ( (Obs[0][0] - x2[0])**2 +  (Obs[1][1] - x2[1])**2 )**(1/2) 
    beta3 = np.arcsin(np.min([r/np.max([d03,0.01]),1]))
    d04 =  utilities.GPSDist(x2[0], x2[1], Obs[0][1] , Obs[1][1] )  # ( (Obs[0][1] - x2[0])**2 +  (Obs[1][1] - x2[1])**2 )**(1/2) 
    beta4 = np.arcsin(np.min([r/np.max([d04,0.01]),1]))


    # calcul de la dead zone etendu
    alpha10 = np.mod(alpha1 - beta1,2*pi) - alpha00
    alpha20 = np.mod(alpha2 - beta2,2*pi) - alpha00
    alpha30 = np.mod(alpha3 - beta3,2*pi) - alpha00
    alpha40 = np.mod(alpha4 - beta4,2*pi) - alpha00

    alpha11 = np.mod(alpha1 + beta1,2*pi) - alpha00
    alpha22 = np.mod(alpha2 + beta2,2*pi) - alpha00
    alpha33 = np.mod(alpha3 + beta3,2*pi) - alpha00
    alpha44 = np.mod(alpha4 + beta4,2*pi) - alpha00
    
    Lalpha = [alpha10,alpha20,alpha30,alpha40,alpha11,alpha22,alpha33,alpha44]
    
    for i in range(0,len(Lalpha)):
        
            a1 = Lalpha[i] 
            if a1 > pi:
                a1 = a1 - 2*pi
            elif a1 < -pi:
                a1 = a1 + 2*pi
                
            if i == 0:
                I2 = py.Interval(a1, a1)
            else:
                I2 = py.Interval(np.min([a1,I2[0]]), np.max([a1,I2[1]]))
            
    I3 = I2 +  alpha00*py.Interval(1,1) 
    
    
    alpha0 = np.mod(I3[0],2*pi)
    alpha1 = np.mod(I3[1],2*pi)
    if alpha0 > pi:
        alpha0 = alpha0 - 2*pi
    elif alpha0 < - pi:
        alpha0 = alpha0 + 2*pi
    if alpha1 > pi:
        alpha1 = alpha1 - 2*pi
    elif alpha1 < -pi:
        alpha1 = alpha1 + 2*pi

        
    if alpha0 <= alpha1:
        I3 = [py.Interval(alpha0,alpha1)] 
        nI3 = 1
    else:
        I3 = [py.Interval(alpha0,pi+0.005)] + [py.Interval(-pi-0.005,alpha1)]
        nI3 = 2


    for i in range(0,nI3):
        for j in range(0,nLmax):
            if not((I3[i]&Lmax[j]).is_empty()):
                I3[i] = I3[i]&Lmax[j]
              

    ##############
    L = I3
    Ldist = [array([dmin, alpha100])]
    
    
    
    if dmin < dsecuObs:
        
        return L, Ldist
  
    # sinon
    return [],[]


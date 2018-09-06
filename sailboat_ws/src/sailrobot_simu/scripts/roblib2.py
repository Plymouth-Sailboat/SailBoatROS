#https://www.ensta-bretagne.fr/jaulin/robmooc.html

import numpy as np
import matplotlib.pyplot as plt
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
#from scipy.linalg import sqrtm,expm,norm,block_diag
#from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial

from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc

from matplotlib.collections import PatchCollection



def angle(x):
    x=x.flatten()
    return arctan2(x[1], x[0])

def adjoint(w):
    w=w.flatten()
    return array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def translate_motif(R,x):
	return R + x*ones((1,R.shape[1]))


def motif_circle3D(r):
    n = 10
    theta = linspace(0, 2*pi, n)
    x = r*cos(theta) + array(n*[0])
    y = r*sin(theta) + array(n*[0])
    z = zeros(n)
    return array([x,y,z])

def motif_auv3D(): #needed by draw_auv3d and sphere
    return array([ [0.0,0.0,10.0,0.0,0.0,10.0,0.0,0.0],
                   [-1.0,1.0,0.0,-1.0,-0.2,0.0,0.2,1.0],
                   [0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0]])


def draw_arrow3D(ax,x,w,col):  # initial point : x ; final point x+w
    x,w=x.flatten(),w.flatten()
    ax.quiver(x[0],x[1],x[2],w[0],w[1],w[2],color=col,lw=1,pivot='tail',length=1)


def plot2D(M,col='black',w=1):
    plot(M[0, :], M[1, :], col, linewidth = w)

def plot3D(ax,M,col='black',w=1):
    ax.plot(M[0, :], M[1, :],M[2, :], col, linewidth = w)

def draw_tank(x,col='darkblue',r=1):
    x=x.flatten()
    M = r*array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=move_motif(M,x[0],x[1],x[2])
    plot2D(M,col,2)

def draw_disk(c,r,ax,col):
    #draw_disk(array([[1],[2]]),0.5,ax,"blue")
    e = Ellipse(xy=c, width=2*r, height=2*r, angle=0)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.7)
    e.set_facecolor(col)
    
    

def draw_box(x1,x2,y1,y2,ax,col): 
    c=array([[x1],[y1]])    
    rect = Rectangle(c, width=x2-x1, height=y2-y1, angle=0)
    rect.set_facecolor(array([0.4,0.3,0.6]))   
    ax.add_patch(rect)
    rect.set_clip_box(ax.bbox)
    rect.set_alpha(0.7)
    rect.set_facecolor(col)    

def draw_polygon(P,ax,col): 
    patches = []     
    patches.append(Polygon(P, True))    
    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4, color=col)
    ax.add_collection(p)

def draw_arrow(x,y,theta,L,col):
    e=0.2
    M1=L*array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M=np.append(M1,[[1,1,1,1,1]],axis=0)
    R=array([[cos(theta),-sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    plot2D(np.dot(R,M),col)


def draw_sailboat(x,deltas,deltar,phi,awind,color,coeff):
    x=x.flatten()
    theta=x[2]

    p6 = 0.5*coeff
    p7 = 0.5*coeff
    p8 = 2*coeff
    p60 = p6*2*coeff

    hull= array([[-p8,p7,2*p7,2*p7,p7,-p8,-p8,-p8],[-p60,-p60,-p60/2,p60/2,p60,p60,-p60,-p60],[1,1,1,1,1,1,1,1]])
    sail= array([[-8*p7,0],[0,0],[1,1]])
    rudder= array([[-1.5*p6,1.5*p6],[0,0],[1,1]])
    R= array([[cos(theta),-sin(theta),x[0]],[sin(theta),cos(theta),x[1]],[0,0,1]])
    Rs= array([[cos(deltas),-sin(deltas),0],[sin(deltas),cos(deltas),0],[0,0,1]])
    Rr= array([[cos(deltar),-sin(deltar),-p8],[sin(deltar),cos(deltar),0],[0,0,1]])
    draw_arrow(x[0]+ coeff*5,x[1],phi, coeff*5,'red')

    plot2D(np.dot(R,hull),'black');
    plot2D(np.dot(R,np.dot(Rs,sail)),'red');
    plot2D(np.dot(R,np.dot(Rr,rudder)),'red');

#    plot2D(R@hull,'black');       
#    plot2D(R@Rs@sail,'red');       
#    plot2D(R@Rr@rudder,'red');   


def tondarray(M):
    if type(M)==float:
        return array([[M]])
    elif type(M)==int:
        return array([[M]])        
    else:
        return M    



def mvnrnd2(x,G): 
    n=len(x)
    x1=x.reshape(n)
    y = np.random.multivariate_normal(x1,G).reshape(n,1)
    return(y)    

def mvnrnd1(G):
    G=tondarray(G)
    n=len(G)
    x=array([[0]] * n)
    return(mvnrnd2(x,G))  


def demo_draw():  
    fig = figure(0)
    ax = fig.add_subplot(111, aspect='equal')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    
    c=array([[5],[0]])
    e = Ellipse(xy=c, width=13.0, height=2.0, angle=45)  
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.9)
    e.set_facecolor(array([0.7,0.3,0.6]))   
    
    rect = Rectangle( (1,1), width=5, height=3)
    rect.set_facecolor(array([0.4,0.3,0.6]))   
    ax.add_patch(rect)    
        
    pause(0.2)    
    draw_tank(array([[-7],[5],[1]]))
    draw_tank(array([[-7],[5],[1]]),'red',0.2)

    
    draw_car(array([[1],[2],[3],[4],[0.5]]))   
    
    c = array([[-2],[-3]])
    G = array([[2,-1],[-1,4]])
    draw_ellipse(c,G,0.9,ax,[0.8,0.8,1])
    P=array([[5,-3],[9,-10],[7,-4],[7,-6]])
    draw_polygon(P,ax,'green')
    
    draw_disk(array([[-8],[-8]]),2,ax,"blue")
    
    draw_arc(array([[0],[5]]),array([[4],[6]]),2,'red')
    
    show()  # only at the end. Otherwize, it closes the figure in a terminal mode



def demo_animation():    
    fig = figure(0)
    ax = fig.add_subplot(111, aspect='equal')
    for t in arange(0,5,0.1) :
        pause(0.01) #needed. Otherwize, draws only at the end 
        cla()
        ax.set_xlim(-15,15)
        ax.set_ylim(-15,15)
        draw_car(array([[t],[2],[3+t],[4],[5+t]]))    
        c = array([[-2+2*t],[-3]])
        G = array([[2+t,-1],[-1,4+t]])
        draw_ellipse(c,G,0.9,ax,[0.8,0.8,1])
#        if (t>50)&(k%2000==0):
#            fig.savefig('convoy'+str(k)+'.pdf', dpi=fig.dpi)
    show()



def sawtooth(x):
    return (x+pi)%(2*pi)-pi   # or equivalently   2*arctan(tan(x/2))



if __name__ == "__main__":
 
    np.set_printoptions(threshold=np.nan)  # print vectors in the console without "..."
    R=zeros((3,4))
    x=[[1],[2],[3]]
    R1=translate_motif(R,x)
    print('R1=',R1)

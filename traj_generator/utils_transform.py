# -*- coding: utf-8 -*-
"""
Created on Wed Oct 27 09:01:01 2021

@author: 75678
"""
import numpy as np
import math

"""
This is a function which gives rotation matrix. Notice that when use, input axis should be 'x'
Like Rot('y',45), the second input is degree
"""
def Rot(axis,th):
    pi=math.pi
    th=th*pi/180
    s=np.sin(th)
    c=np.cos(th)
    if axis == 'x':
        R=np.array([[1,0,0], [0,c,-s], [0,s,c]])
    elif axis == 'y':
        R=np.array([[c,0,s], [0,1,0], [-s,0,c]])
    elif axis == 'z':
        R=np.array([[c,-s,0], [s,c,0], [0,0,1]])
    else:
        print('axis not right')
    
    return R


"""
This is a function which gives 4x4 trans matrix. Notice that when use, input axis should be 'x'
Like Rot('y',45), the second input is degree. And the 
"""
def Trans(axis,th,trans):
    pi=math.pi
    th=th*pi/180
    s=np.sin(th)
    c=np.cos(th)
    x=trans[0]
    y=trans[1]
    z=trans[2]
    if axis == 'x':
        T=np.array([[1,0,0,x], [0,c,-s,y], [0,s,c,z], [0,0,0,1]])
    elif axis == 'y':
        T=np.array([[c,0,s,x], [0,1,0,y], [-s,0,c,z],[0,0,0,1]])
    elif axis == 'z':
        T=np.array([[c,-s,0,x], [s,c,0,y], [0,0,1,z],[0,0,0,1]])
    else:
        print('axis not right')
    
    return T


"""
This is a function which gives rotation matrix of RPY. Notice that when use, r is 
rotation of Z, p is about Y, and y is about X
"""
def RPY(r,p,y):
    pi=math.pi
   
    R=np.dot(np.dot(Rot('z',r),Rot('y',p)),Rot('x',y))
    return R

"""
This is a function which gives inverse of a trans matrix, notice that for normal 
matrix, this function is not usable.
"""
def Tinv(T):
    Ti=np.zeros([4,4])
    R=T[0:3,0:3]
    Ti[3,3]=1
    Ti[0:3,0:3]=R.T
    trans=T[0:3,3]
    Ti[0:3,3]=np.dot(-R.T,trans)
    return Ti


"""
This is a DH calculator, notice that the input is rad
"""
def DHcal(al,a,d,th,datatype='degree'):
    pi=math.pi
    if datatype=='degree':
        al=al
        th=th
    elif datatype=='rad':
        al=al*180/pi
        th=th*180/pi
    else:
        print('data type should be degree or rad')
    
    T=Trans('x',al,[0,0,0]).dot(Trans('x',0,[a,0,0])).dot(Trans('z',0,[0,0,d])).dot(Trans('z',th,[0,0,0]))
    return T
    
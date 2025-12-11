import numpy as np

from numpy import cos
from numpy import sin

def get_e1234(param,theta1234):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [theta1, theta2, theta3, theta4] = theta1234

    e1234 = [[(1/2)*((-1)*2**(1/2)*sin(theta2)*(sin(theta3)+cos(theta3)* \
  sin(theta4))+cos(theta2)*(cos(theta3)+cos(theta4)+(-1)*sin( \
  theta3)*sin(theta4))),2**(-1/2)*sin(theta2)*(sin(theta3)+(-1)* \
  cos(theta3)*sin(theta4))+(-1/2)*cos(theta2)*(cos(theta3)+(-1)* \
  cos(theta4)+sin(theta3)*sin(theta4)),cos(theta3)*cos(theta4)* \
  sin(theta2)+2**(-1/2)*cos(theta2)*(cos(theta4)*sin(theta3)+sin( \
  theta4)),(-1)*(AA1+AA2+LL+(-1)*(AA1+AA2+LL+UC1)*cos(theta3)* \
  cos(theta4))*sin(theta2)+2**(-1/2)*(AA1+AA2+LL+UC1)*cos(theta2) \
  *(cos(theta4)*sin(theta3)+sin(theta4))],[(1/2)*(cos(theta1)*(( \
  -1)*cos(theta3)+cos(theta4)+sin(theta3)*sin(theta4))+sin(theta1) \
  *(2**(1/2)*cos(theta2)*(sin(theta3)+cos(theta3)*sin(theta4))+ \
  sin(theta2)*(cos(theta3)+cos(theta4)+(-1)*sin(theta3)*sin( \
  theta4)))),(1/2)*(cos(theta1)*(cos(theta3)+cos(theta4)+sin( \
  theta3)*sin(theta4))+sin(theta1)*(2**(1/2)*cos(theta2)*((-1)* \
  sin(theta3)+cos(theta3)*sin(theta4))+(-1)*sin(theta2)*(cos( \
  theta3)+(-1)*cos(theta4)+sin(theta3)*sin(theta4)))),(-1)*cos( \
  theta2)*cos(theta3)*cos(theta4)*sin(theta1)+2**(-1/2)*(cos( \
  theta1)*((-1)*cos(theta4)*sin(theta3)+sin(theta4))+sin(theta1) \
  *sin(theta2)*(cos(theta4)*sin(theta3)+sin(theta4))),cos(theta2) \
  *(AA1+AA2+LL+(-1)*(AA1+AA2+LL+UC1)*cos(theta3)*cos(theta4))* \
  sin(theta1)+2**(-1/2)*(AA1+AA2+LL+UC1)*(cos(theta1)*((-1)*cos( \
  theta4)*sin(theta3)+sin(theta4))+sin(theta1)*sin(theta2)*(cos( \
  theta4)*sin(theta3)+sin(theta4)))],[(1/2)*(sin(theta1)*((-1)* \
  cos(theta3)+cos(theta4)+sin(theta3)*sin(theta4))+cos(theta1)*(( \
  -1)*2**(1/2)*cos(theta2)*(sin(theta3)+cos(theta3)*sin(theta4)) \
  +(-1)*sin(theta2)*(cos(theta3)+cos(theta4)+(-1)*sin(theta3)* \
  sin(theta4)))),(1/2)*(sin(theta1)*(cos(theta3)+cos(theta4)+sin( \
  theta3)*sin(theta4))+cos(theta1)*(2**(1/2)*cos(theta2)*(sin( \
  theta3)+(-1)*cos(theta3)*sin(theta4))+sin(theta2)*(cos(theta3)+ \
  (-1)*cos(theta4)+sin(theta3)*sin(theta4)))),cos(theta1)*cos( \
  theta2)*cos(theta3)*cos(theta4)+2**(-1/2)*sin(theta1)*((-1)* \
  cos(theta4)*sin(theta3)+sin(theta4))+(-1)*2**(-1/2)*cos(theta1) \
  *sin(theta2)*(cos(theta4)*sin(theta3)+sin(theta4)),(1/2)*((-2) \
  *UC1+(-1)*2**(1/2)*(AA1+AA2+LL+UC1)*sin(theta1)*(cos(theta4) \
  *sin(theta3)+(-1)*sin(theta4))+cos(theta1)*((-2)*(AA1+AA2+LL) \
  *cos(theta2)+2*(AA1+AA2+LL+UC1)*cos(theta2)*cos(theta3)*cos( \
  theta4)+(-1)*2**(1/2)*(AA1+AA2+LL+UC1)*sin(theta2)*(cos( \
  theta4)*sin(theta3)+sin(theta4))))],[0,0,0,1]]

    return np.array(e1234)


def get_e12(param,theta1234):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [theta1, theta2, theta3, theta4] = theta1234

    e12 = [ 
        [cos(theta2),0,sin(theta2),UC1*sin(theta2)],[sin(theta1)*sin( \
  theta2),cos(theta1),(-1)*cos(theta2)*sin(theta1),(-1)*UC1*cos( \
  theta2)*sin(theta1)],[(-1)*cos(theta1)*sin(theta2),sin(theta1), \
  cos(theta1)*cos(theta2),UC1*((-1)+cos(theta1)*cos(theta2))],[0,0, \
  0,1]
    ]
    
    return np.array(e12)

def get_ee_e1(qee1, theta1):
    ee1x, ee1y, ee1z = qee1
    tee1 = theta1
    ee_e1 = [[cos(tee1),0,sin(tee1),ee1x+(-1)*ee1x*cos(tee1)+(-1)*ee1z*sin( \
    tee1)],[0,1,0,0],[(-1)*sin(tee1),0,cos(tee1),ee1z+(-1)*ee1z*cos( \
    tee1)+ee1x*sin(tee1)],[0,0,0,1]]

    return np.array(ee_e1)

def get_ee_e12(qee1, qee2, theta12):
    
    tee1, tee2 = theta12
    ee1x, ee1y, ee1z = qee1
    ee2x, ee2y, ee2z = qee2
    
    ee_e12 = [[cos(tee1),sin(tee1)*sin(tee2),cos(tee2)*sin(tee1),ee1x+(-1)* \
            ee1x*cos(tee1)+(-1)*sin(tee1)*(ee1z+(-1)*ee2z+ee2z*cos(tee2)+ \
            ee2y*sin(tee2))],[0,cos(tee2),(-1)*sin(tee2),ee2y+(-1)*ee2y*cos( \
            tee2)+ee2z*sin(tee2)],[(-1)*sin(tee1),cos(tee1)*sin(tee2),cos( \
            tee1)*cos(tee2),ee1z+ee1x*sin(tee1)+(-1)*cos(tee1)*(ee1z+(-1) \
            *ee2z+ee2z*cos(tee2)+ee2y*sin(tee2))],[0,0,0,1]]
    
    return np.array(ee_e12)


def get_t43(param, theta1234):
    [JA1,JA2,UC1,UC2,AA1,AA2,AO1,AO2,LL,JD] = param
    [theta1, theta2, theta3, theta4] = theta1234
    
    t43 = [[(1/2)*(cos(theta3)+cos(theta4)+(-1)*sin(theta3)*sin(theta4)),( \
    1/2)*((-1)*cos(theta3)+cos(theta4)+sin(theta3)*sin(theta4)),( \
    -1)*2**(-1/2)*(sin(theta3)+cos(theta3)*sin(theta4)),(-1)*2**( \
    -1/2)*(AA1+AA2+LL+UC1)*(sin(theta3)+cos(theta3)*sin(theta4))],[( \
    1/2)*((-1)*cos(theta3)+cos(theta4)+(-1)*sin(theta3)*sin( \
    theta4)),(1/2)*(cos(theta3)+cos(theta4)+sin(theta3)*sin(theta4)) \
    ,2**(-1/2)*(sin(theta3)+(-1)*cos(theta3)*sin(theta4)),2**(-1/2) \
    *(AA1+AA2+LL+UC1)*(sin(theta3)+(-1)*cos(theta3)*sin(theta4))],[ \
    2**(-1/2)*(cos(theta4)*sin(theta3)+sin(theta4)),2**(-1/2)*((-1) \
    *cos(theta4)*sin(theta3)+sin(theta4)),cos(theta3)*cos(theta4),( \
    AA1+AA2+LL+UC1)*((-1)+cos(theta3)*cos(theta4))],[0,0,0,1]]

    return t43
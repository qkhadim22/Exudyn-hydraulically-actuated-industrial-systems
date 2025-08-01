#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                            #PARAMETERS
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from math import sin, cos, sqrt, pi, tanh, atan2, degrees
import sys, os, time, math, scipy.io
import math as mt

#EXUDYN Libraries
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.plot import PlotSensor, listMarkerStyles
from exudyn.signalProcessing import GetInterpolatedSignalValue
from exudyn.physics import StribeckFunction
from exudyn.FEM import *


fileName1       = 'AbaqusMesh/Pillar.stl'
fileName2       = 'AbaqusMesh/LiftBoom.stl'
fileName3       = 'AbaqusMesh/TiltBoom.stl'
fileName4       = 'AbaqusMesh/Bracket1.stl'
fileName5       = 'AbaqusMesh/Bracket2.stl'
fileName6       = 'AbaqusMesh/ExtensionBoom.stl'

# physical parameters
Gravity         = [0, -9.8066, 0]  # Gravity
# Cylinder and piston parameters
L_Cyl1          = 820e-3                            # Cylinder length
D_Cyl1          = 100e-3                             # Cylinder dia
A_1             = (pi/4)*(D_Cyl1)**2                # Area of cylinder side
L_Pis1          = 535e-3                             # Piston length, also equals to stroke length
d_pis1          = 56e-3                             # Piston dia
A_2             = A_1-(pi/4)*(d_pis1)**2            # Area on piston-rod side
L_Cyl2          = 1050e-3                            # Cylinder length
L_Pis2          = 780e-3                             # Piston length, also equals to stroke length
d_1             = 12.7e-3                         # Dia of volume 1
V1              = (pi/4)*(d_1)**2*1.5             # Volume V1 = V0
V2              = (pi/4)*(d_1)**2*1.5             # Volume V2 = V1
A               = [A_1, A_2]
Bh              = 700e6
Bc              = 2.1000e+11
Bo              = 1650e6
Fc              = 210
Fs              = 300
sig2            = 330
vs              = 5e-3
Qn               =(24/60000)/((9.9)*sqrt(35e5))                     # Nominal flow rate of valve at 18 l/min under
Qn1             = (24/60000)/((9.9)*sqrt(35e5))                      # Nominal flow rate of valve at 18 l/min under
Qn2             = (32/60000)/((9.9)*sqrt(35e5))                      # Nominal flow rate of valve at 18 l/min under



PillarP         = np.array([0, 0, 0])
L1              = 0.365    # Length in x-direction
H1              = 1.4769      # Height in y-direction
W1              = 0.25    # Width in z-direction
bodyDim1        = [L1, H1, W1]  # body dimensions
m1              = 93.26
pMid1           = np.array([-0.017403, 0.577291, 0])  # center of mass, body0,0.004000,-0.257068
Inertia1        = np.array([[16.328381,-1.276728, 0.000016],[-1.276728, 0.612003, -5.9e-5],[0.000016,  -5.9e-5  , 16.503728]])
Mark3           = [0,0,0]

# Second Body: LiftBoom
Mark4           = [-90*1e-3, 1426.1*1e-3, 0]

L2              = 3.01055           # Length in x-direction
H2              = 0.45574           # Height in y-direction
W2              = 0.263342          # Width in z-direction
pMid2           = np.array([1.229248, 0.055596, 0])
m2              = 143.66
Inertia2        = np.array([[1.055433, 1.442440,  -0.000003],[ 1.442440,  66.577004, 0],[ -0.000003,              0  ,  67.053707]])
graphicsBody2   = GraphicsDataFromSTLfile(fileName2, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
graphicsBody2   = AddEdgesAndSmoothenNormals(graphicsBody2, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)
LiftP           = np.array(Mark4)
                          
L3              = 2.580         # Length in x-direction
H3              = 0.419         # Height in y-direction
W3              = 0.220         # Width in z-direction
m3              = 141.942729+ 15.928340
pMid3           = np.array([ 0.659935,  0.251085, 0])  # center of mass
Inertia3        = np.array([[1.055433, 1.442440,  -0.000003],[1.442440,  66.577004,    0],[ -0.000003, 0,        67.053707]])
graphicsBody3   = GraphicsDataFromSTLfile(fileName3, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
graphicsBody3   = AddEdgesAndSmoothenNormals(graphicsBody3, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)

L4              = 0.557227    # Length in x-direction
H4              = 0.1425      # Height in y-direction
W4              = 0.15        # Width in z-direction
pMid4           = np.array([0.257068, 0.004000 , 0])
m4              = 11.524039
Inertia4        = np.array([[0.333066, 0.017355, 0],[0.017355, 0.081849, 0],[0,              0, 0.268644]])
graphicsBody4   = GraphicsDataFromSTLfile(fileName4, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
graphicsBody4   = AddEdgesAndSmoothenNormals(graphicsBody4, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)

L5              = 0.569009       # Length in x-direction
H5              = 0.078827       # Height in y-direction
W5              = 0.15           # Width in z-direction
pMid5           = np.array([0.212792, 0, 0])
m5              = 7.900191
Inertia5        = np.array([[0.052095, 0, 0],[0,  0.260808, 0],[0,              0,  0.216772]])
graphicsBody5   = GraphicsDataFromSTLfile(fileName5, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
graphicsBody5   = AddEdgesAndSmoothenNormals(graphicsBody5, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)

pMid6           = np.array([1.15, 0.06, 0])
m6              = 58.63
Inertia6        = np.array([[0.13, 0, 0],[0.10,  28.66, 0],[0,              0,  28.70]])
graphicsBody6   = GraphicsDataFromSTLfile(fileName6, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
graphicsBody6   = AddEdgesAndSmoothenNormals(graphicsBody6, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)



Mark5           = [170*1e-3, 386.113249*1e-3, 0]
pS              = 160e5
pT              = 1e5  
A_d             = (1/100)

a4_width_inches, a4_height_inches = 11.7, 8.3 / 2  # Horizontal layout
fontSize1       = 16
fontSize2       = 16

#fileNameT       = 'TiltBoomANSYS/TiltBoom' #for load/save of FEM data

import matplotlib.pyplot as plt

import scipy.io 
from scipy.optimize import fsolve, newton



import random

from Models.Control import *
from Models.FlexibleMultibody import *
from Models.ExudynModels import *

pMid1           = np.array([-0.017403, 0.577291, 0])  # center of mass, body0
TiltL           = LiftP + np.array([2.879420180699481, -0.040690041435711005, 0])





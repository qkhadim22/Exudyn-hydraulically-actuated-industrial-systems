#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                            #PARAMETERS
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from math import sin, cos, sqrt, pi, tanh, atan2, degrees

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
Qn               = 1.667*10*2.1597e-08                     # Nominal flow rate of valve at 18 l/min under
Qn1             = (40/60000)/((9.9)*sqrt(35e5))                      # Nominal flow rate of valve at 18 l/min under
Qn2             = (40/60000)/((9.9)*sqrt(35e5))                      # Nominal flow rate of valve at 18 l/min under



PillarP         = np.array([0, 0, 0])
L1              = 0.365    # Length in x-direction
H1              = 1.4769      # Height in y-direction
W1              = 0.25    # Width in z-direction

# Second Body: LiftBoom
L2              = 3.01055           # Length in x-direction
H2              = 0.45574           # Height in y-direction
W2              = 0.263342          # Width in z-direction

L3              = 2.580         # Length in x-direction
H3              = 0.419         # Height in y-direction
W3              = 0.220         # Width in z-direction

L4              = 0.557227    # Length in x-direction
H4              = 0.1425      # Height in y-direction
W4              = 0.15        # Width in z-direction

L5              = 0.569009       # Length in x-direction
H5              = 0.078827       # Height in y-direction
W5              = 0.15           # Width in z-direction

bodyDim1        = [L1, H1, W1]  # body dimensions
m1              = 93.26
pMid1           = np.array([-0.017403, 0.577291, 0])  # center of mass, body0,0.004000,-0.257068
Inertia1        = np.array([[16.328381,-1.276728, 0.000016],[-1.276728, 0.612003, -5.9e-5],[0.000016,  -5.9e-5  , 16.503728]])
Mark3           = [0,0,0]
Mark4           = [-90*1e-3, 1426.1*1e-3, 0]
Mark5           = [170*1e-3, 386.113249*1e-3, 0]
pS              = 160e5
pT              = 1e5  
A_d             = (1/100)

a4_width_inches, a4_height_inches = 11.7, 8.3 / 2  # Horizontal layout
fontSize1       = 16
fontSize2       = 16

#fileNameT       = 'TiltBoomANSYS/TiltBoom' #for load/save of FEM data

import math as mt
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import scipy.io 
from scipy.optimize import fsolve, newton

#EXUDYN Libraries
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.plot import PlotSensor, listMarkerStyles
from exudyn.signalProcessing import GetInterpolatedSignalValue
from exudyn.physics import StribeckFunction
from exudyn.processing import ParameterVariation
from exudyn.FEM import *


import matplotlib.gridspec as gridspec
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
from sklearn.metrics import mean_absolute_percentage_error, mean_absolute_error
from matplotlib.patches import FancyArrowPatch

import torch
from torch.utils.data import TensorDataset, DataLoader
from torch.nn.utils import clip_grad_norm_
from torch import nn
import multiprocessing as mp 



import random
import time

from Models.ComputeSLIDE import *
from Models.Control import *
from Models.ExudynFlexible import *
from scipy.stats import norm
import sys, os, time, math, scipy.io
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                            #LIBRARIES
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#Files and folders
from SLIDE.fnnModels import NNtestModel
from Models.ModelNN import NNHydraulics

from SLIDE.fnnLib import * 
import SLIDE.fnnLib

from timeit import default_timer as timer
import torch.nn.utils as utils
from enum import Enum #for data types

import sys
import numpy as np
# #from math import sin, cos, sqrt,pi
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

import torch
from torch import nn
from torch.utils.data import TensorDataset, DataLoader
# torch.set_num_threads(14)

# useCUDA = torch.cuda.is_available()
# useCUDA = False #CUDA support helps for fully connected networks > 256

# computeDevice = torch.device('cuda' if useCUDA else 'cpu')
# print('pytorch cuda=',useCUDA)


os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

#colLift     = color4blue





pMid1           = np.array([-0.017403, 0.577291, 0])  # center of mass, body0
LiftP           = np.array([-0.09, 1.4261, 0])
TiltL           = LiftP + np.array([2.879420180699481, -0.040690041435711005, 0])

# Second Body: LiftBoom
L2              = 3.01055           # Length in x-direction
H2              = 0.45574           # Height in y-direction
W2              = 0.263342          # Width in z-direction


L3              = 2.580         # Length in x-direction
H3              = 0.419         # Height in y-direction
W3              = 0.220         # Width in z-direction

L4              = 0.557227    # Length in x-direction
H4              = 0.1425      # Height in y-direction
W4              = 0.15        # Width in z-direction

L5              = 0.569009       # Length in x-direction
H5              = 0.078827       # Height in y-direction
W5              = 0.15           # Width in z-direction
        
                         # Tank pressure





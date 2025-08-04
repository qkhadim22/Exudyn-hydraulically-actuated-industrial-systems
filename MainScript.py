#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Author:   Qasim Khadim and Johannes Gerstmayr
# Contact : qasim.khadim@outlook.com,qkhadim22 (Github)
# Date:     2025-01-08
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from Models.Container import *

###############################

Patu            = True
timeStep        = 5e-3                  #Simulation time step: Change it as desired.
T               = 20                   #Time period
ns              = int(T/timeStep)       
angleInit1      = np.deg2rad(14.6)      #LiftBoom anglee              
angleInit2      = np.deg2rad(-58.8)     #TiltBoom angle 
LiftLoad        = 300              #469.1387

#in this section we create or load data
if  Patu:
    dataFile        = 'solution/TwoArms/'+str(T) + '-' + 's' + str(ns) + 'Steps' + str(LiftLoad)+'Load'
else: 
    dataFile        = 'solution/OneArm/'+str(T) + '-' + 's' + str(ns) + 'Steps' + str(LiftLoad)+'Load'
    

model       = NNHydraulics(nStepsTotal=ns, endTime=T,  mL    = LiftLoad,Flexible=True, 
                           nModes=100, loadFromSavedNPY=True, system=Patu, verboseMode=1)

inputVec    = model.CreateInputVector( ns,  angleInit1,angleInit2,system=Patu )


data = model.ComputeModel(inputVec,system=Patu,  solutionViewer = True) #solutionViewer: for visualization

data_array = np.array(data, dtype=object)
np.save(dataFile, data_array)

Plotting    =  False

if Plotting:
   model.Plotting(data_array)

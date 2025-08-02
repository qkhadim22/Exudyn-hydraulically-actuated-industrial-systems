
from Models.Container import *
from Models.ExudynModels import *

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class NNHydraulics():

    #initialize class 
    def __init__(self, nStepsTotal=100, endTime=0.5,Flexible=False, nModes = 2,loadFromSavedNPY=True, 
                 mL= 50,  visualization = False,system = True, verboseMode = 0):


        self.nStepsTotal        = nStepsTotal
        self.endTime            = endTime
        self.TimeStep           = self.endTime / (self.nStepsTotal)
        self.Flexible           = Flexible
        
        self.nModes             = nModes
        self.loadFromSavedNPY   = loadFromSavedNPY
        self.mL                 = mL
        
        self.StaticCase         = False
        self.Hydraulics         = True
        self.Visualization      = visualization
        self.Patu               = system

        
        self.angleMinDeg1      = 0
        self.angleMaxDeg1      = 50
        self.p1Init            = 8e6
        self.p2Init            = 8e6
        self.p3Init            = 8e6
        self.p4Init            = 8e6
        
        self.timeVecOut       = np.arange(1,self.nStepsTotal+1)/self.nStepsTotal*self.endTime
        

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def CreateModel(self):
        self.SC  = exu.SystemContainer()
        self.mbs = self.SC.AddSystem()
        
    #get time vector according to output data
    def GetOutputXAxisVector(self):
        return self.timeVecOut
    
    def CreateInputVector(self, relCnt = 0, theta1 = 0,theta2=0, system=False, isTest=False):
    
        if system:
            vec = np.zeros(6*self.nStepsTotal)
            U1  = np.zeros(self.nStepsTotal)
            U2  = np.zeros(self.nStepsTotal)
            pP  = np.zeros(self.nStepsTotal)
    
            for i in range(self.nStepsTotal):
                U1[i] =uref_1(self.timeVecOut[i])
                U2[i] =uref_2(self.timeVecOut[i])
                pP[i] = 100e5
    
        

            vec[0:self.nStepsTotal]                         = U1[0:self.nStepsTotal]      
            vec[1*(self.nStepsTotal):2*(self.nStepsTotal)]  = U2[0:self.nStepsTotal]
            vec[2*(self.nStepsTotal):3*(self.nStepsTotal)]  = pP[0:self.nStepsTotal]
            vec[4*(self.nStepsTotal)]  = theta1
            vec[5*(self.nStepsTotal)]  = theta2
        
        else:
            vec = np.zeros(3*self.nStepsTotal)
            U1  = np.zeros(self.nStepsTotal)
            pP  = np.zeros(self.nStepsTotal)
    
            for i in range(self.nStepsTotal):
                U1[i] =uref(self.timeVecOut[i])
                pP[i] = 100e5
    
            vec[0:self.nStepsTotal]                         = U1[0:self.nStepsTotal]      
            vec[1*(self.nStepsTotal):2*(self.nStepsTotal)]  = pP[0:self.nStepsTotal]
            vec[2*(self.nStepsTotal)]                       = theta1
    
        return vec

    

    #get number of simulation steps
    def GetNSimulationSteps(self):
        return self.nStepsTotal # x finer simulation than output

    #split input data into initial values, forces or other inputs
    def SplitInputData(self, inputData, system=False):
        rv = {}
        
        if system: 
            rv['U1']         = inputData[0*(self.nStepsTotal):1*(self.nStepsTotal)]      
            rv['U2']         = inputData[1*(self.nStepsTotal):2*(self.nStepsTotal)]   
            rv['s1']         = inputData[2*(self.nStepsTotal):3*(self.nStepsTotal)]    
            rv['s2']         = inputData[3*(self.nStepsTotal):4*(self.nStepsTotal)]       
            rv['ds1']        = inputData[4*(self.nStepsTotal):5*(self.nStepsTotal)]    
            rv['ds2']        = inputData[5*(self.nStepsTotal):6*(self.nStepsTotal)]
            rv['p1']         = inputData[6*(self.nStepsTotal):7*(self.nStepsTotal)]    
            rv['p2']         = inputData[7*(self.nStepsTotal):8*(self.nStepsTotal)]
            rv['p3']         = inputData[8*(self.nStepsTotal):9*(self.nStepsTotal)]       
            rv['p4']         = inputData[9*(self.nStepsTotal):10*(self.nStepsTotal)]     
            
        else:
            rv['U']          = inputData[0:self.nStepsTotal] 
            rv['s']          = inputData[1*(self.nStepsTotal):2*(self.nStepsTotal)]
            rv['ds']         = inputData[2*(self.nStepsTotal):3*(self.nStepsTotal)]  
            rv['p1']         = inputData[3*(self.nStepsTotal):4*(self.nStepsTotal)]
            rv['p2']         = inputData[4*(self.nStepsTotal):5*(self.nStepsTotal)]
        return rv
    
    #initialState contains position and velocity states as list of two np.arrays 
    def ComputeModel(self, inputData, system=None,solutionViewer = False, verboseMode = 0):
        self.CreateModel()
        # print('compute model')
        self.verboseMode = verboseMode
        
        #++++++++++++++++++++++++++++++++++++++++++
        if system:
            
            outputData=np.zeros(self.nStepsTotal*12)
            inputDict = self.SplitInputData(np.array(inputData), system)
            self.inputTimeU2        = np.zeros((self.nStepsTotal,2))
            self.inputTimeU1        = np.zeros((self.nStepsTotal,2))
            self.inputTimeU1[:,0]   = self.timeVecOut
            self.inputTimeU1[:,1]   = inputDict['U1']  
            self.inputTimeU2[:,0]   = self.timeVecOut
            self.inputTimeU2[:,1]   = inputDict['U2']        
            
            self.mbs.variables['inputTimeU1'] = self.inputTimeU1            
            self.mbs.variables['inputTimeU2'] = self.inputTimeU2            
            self.mbs.variables['theta1'] = inputData[self.nStepsTotal*3]
            self.mbs.variables['theta2'] = inputData[self.nStepsTotal*4]
            
            PatuCrane(self, self.mbs.variables['theta1'],self.mbs.variables['theta2'], 
                          self.p1Init, self.p2Init, self.p3Init, self.p4Init)
            
            # Data arrangement
            DS = self.dictSensors
            
            inputData[0:self.nStepsTotal]                           =  inputData[0:self.nStepsTotal]
            inputData[1*self.nStepsTotal:2*self.nStepsTotal]        =  inputData[1*self.nStepsTotal:2*self.nStepsTotal]
            
            outputData[0:self.nStepsTotal]                          =  self.mbs.GetSensorStoredData(DS['sDistance1'])[0:self.nStepsTotal,1]
            outputData[1*self.nStepsTotal:2*self.nStepsTotal]       =  self.mbs.GetSensorStoredData(DS['sDistance2'])[0:self.nStepsTotal,1]
            outputData[2*self.nStepsTotal:3*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sVelocity1'])[0:self.nStepsTotal,1]
            outputData[3*self.nStepsTotal:4*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sVelocity2'])[0:self.nStepsTotal,1]
            outputData[4*self.nStepsTotal:5*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sPressures1'])[0:self.nStepsTotal,1]
            outputData[5*self.nStepsTotal:6*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sPressures1'])[0:self.nStepsTotal,2]
            outputData[6*self.nStepsTotal:7*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sPressures2'])[0:self.nStepsTotal,1]
            outputData[7*self.nStepsTotal:8*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sPressures2'])[0:self.nStepsTotal,2]

            if self.Flexible:
                outputData[8*self.nStepsTotal:9*self.nStepsTotal]   = self.mbs.GetSensorStoredData(DS['StrainF1'])[0:1*self.nStepsTotal,1]
                outputData[9*self.nStepsTotal:10*self.nStepsTotal]  = self.mbs.GetSensorStoredData(DS['StrainF2'])[0:1*self.nStepsTotal,1]
                outputData[10*self.nStepsTotal:11*self.nStepsTotal]   = self.mbs.GetSensorStoredData(DS['Stress1'])[0:1*self.nStepsTotal,1]
                outputData[11*self.nStepsTotal:12*self.nStepsTotal]  = self.mbs.GetSensorStoredData(DS['Stress2'])[0:1*self.nStepsTotal,1]

            
        else:
            inputDict = self.SplitInputData(np.array(inputData), system)
            
            outputData=np.zeros(self.nStepsTotal*6)
            
            self.inputTimeU1 = np.zeros((self.nStepsTotal,2))
            self.inputTimeU1[:,0] = self.timeVecOut
            self.inputTimeU1[:,1] = inputDict['U']        
            
            self.mbs.variables['inputTimeU1'] = self.inputTimeU1            
            self.mbs.variables['theta1'] = inputData[self.nStepsTotal*2]
            
            LiftBoom(self, self.mbs.variables['theta1'], self.p1Init, self.p2Init)

            #++++++++++++++++++++++++++
            DS = self.dictSensors

            # input
            inputData[0:self.nStepsTotal]                           = inputData[0:self.nStepsTotal]
            
            # output
            
            outputData[0:self.nStepsTotal]                          = self.mbs.GetSensorStoredData(DS['sDistance'])[0:self.nStepsTotal,1]
            outputData[1*self.nStepsTotal:2*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sVelocity'])[0:self.nStepsTotal,1]
            outputData[2*self.nStepsTotal:3*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sPressures'])[0:self.nStepsTotal,1]
            outputData[3*self.nStepsTotal:4*self.nStepsTotal]       = self.mbs.GetSensorStoredData(DS['sPressures'])[0:self.nStepsTotal,2]
        
            if self.Flexible:
                outputData[4*self.nStepsTotal:5*self.nStepsTotal]   = self.mbs.GetSensorStoredData(DS['StrainPoint'])[0:1*self.nStepsTotal,6]
                outputData[5*self.nStepsTotal:6*self.nStepsTotal]   = self.mbs.GetSensorStoredData(DS['StressPoint'])[0:1*self.nStepsTotal,1]
        
        if solutionViewer:
           self.mbs.SolutionViewer()
           

        return [inputData, outputData]
    
    
    def Plotting(self, data): 
        Time    = self.timeVecOut
        end     = Time.shape[0]
        
        if self.Patu: 
            
            s1      = data[1][0:end]
            s2      = data[1][1*end:2*end]
            dots1   = data[1][2*end:3*end]
            dots2   = data[1][3*end:4*end]
            p1      = data[1][4*end:5*end]
            p2      = data[1][5*end:6*end]
            p3      = data[1][6*end:7*end]
            p4      = data[1][7*end:8*end]
            
            # Signal 1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, self.inputTimeU1[:,1], #label='Simulation', 
                     linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
            
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('Control signal, V')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(-12, 12)
            plt.savefig('solution/TwoArms/inputU1.png')
            plt.show()
            
            # Signal 2
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, self.inputTimeU2[:,1], #label='Simulation', 
                     linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
            
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('Control signal, V')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(-12, 12)
            plt.savefig('solution/TwoArms/inputU2.png')
            plt.show()
            
            
            #s1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, 1000*s1, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('s, mm')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(800, 1200)
            plt.savefig('solution/TwoArms/s1.png')
            plt.show()
            
            
            #s2
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, 1000*s2, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('s, mm')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(950, 1400)
            plt.savefig('solution/TwoArms/s2.png')
            plt.show()
            
            
            #dots1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, dots1, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('v, m/s')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(-0.125, 0.125)
            plt.savefig('solution/TwoArms/dots1.png')
            plt.show()
            
            
            #dots1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, dots2, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('v, m/s')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(-0.15, 0.15)
            plt.savefig('solution/TwoArms/dots2.png')
            plt.show()
            
            #p1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, p1, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('p1, Pa')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(1e5, 250e5)
            plt.savefig('solution/TwoArms/p1.png')
            plt.show()
            
            
            #p1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, p2, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('p2, Pa')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(1e5, 150e5)
            plt.savefig('solution/TwoArms/p2.png')
            plt.show()
            
            
            #p1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, p3, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('p3, Pa')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(-25e5, 150e5)
            plt.savefig('solution/TwoArms/p3.png')
            plt.show()
            
            
            #p1
            plt.figure(figsize=(10, 5))
            # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
            #  linewidth=1, markersize=2, color='green')
            plt.plot(Time, p4, #label='Simulation', 
                    linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           
            plt.xlabel('Time, s')  # Adjust label as appropriate
            plt.ylabel('p4, Pa')  # Adjust label as appropriate
            plt.grid(True)  # Add grid
            # Set axis limits
            plt.xlim(0, self.endTime)
            plt.ylim(0e5, 100e5)
            plt.savefig('solution/TwoArms/p4.png')
            plt.show()
            
            
            if self.Flexible:
                deltaEps1      = data[1][8*end:9*end]
                deltaEps2      = data[1][9*end:10*end]
                deltaSig1      = data[1][10*end:11*end]
                deltaSig2      = data[1][11*end:12*end]
                
                #Strain
                plt.figure(figsize=(10, 5))
                # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
                #  linewidth=1, markersize=2, color='green')
                plt.plot(Time, 1e6*deltaEps1, #label='Simulation', 
                        linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
               
                plt.xlabel('Time, s')  # Adjust label as appropriate
                plt.ylabel(r'$\epsilon_1$, $\mu$m')  # Adjust label as appropriate
                plt.grid(True)  # Add grid
                # Set axis limits
                plt.xlim(0, self.endTime)
                plt.ylim(-250, 450)
                plt.savefig('solution/TwoArms/deltaEps1.png')
                plt.show()
                
                #Strain
                plt.figure(figsize=(10, 5))
                # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
                #  linewidth=1, markersize=2, color='green')
                plt.plot(Time, 1e6*deltaEps2, #label='Simulation', 
                        linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
               
                plt.xlabel('Time, s')  # Adjust label as appropriate
                plt.ylabel(r'$\epsilon_2$, $\mu$m')  # Adjust label as appropriate
                plt.grid(True)  # Add grid
                # Set axis limits
                plt.xlim(0, self.endTime)
                plt.ylim(-30, 30)
                plt.savefig('solution/TwoArms/deltaEps2.png')
                plt.show()
                
                
                #Stress
                plt.figure(figsize=(10, 5))
                # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
                #  linewidth=1, markersize=2, color='green')
                plt.plot(Time, deltaSig1/1e6, #label='Simulation', 
                        linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
               
                plt.xlabel('Time, s')  # Adjust label as appropriate
                plt.ylabel(r'$\sigma_1$, MPa')  # Adjust label as appropriate
                plt.grid(True)  # Add grid
                # Set axis limits
                plt.xlim(0, self.endTime)
                plt.ylim(-60, 100)
                plt.savefig('solution/TwoArms/deltaSig1.png')
                plt.show()
                
                #Stress
                plt.figure(figsize=(10, 5))
                # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
                #  linewidth=1, markersize=2, color='green')
                plt.plot(Time, deltaSig2/1e6, #label='Simulation', 
                        linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
               
                plt.xlabel('Time, s')  # Adjust label as appropriate
                plt.ylabel(r'$\sigma_2$, MPa')  # Adjust label as appropriate
                plt.grid(True)  # Add grid
                # Set axis limits
                plt.xlim(0, self.endTime)
                plt.ylim(-5, 7)
                plt.savefig('solution/TwoArms/deltaSig2.png')
                plt.show()
                
            
            
            
            
        else:
          
           s1      = data[1][0:end]
           dots1   = data[1][1*end:2*end]
           p1      = data[1][2*end:3*end]
           p2      = data[1][3*end:4*end]
          
           # Signal 1
           plt.figure(figsize=(10, 5))
          # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
          #  linewidth=1, markersize=2, color='green')
           plt.plot(Time, self.inputTimeU1[:,1], #label='Simulation', 
                   linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
          
           plt.xlabel('Time, s')  # Adjust label as appropriate
           plt.ylabel('Control signal, V')  # Adjust label as appropriate
           plt.grid(True)  # Add grid
           # Set axis limits
           plt.xlim(0, self.endTime)
           plt.ylim(-12, 12)
           plt.savefig('solution/OneArm/inputU1.png')
           plt.show()
           
           
           #s1
           plt.figure(figsize=(10, 5))
           # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
           #  linewidth=1, markersize=2, color='green')
           plt.plot(Time, 1000*s1, #label='Simulation', 
                   linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
           plt.xlabel('Time, s')  # Adjust label as appropriate
           plt.ylabel('s, mm')  # Adjust label as appropriate
           plt.grid(True)  # Add grid
           # Set axis limits
           plt.xlim(0, self.endTime)
           plt.ylim(L_Cyl1*1000, 1200)
           plt.savefig('solution/OneArm/s1.png')
           plt.show()
           
           #dots1
           plt.figure(figsize=(10, 5))
           # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
           #  linewidth=1, markersize=2, color='green')
           plt.plot(Time, dots1, #label='Simulation', 
                   linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
          
           plt.xlabel('Time, s')  # Adjust label as appropriate
           plt.ylabel('v, m/s')  # Adjust label as appropriate
           plt.grid(True)  # Add grid
           # Set axis limits
           plt.xlim(0, self.endTime)
           plt.ylim(-0.12, 0.12)
           plt.savefig('solution/OneArm/dots1.png')
           plt.show()
           
           #p1
           plt.figure(figsize=(10, 5))
           # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
           #  linewidth=1, markersize=2, color='green')
           plt.plot(Time, p1, #label='Simulation', 
                   linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
          
           plt.xlabel('Time, s')  # Adjust label as appropriate
           plt.ylabel('p1, Pa')  # Adjust label as appropriate
           plt.grid(True)  # Add grid
           # Set axis limits
           plt.xlim(0, self.endTime)
           plt.ylim(0e5, 100e5)
           plt.savefig('solution/OneArm/p1.png')
           plt.show()
           
           
           #p1
           plt.figure(figsize=(10, 5))
           # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
           #  linewidth=1, markersize=2, color='green')
           plt.plot(Time, p2, #label='Simulation', 
                   linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
          
           plt.xlabel('Time, s')  # Adjust label as appropriate
           plt.ylabel('p2, Pa')  # Adjust label as appropriate
           plt.grid(True)  # Add grid
           # Set axis limits
           plt.xlim(0, self.endTime)
           plt.ylim(0e5, 100e5)
           plt.savefig('solution/OneArm/p2.png')
           plt.show()
           
           
           if self.Flexible:
               deltaEps      = data[1][4*end:5*end]
               deltaSig      = data[1][5*end:6*end]
               
               #Strain
               plt.figure(figsize=(10, 5))
               # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
               #  linewidth=1, markersize=2, color='green')
               plt.plot(Time, 1e6*deltaEps, #label='Simulation', 
                       linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
              
               plt.xlabel('Time, s')  # Adjust label as appropriate
               plt.ylabel(r'$\epsilon$, $\mu$m')  # Adjust label as appropriate
               plt.grid(True)  # Add grid
               # Set axis limits
               plt.xlim(0, self.endTime)
               plt.ylim(-30, 20)
               plt.savefig('solution/OneArm/deltaEps.png')
               plt.show()
               
               
               #Stress
               plt.figure(figsize=(10, 5))
               # plt.plot(Time, ExpData[0:end,16], label='Experimental', marker='x', 
               #  linewidth=1, markersize=2, color='green')
               plt.plot(Time, deltaSig/1e6, #label='Simulation', 
                       linestyle='-', marker='x', linewidth=1, markersize=2, color='black')
              
               plt.xlabel('Time, s')  # Adjust label as appropriate
               plt.ylabel(r'$\sigma$, MPa')  # Adjust label as appropriate
               plt.grid(True)  # Add grid
               # Set axis limits
               plt.xlim(0, self.endTime)
               plt.ylim(-15, 20)
               plt.savefig('solution/OneArm/deltaSig.png')
               plt.show()
            
            
       
      
        
        # data_dict = data[1]
        
        # for key, value in data_dict.items(): 
        #         title = f' {key}'  # Customize your title
        #         plotData = np.column_stack([Time, value])
        #         self.mbs.PlotSensor(plotData, title=title, newFigure = True, closeAll = False)

        
        return    
    
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if __name__ == '__main__': #include this to enable parallel processing

    model = NNHydraulics(nStepsTotal=250, endTime=1, 
                         verboseMode=1)
    
    inputData = model.CreateInputVector(0)
    [inputData, output] = model.ComputeModel(inputData, verboseMode=True, 
                                             solutionViewer=False)
    
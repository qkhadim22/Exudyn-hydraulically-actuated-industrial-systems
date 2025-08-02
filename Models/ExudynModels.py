
from Models.Container import *

feL             = FEMinterface()
feT             = FEMinterface()


def LiftBoom(self, theta1, p1, p2):
        self.theta1                 = theta1
        self.p1                     = p1
        self.p2                     = p2                         
        self.dictSensors            = {}
        
        self.StaticInitialization   = True
        Emodulus                    = 2.1e11
        nu                          = 0.3
        rho                         = 7850
        mat                         = KirchhoffMaterial(Emodulus, nu, rho)
        varType2                    = exu.OutputVariableType.StrainLocal
    
        #Ground body
        oGround                     = self.mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                                                                          visualization=VObjectGround(graphicsData=[plane])))
        
        markerGround                = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
        iCube1                      = RigidBodyInertia(mass=m1, com=pMid1,inertiaTensor=Inertia1,inertiaTensorAtCOM=True)
        graphicsCOM1                = GraphicsDataBasis(origin=iCube1.com, length=2*W1)
        
        # Definintion of pillar as body in Exudyn and node n1
        [n1, b1]                    = AddRigidBody(mainSys=self.mbs,inertia=iCube1,nodeType=exu.NodeType.RotationEulerParameters,
                                                   position=PillarP,rotationMatrix=np.diag([1, 1, 1]),gravity=[0, -9.8066, 0] ,graphicsDataList=[graphicsCOM1, graphicsBody1]
                                                   )
        Marker3                     = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=Mark3))                     #With Ground
        Marker4                     = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=Mark4))            #Lift Boom
        Marker5                     = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=Mark5))        # Cylinder 1 position
        
        # Fixed joint between Pillar and Ground
        self.mbs.AddObject(GenericJoint(markerNumbers=[markerGround, Marker3],constrainedAxes=[1, 1, 1, 1, 1, 1],
                                        visualization=VObjectJointGeneric(axesRadius=0.2*W1,axesLength=1.4*W1)))
        
        
        if self.Flexible:
            filePath               = 'AbaqusMesh/Job-1'
        
            if not self.loadFromSavedNPY: 
                start_time              = time.time()
                nodes1                  = feL.ImportFromAbaqusInputFile(filePath+'.inp', typeName='Part', name='Job-1')
                feL.ReadMassMatrixFromAbaqus(fileName=filePath + '_MASS2.mtx')             #Load mass matrix
                feL.ReadStiffnessMatrixFromAbaqus(fileName=filePath + '_STIF2.mtx')        #Load stiffness matrix
                feL.SaveToFile(filePath,mode='PKL')
            
                if self.verboseMode:
                    print("--- saving LiftBoom FEM Abaqus data took: %s seconds ---" % (time.time() - start_time)) 
                    
                 
            else:       
                    if self.verboseMode:
                        print('importing Abaqus FEM data structure of Lift Boom...')
                    
                    start_time          = time.time()
                    feL.LoadFromFile(filePath,mode='PKL')
                    cpuTime             = time.time() - start_time
                    
                    if self.verboseMode:
                        print("--- importing FEM data took: %s seconds ---" % (cpuTime))
                    
        
                    
            p2                  = [0, 0,-100*1e-3]
            p1                  = [0, 0, 100*1e-3]
            radius1             = 25*1e-3
            nodeListJoint1      = feL.GetNodesOnCylinder(p1, p2, radius1, tolerance=1e-4) 
            pJoint1             = feL.GetNodePositionsMean(nodeListJoint1)
            nodeListJoint1Len   = len(nodeListJoint1)
            noodeWeightsJoint1  = [1/nodeListJoint1Len]*nodeListJoint1Len
            noodeWeightsJoint1  = feL.GetNodeWeightsFromSurfaceAreas(nodeListJoint1)
            
            p4                  = [304.19*1e-3,-100.01*1e-3,-100*1e-3]
            p3                  = [304.19*1e-3,-100.01*1e-3, 100*1e-3]
            radius2             = 36*1e-3
            nodeListPist1       = feL.GetNodesOnCylinder(p3, p4, radius2, tolerance=1e-2)  
            pJoint2             = feL.GetNodePositionsMean(nodeListPist1)
            nodeListPist1Len    = len(nodeListPist1)
            noodeWeightsPist1   = [1/nodeListPist1Len]*nodeListPist1Len
            
            if self.mL != 0:
                p10             = [2875*1e-3,15.15*1e-3,    74*1e-3]
                p9              = [2875*1e-3,15.15*1e-3,   -74*1e-3]
                radius5         = 46*1e-3
                nodeListJoint3  = feL.GetNodesOnCylinder(p9, p10, radius5, tolerance=1e-4)  
                pJoint5         = feL.GetNodePositionsMean(nodeListJoint3)
                nodeListJoint3Len= len(nodeListJoint3)
                noodeWeightsJoint3  = [1/nodeListJoint3Len]*nodeListJoint3Len
            
            
                # STEP 2: Craig-Bampton Modes
                boundaryList        = [nodeListJoint1, nodeListPist1, nodeListJoint3]
            
            else: 
                # STEP 2: Craig-Bampton Modes
                boundaryList        = [nodeListJoint1, nodeListPist1]
    
        
            start_time          = time.time()

            if self.loadFromSavedNPY:
                    if self.mL != 0:
                        feL.LoadFromFile('AbaqusMesh/feL_Load',mode='PKL')
                    else:
                        feL.LoadFromFile('AbaqusMesh/feL',mode='PKL')
            else:
                feL.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, nEigenModes=self.nModes, 
                                                    useSparseSolver=True,computationMode = HCBstaticModeSelection.RBE2) 
          
                print("ComputePostProcessingModes ... (may take a while)")
                feL.ComputePostProcessingModes(material=mat,outputVariableType=varType2,)
         
                if self.mL != 0:  
                    feL.SaveToFile('AbaqusMesh/feL_Load', mode='PKL')
                else:
                    feL.SaveToFile('AbaqusMesh/feL', mode='PKL')
              
            if self.verboseMode:
                print("Hurty-Craig Bampton modes... ")
                print("eigen freq.=", feL.GetEigenFrequenciesHz())
                print("HCB modes needed %.3f seconds" % (time.time() - start_time))  


            colLift = color4blue
            LiftBoom            = ObjectFFRFreducedOrderInterface(feL)

            LiftBoomFFRF        = LiftBoom.AddObjectFFRFreducedOrder(self.mbs, positionRef=np.array(Mark4), 
                                          initialVelocity=[0,0,0], 
                                          initialAngularVelocity=[0,0,0],
                                          rotationMatrixRef  = RotationMatrixZ(mt.radians(self.theta1)),
                                          gravity= [0, -9.8066, 0],
                                          #massProportionalDamping = 0, stiffnessProportionalDamping = 1e-5 ,
                                         massProportionalDamping = 0, stiffnessProportionalDamping = 3.35e-3 ,
                                         color=colLift,)
        
       
            self.mbs.SetObjectParameter(objectNumber=LiftBoomFFRF['oFFRFreducedOrder'],parameterName='outputVariableTypeModeBasis',value=varType2)
    
            Marker7             = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'], meshNodeNumbers=np.array(nodeListJoint1), #these are the meshNodeNumbers
                                          weightingFactors=noodeWeightsJoint1))
            Marker8             = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'], meshNodeNumbers=np.array(nodeListPist1), #these are the meshNodeNumbers
                                          weightingFactors=noodeWeightsPist1))
        
        
        else:
            iCube2          = RigidBodyInertia(mass=m2, com=pMid2,inertiaTensor=Inertia2,inertiaTensorAtCOM=True)
            graphicsCOM2    = GraphicsDataBasis(origin=iCube2.com, length=2*W2)

            [n2, b2]        = AddRigidBody(mainSys=self.mbs,inertia=iCube2,nodeType=exu.NodeType.RotationEulerParameters,position=LiftP,  
                                           rotationMatrix= RotationMatrixZ(mt.radians(self.theta1)), gravity= [0, -9.8066, 0],
                                           graphicsDataList=[graphicsCOM2, graphicsBody2])
    
            Marker7         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0, 0, 0]))                       #With Pillar     
            Marker8         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0.3025, -0.105, 0]))             #With Cylinder 1
    
            
        #Revolute Joint
        self.mbs.AddObject(GenericJoint(markerNumbers=[Marker4, Marker7],constrainedAxes=[1,1,1,1,1,0],
                                        visualization=VObjectJointGeneric(axesRadius=0.18*0.263342,axesLength=1.1*0.263342)))
    
       
        # Add load 
        if self.mL != 0:
            if self.Flexible:
                Marker9             = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'],
                                              meshNodeNumbers=np.array(nodeListJoint3), #these are the meshNodeNumbers
                                              weightingFactors=noodeWeightsJoint3))
            
            else:
                Marker9             = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[2875*1e-3,15.15*1e-3])) 
                
            
            pos = self.mbs.GetMarkerOutput(Marker9, variableType=exu.OutputVariableType.Position, 
                                           configuration=exu.ConfigurationType.Reference)
            #print('pos=', pos)
            bMass = self.mbs.CreateMassPoint(physicsMass=self.mL, referencePosition=pos, show=True, gravity=[0, -9.8066, 0],
                                     graphicsDataList=[GraphicsDataSphere(radius=0.04, color=color4red)])
            mMass = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=bMass))
            self.mbs.AddObject(SphericalJoint(markerNumbers=[Marker9, mMass], visualization=VSphericalJoint(show=False)))

            #self.mbs.AddLoad(LoadForceVector(markerNumber=Marker9, bodyFixed=True, loadVector=[0,-self.mL,0]))
        
        colCyl              = color4orange
        colPis              = color4grey 
    
    
        #ODE1 for pressures:
        nODE1            = self.mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0,0],
                                initialCoordinates=[self.p1,
                                                    self.p2], #initialize with 20 bar
                                numberOfODE1Coordinates=2))
    
        # #Not used
        def CylinderFriction1(mbs, t, itemNumber, u, v, k, d, F0):

                Ff = 0.2*StribeckFunction(v, muDynamic=1, muStaticOffset=1.5, regVel=1e-4)+(k*(u) + d*v + k*(u)**3-F0)
                #print(Ff)
                return Ff
            
        def UFfrictionSpringDamper(mbs, t, itemIndex, u, v, k, d, f0):
            return   1*(Fc*tanh(4*(abs(v    )/vs))+(Fs-Fc)*((abs(v    )/vs)/((1/4)*(abs(v    )/vs)**2+3/4)**2))*np.sign(v )+sig2*v    *tanh(4)
          
            

        oFriction1       = self.mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[Marker5, Marker8], referenceLength=0,stiffness=0,
                                                            damping=0, force=0, velocityOffset = 0., activeConnector = True,
                                                            springForceUserFunction=CylinderFriction1,
                                                              visualization=VSpringDamper(show=False) ))
        
       
        oHA1 = None
        if True:
            oHA1                = self.mbs.AddObject(HydraulicActuatorSimple(name='LiftCylinder', markerNumbers=[ Marker5, Marker8], 
                                                    nodeNumbers=[nODE1], offsetLength=L_Cyl1, strokeLength=L_Pis1, chamberCrossSection0=A[0], 
                                                    chamberCrossSection1=A[1], hoseVolume0=V1, hoseVolume1=V2, valveOpening0=0, 
                                                    valveOpening1=0, actuatorDamping=2e4, oilBulkModulus=Bo, cylinderBulkModulus=Bc, 
                                                    hoseBulkModulus=Bh, nominalFlow=Qn, systemPressure=pS, tankPressure=pT, 
                                                    useChamberVolumeChange=True, activeConnector=True, 
                                                    visualization={'show': True, 'cylinderRadius': 50e-3, 'rodRadius': 28e-3, 
                                                                    'pistonRadius': 0.04, 'pistonLength': 0.001, 'rodMountRadius': 0.0, 
                                                                    'baseMountRadius': 20.0e-3, 'baseMountLength': 20.0e-3, 'colorCylinder': color4blue,
                                                                'colorPiston': color4grey}
                                                    ))
            self.oHA1 = oHA1

        if self.StaticCase or self.StaticInitialization:
            #compute reference length of distance constraint 
            self.mbs.Assemble()
            mGHposition = self.mbs.GetMarkerOutput(Marker5, variableType=exu.OutputVariableType.Position, 
                                             configuration=exu.ConfigurationType.Initial)
            mRHposition = self.mbs.GetMarkerOutput(Marker8, variableType=exu.OutputVariableType.Position, 
                                             configuration=exu.ConfigurationType.Initial)
            
            dLH0 = NormL2(mGHposition - mRHposition)
            if self.verboseMode:
                print('dLH0=', dLH0)
            
            #use distance constraint to compute static equlibrium in static case
            oDC = self.mbs.AddObject(DistanceConstraint(markerNumbers=[Marker5, Marker8], distance=dLH0))

        self.mbs.variables['isStatics'] = False
        from exudyn.signalProcessing import GetInterpolatedSignalValue

        #function which updates hydraulics values input        
        def PreStepUserFunction(mbs, t):
            if not mbs.variables['isStatics']: #during statics, valves must be closed
                Av0 = GetInterpolatedSignalValue(t, mbs.variables['inputTimeU1'], timeArray= [], dataArrayIndex= 1, 
                                            timeArrayIndex= 0, rangeWarning= False)
                # Av0 = 10
                distance = mbs.GetObjectOutput(self.oHA1, exu.OutputVariableType.Distance)

                if distance < 0.75 or distance > 1.2: #limit stroke of actuator
                # if distance < 0.9 or distance > 1:
                    Av0 = 0

                # Av0 = U[mt.trunc(t/h)]
                Av1 = -Av0
            
                if oHA1 != None:
                    mbs.SetObjectParameter(oHA1, "valveOpening0", Av0)
                    mbs.SetObjectParameter(oHA1, "valveOpening1", Av1)

            return True
    
        self.mbs.SetPreStepUserFunction(PreStepUserFunction) 
        if self.verboseMode:
            print('#joint nodes=',len(nodeListJoint3))

        if self.Flexible:
            StressNode  = feL.GetNodeAtPoint(np.array([0.639392078,  0.110807151, 0.0799999982]))
        
            if self.verboseMode:
                print("nMid=",nMid)
                print("nMid=",MarkerTip)
        
            StrainF2    = self.mbs.AddSensor(SensorSuperElement(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'], meshNodeNumber=StressNode, 
                                                               storeInternal=True, outputVariableType=varType2 ))
        
        
        
            def UFStressData(mbs, t, sensorNumbers, factors, configuration):
            
                val = mbs.GetSensorValues(sensorNumbers[0])
                StressVec = mat.StrainVector2StressVector(val)
                return StressVec
        
        
            self.dictSensors['StrainPoint']       = StrainF2
            self.dictSensors['StressPoint']       = self.mbs.AddSensor(SensorUserFunction(sensorNumbers=[StrainF2], 
                                                        storeInternal=True, sensorUserFunction=UFStressData))
           
        if oHA1 != None:
            sForce          = self.mbs.AddSensor(SensorObject(objectNumber=oHA1, storeInternal=True, outputVariableType=exu.OutputVariableType.Force))
            self.dictSensors['sForce']=sForce
            sDistance       = self.mbs.AddSensor(SensorObject(objectNumber=oHA1, storeInternal=True, outputVariableType=exu.OutputVariableType.Distance))
            self.dictSensors['sDistance']=sDistance
    
            sVelocity       = self.mbs.AddSensor(SensorObject(objectNumber=oHA1, storeInternal=True, outputVariableType=exu.OutputVariableType.VelocityLocal))
            self.dictSensors['sVelocity']=sVelocity
            sPressures      = self.mbs.AddSensor(SensorNode(nodeNumber=nODE1, storeInternal=True,outputVariableType=exu.OutputVariableType.Coordinates))   
            self.dictSensors['sPressures']=sPressures
            
        #+++++++++++++++++++++++++++++++++++++++++++++++++++
        #assemble and solve    
        self.mbs.Assemble()

        self.simulationSettings = exu.SimulationSettings()   
        self.simulationSettings.solutionSettings.sensorsWritePeriod = self.endTime / (self.nStepsTotal)
        
        self.simulationSettings.timeIntegration.numberOfSteps            = self.GetNSimulationSteps()
        self.simulationSettings.timeIntegration.endTime                  = self.endTime
        self.simulationSettings.timeIntegration.verboseModeFile          = 0
        self.simulationSettings.timeIntegration.verboseMode              = self.verboseMode
        self.simulationSettings.timeIntegration.newton.useModifiedNewton = True
        self.simulationSettings.linearSolverType                         = exu.LinearSolverType.EigenSparse
        self.simulationSettings.timeIntegration.stepInformation         += 8
        self.simulationSettings.displayStatistics                        = True
        # self.simulationSettings.displayComputationTime                   = True
        self.simulationSettings.linearSolverSettings.ignoreSingularJacobian=True
        self.simulationSettings.timeIntegration.generalizedAlpha.spectralRadius  = 0.9
        self.SC.visualizationSettings.nodes.show = False
        #self.SC.visualizationSettings.contour.outputVariable = varType1
        
        if self.Flexible:
            self.SC.visualizationSettings.contour.outputVariable = varType2
        
        if self.Visualization:
            self.SC.visualizationSettings.window.renderWindowSize            = [1600, 1200]        
            self.SC.visualizationSettings.openGL.multiSampling               = 4        
            self.SC.visualizationSettings.openGL.lineWidth                   = 3  
            self.SC.visualizationSettings.general.autoFitScene               = False      
            self.SC.visualizationSettings.nodes.drawNodesAsPoint             = False        
            self.SC.visualizationSettings.nodes.showBasis                    = True 
            #self.SC.visualizationSettings.markers.                    = True
            exu.StartRenderer()
        
        if self.StaticCase or self.StaticInitialization:
            self.mbs.variables['isStatics'] = True
            self.simulationSettings.staticSolver.newton.relativeTolerance = 1e-10
            # self.simulationSettings.staticSolver.stabilizerODE2term = 2
            self.simulationSettings.staticSolver.verboseMode = self.verboseMode
            self.simulationSettings.staticSolver.numberOfLoadSteps = 10
            self.simulationSettings.staticSolver.constrainODE1coordinates = True #constrain pressures to initial values
        
            exu.SuppressWarnings(True)
            self.mbs.SolveStatic(self.simulationSettings, 
                            updateInitialValues=True) #use solution as new initial values for next simulation
            exu.SuppressWarnings(False)
            
            #now deactivate distance constraint:
            # self.mbs.SetObjectParameter(oDC, 'activeConnector', False)
            force = self.mbs.GetObjectOutput(oDC, variableType=exu.OutputVariableType.Force)
            if self.verboseMode:
                print('initial force=', force)
            
            #deactivate distance constraint
            self.mbs.SetObjectParameter(oDC, 'activeConnector', False)
            
            #overwrite pressures:
            if oHA1 != None:
                dictHA1 = self.mbs.GetObject(oHA1)
                #print('dictHA1=',dictHA1)
                nodeHA1 = dictHA1['nodeNumbers'][0]
                A_HA1 = dictHA1['chamberCrossSection0']
                pDiff = force/A_HA1
            
                # #now we would like to reset the pressures:
                # #2) change the initial values in the system vector
            
                sysODE1 = self.mbs.systemData.GetODE1Coordinates(configuration=exu.ConfigurationType.Initial)
                nODE1index = self.mbs.GetNodeODE1Index(nodeHA1) #coordinate index for node nodaHA1
                if self.verboseMode:
                    # print('sysODE1=',sysODE1)
                    print('p0,p1=',sysODE1[nODE1index],sysODE1[nODE1index+1])
                sysODE1[nODE1index] += pDiff #add required difference to pressure
    
                #now write the updated system variables:
                self.mbs.systemData.SetODE1Coordinates(coordinates=sysODE1, configuration=exu.ConfigurationType.Initial)
    
                if self.verboseMode:
                    print('new p0,p1=',sysODE1[nODE1index],sysODE1[nODE1index+1])

            self.mbs.variables['isStatics'] = False


        if not self.StaticCase:
            exu.SolveDynamic(self.mbs, simulationSettings=self.simulationSettings,
                                solverType=exu.DynamicSolverType.TrapezoidalIndex2)
            
            

        if self.Visualization:
            # self.SC.WaitForRenderEngineStopFlag()
            exu.StopRenderer()
    


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
                            # --PATU CRANE---
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
def PatuCrane(self, theta1,theta2, p1, p2,p3, p4):


       self.theta1      = theta1
       self.theta2      = theta2
       self.p1          = p1
       self.p2          = p2    
       self.p3          = p3
       self.p4          = p4
                     
       self.dictSensors = {}
       
       self.StaticInitialization = True
       Emodulus                  = 2.1e11
       nu                        = 0.3
       rho                       = 7850
       mat                       = KirchhoffMaterial(Emodulus, nu, rho)
       varType1                  = exu.OutputVariableType.StrainLocal
   
       #Ground body
       oGround                   = self.mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
                                                                          visualization=VObjectGround(graphicsData=[plane])))
       markerGround    = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
       iCube1          = RigidBodyInertia(mass=m1, com=pMid1,inertiaTensor=Inertia1,inertiaTensorAtCOM=True)
       graphicsCOM1    = GraphicsDataBasis(origin=iCube1.com, length=2*L1)
       
       # Definintion of pillar as body in Exudyn and node n1
       [n1, b1]        = AddRigidBody(mainSys=self.mbs,inertia=iCube1,nodeType=exu.NodeType.RotationEulerParameters,
                                    position=PillarP,rotationMatrix=np.diag([1, 1, 1]),gravity=[0, -9.8066, 0] ,graphicsDataList=[graphicsCOM1, graphicsBody1]
                                    )
       Marker3         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=Mark3))                     #With Ground
       Marker4         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=Mark4))            #Lift Boom
       Marker5         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=Mark5))        # Cylinder 1 position
       
       # Fixed joint between Pillar and Ground
       self.mbs.AddObject(GenericJoint(markerNumbers=[markerGround, Marker3],constrainedAxes=[1, 1, 1, 1, 1, 1],
                                       visualization=VObjectJointGeneric(axesRadius=0.2*W1,axesLength=1.4*W1)))

       if self.Flexible:
           filePath        = 'AbaqusMesh/Job-1'
           filePath2       = 'AbaqusMesh/Job-2'
  
           if not self.loadFromSavedNPY: 
               start_time      = time.time()
          
               nodes1          = feL.ImportFromAbaqusInputFile(filePath+'.inp', typeName='Part', name='Job-1')
               feL.ReadMassMatrixFromAbaqus(fileName=filePath + '_MASS2.mtx')             #Load mass matrix
               feL.ReadStiffnessMatrixFromAbaqus(fileName=filePath + '_STIF2.mtx')        #Load stiffness matrix
               feL.SaveToFile(filePath,mode='PKL')
          
               nodes2          = feT.ImportFromAbaqusInputFile(filePath2+'.inp', typeName='Part', name='Job-2')
               feT.ReadMassMatrixFromAbaqus(fileName=filePath2 + '_MASS2.mtx')             #Load mass matrix
               feT.ReadStiffnessMatrixFromAbaqus(fileName=filePath2 + '_STIF2.mtx')        #Load stiffness matrix
               feT.SaveToFile(filePath2,mode='PKL')
          
               if self.verboseMode:
                  print("--- saving LiftBoom FEM Abaqus data took: %s seconds ---" % (time.time() - start_time)) 
                  
           else:       
               if self.verboseMode:
                   print('importing Abaqus FEM data structure of Lift Boom...')
               
               start_time = time.time()
          
               feL.LoadFromFile(filePath,mode='PKL')
               feT.LoadFromFile(filePath2,mode='PKL')
          
               cpuTime = time.time() - start_time
               if self.verboseMode:
                   print("--- importing FEM data took: %s seconds ---" % (cpuTime))
                   
       
                   
           p2                  = [0, 0,-100*1e-3]
           p1                  = [0, 0, 100*1e-3]
           radius1             = 25*1e-3
           nodeListJoint1      = feL.GetNodesOnCylinder(p1, p2, radius1, tolerance=1e-4) 
           pJoint1             = feL.GetNodePositionsMean(nodeListJoint1)
           nodeListJoint1Len   = len(nodeListJoint1)
           noodeWeightsJoint1  = [1/nodeListJoint1Len]*nodeListJoint1Len
           noodeWeightsJoint1  =feL.GetNodeWeightsFromSurfaceAreas(nodeListJoint1)
           
       
           p4                  = [304.19*1e-3,-100.01*1e-3,-100*1e-3]
           p3                  = [304.19*1e-3,-100.01*1e-3, 100*1e-3]
           radius2             = 36*1e-3
           nodeListPist1       = feL.GetNodesOnCylinder(p3, p4, radius2, tolerance=1e-2)  
           pJoint2             = feL.GetNodePositionsMean(nodeListPist1)
           nodeListPist1Len    = len(nodeListPist1)
           noodeWeightsPist1   = [1/nodeListPist1Len]*nodeListPist1Len
       
           # Boundary condition at cylinder 1
           p6                  = [1258e-3,194.59e-3,  65.701e-3]
           p5                  = [1258e-3,194.59e-3, -65.701e-3]
           radius3             = 32e-3
           nodeListCyl2        = feL.GetNodesOnCylinder(p5, p6, radius3, tolerance=1e-2)  
           pJoint3             = feL.GetNodePositionsMean(nodeListCyl2)
           nodeListCyl2Len     = len(nodeListCyl2)
           noodeWeightsCyl2    = [1/nodeListCyl2Len]*nodeListCyl2Len 
                   
           # Boundary condition at Joint 2
           p8                  = [2685e-3,0.15e-03,  74e-3]
           p7                  = [2685e-3,0.15e-03, -74e-3]
           radius4             = 32e-3
           nodeListJoint2      = feL.GetNodesOnCylinder(p7, p8, radius4, tolerance=1e-4)  
           pJoint4             = feL.GetNodePositionsMean(nodeListJoint2)
           nodeListJoint2Len   = len(nodeListJoint2)
           noodeWeightsJoint2  = [1/nodeListJoint2Len]*nodeListJoint2Len
               
           # Joint 3
           p10                 = [2875e-3,15.15e-3,    74e-3]
           p9                  = [2875e-3,15.15e-3,   -74e-3]
           radius5             = 4.60e-002
           nodeListJoint3      = feL.GetNodesOnCylinder(p9, p10, radius5, tolerance=1e-4)  
           pJoint5             = feL.GetNodePositionsMean(nodeListJoint3)
           nodeListJoint3Len   = len(nodeListJoint3)
           noodeWeightsJoint3  = [1/nodeListJoint3Len]*nodeListJoint3Len
               
           # Boundary condition at pillar
           p12                 = [0, 0,  88e-3]
           p11                 = [0, 0, -88e-3]
           radius6             = 48e-3
           nodeListJoint1T     = feT.GetNodesOnCylinder(p11, p12, radius6, tolerance=1e-4) 
           pJoint1T            = feT.GetNodePositionsMean(nodeListJoint1T)
           nodeListJoint1TLen  = len(nodeListJoint1T)
           noodeWeightsJoint1T = [1/nodeListJoint1TLen]*nodeListJoint1TLen
                   
           # Boundary condition at Piston 1
           p14                 = [-95e-3,243.2e-3,  55.511e-3]
           p13                 = [-95e-3,243.2e-3, -55.511e-3]
           radius7             = 26e-3
           nodeListPist1T      = feT.GetNodesOnCylinder(p13, p14, radius7, tolerance=1e-4)  
           pJoint2T            = feT.GetNodePositionsMean(nodeListPist1T)
           nodeListPist1TLen   = len(nodeListPist1T)
           noodeWeightsPist1T  = [1/nodeListPist1TLen]*nodeListPist1TLen

           # Boundary condition at extension boom
           p16                 = [-415e-3,287e-3, 48.011e-3]
           p15                 = [-415e-3,287e-3, -48.011e-3]
           radius8             = 2.3e-002
           nodeListExtT        = feT.GetNodesOnCylinder(p15, p16, radius8, tolerance=1e-4)  
           pExtT               = feT.GetNodePositionsMean(nodeListExtT)
           nodeListExTLen      = len(nodeListExtT)
           noodeWeightsExt1T   = [1/nodeListExTLen]*nodeListExTLen
           
       
           # STEP 2: Craig-Bampton Modes
           boundaryListL   = [nodeListJoint1,nodeListPist1, nodeListJoint2,  nodeListJoint3] 
           boundaryListT  = [nodeListJoint1T, nodeListPist1T,nodeListExtT]
   
           start_time          = time.time()

           if self.loadFromSavedNPY:
               feL.LoadFromFile('AbaqusMesh/feL',mode='PKL')
               feT.LoadFromFile('AbaqusMesh/feT',mode='PKL')

          
           else:
               feL.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryListL, nEigenModes=self.nModes, 
                                                   useSparseSolver=True,computationMode = HCBstaticModeSelection.RBE2) 
               feT.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryListT, nEigenModes=self.nModes, 
                                                         useSparseSolver=True,computationMode = HCBstaticModeSelection.RBE2) 
         
               print("ComputePostProcessingModes ... (may take a while)")
               feL.ComputePostProcessingModes(material=mat,outputVariableType=varType1,)
               feT.ComputePostProcessingModes(material=mat,outputVariableType=varType1,)
        
               feL.SaveToFile('AbaqusMesh/feL', mode='PKL')
               feT.SaveToFile('AbaqusMesh/feT', mode='PKL')

           if self.verboseMode:
               print("Hurty-Craig Bampton modes... ")
               print("eigen freq.=", feL.GetEigenFrequenciesHz())
               print("eigen freq.=", feT.GetEigenFrequenciesHz())
           
               print("HCB modes needed %.3f seconds" % (time.time() - start_time)) 
               
           colLift = color4blue
           LiftBoom            = ObjectFFRFreducedOrderInterface(feL)
           TiltBoom            = ObjectFFRFreducedOrderInterface(feT)
           LiftBoomFFRF        = LiftBoom.AddObjectFFRFreducedOrder(self.mbs, positionRef=np.array(Mark4), initialVelocity=[0,0,0], 
                                              initialAngularVelocity=[0,0,0], rotationMatrixRef  = RotationMatrixZ(mt.radians(self.theta1)),
                                              gravity= [0, -9.8066, 0],#massProportionalDamping = 0, stiffnessProportionalDamping = 1e-5 ,
                                             massProportionalDamping = 0, stiffnessProportionalDamping = 3.35e-3 ,color=colLift,)
           self.mbs.SetObjectParameter(objectNumber=LiftBoomFFRF['oFFRFreducedOrder'],parameterName='outputVariableTypeModeBasis',
                                           value=varType1)
   
           Marker7             = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'],
                                         meshNodeNumbers=np.array(nodeListJoint1), #these are the meshNodeNumbers
                                         weightingFactors=noodeWeightsJoint1))
           Marker8             = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'],
                                         meshNodeNumbers=np.array(nodeListPist1), #these are the meshNodeNumbers
                                         weightingFactors=noodeWeightsPist1))
       
           Marker9             = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'],
                                                      meshNodeNumbers=np.array(nodeListCyl2), #these are the meshNodeNumbers
                                                      weightingFactors=noodeWeightsCyl2)) 
           Marker10        = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'],
                                                      meshNodeNumbers=np.array(nodeListJoint2), #these are the meshNodeNumbers
                                                      weightingFactors=noodeWeightsJoint2))      
               
           Marker11        = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'],
                                                    meshNodeNumbers=np.array(nodeListJoint3), #these are the meshNodeNumbers
                                                    weightingFactors=noodeWeightsJoint3))
      
       else:
           
           iCube2          = RigidBodyInertia(mass=m2, com=pMid2,inertiaTensor=Inertia2,inertiaTensorAtCOM=True)
           graphicsCOM2    = GraphicsDataBasis(origin=iCube2.com, length=2*W2)
           [n2, b2]        = AddRigidBody(mainSys=self.mbs,inertia=iCube2,nodeType=exu.NodeType.RotationEulerParameters,position=LiftP,  
                                               rotationMatrix= RotationMatrixZ(mt.radians(self.theta1)), gravity= [0, -9.8066, 0], graphicsDataList=[graphicsCOM2, graphicsBody2])
        
           Marker7         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0, 0, 0]))                       #With Pillar    
           Marker8         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0.3025, -0.105, 0]))             #With Cylinder 1
           Marker9         = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[1.263, 0.206702194, 0]))         #With Cylinder 2
           Marker10        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[2.69, -0.006592554, 0]))         #With Bracket 1  
           Marker11        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[2.881080943, 0.021592554, 0]))   #With Tilt Boom
           
      
       if self.StaticCase or self.StaticInitialization:
                #compute reference length of distance constraint 
                self.mbs.Assemble()
                TiltP = self.mbs.GetMarkerOutput(Marker11, variableType=exu.OutputVariableType.Position, configuration=exu.ConfigurationType.Initial)
       
       if not self.Flexible: 
           iCube3          = RigidBodyInertia(mass=m3, com=pMid3, inertiaTensor=Inertia3,inertiaTensorAtCOM=True)
           graphicsCOM3    = GraphicsDataBasis(origin=iCube3.com, length=2*W3)
           [n3, b3]        = AddRigidBody(mainSys=self.mbs,inertia=iCube3, nodeType=exu.NodeType.RotationEulerParameters,
                                   position=TiltP, rotationMatrix=RotationMatrixZ(mt.radians(self.theta2)),
                                   gravity= [0, -9.8066, 0],graphicsDataList=[graphicsCOM3, graphicsBody3])
           Marker13        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=[0, 0, 0]))                        #With LIft Boom 
           Marker14        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=[-0.095, 0.24043237, 0])) 
           MarkerEx        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=[-0.415,0.295, 0]))
           
       else:    
           TiltBoomFFRF        = TiltBoom.AddObjectFFRFreducedOrder(self.mbs, positionRef=TiltP, #2.879420180699481+27e-3, -0.040690041435711005+8.3e-2, 0
                                                          initialVelocity=[0,0,0], 
                                                          initialAngularVelocity=[0,0,0],
                                                          rotationMatrixRef  = RotationMatrixZ(mt.radians(self.theta2))   ,
                                                          gravity= [0, -9.8066, 0],
                                                          #massProportionalDamping = 0, stiffnessProportionalDamping = 1e-5 ,
                                                         massProportionalDamping = 0, stiffnessProportionalDamping = 3.35e-3 ,
                                                          color=colLift,)
       
           self.mbs.SetObjectParameter(objectNumber=TiltBoomFFRF['oFFRFreducedOrder'],parameterName='outputVariableTypeModeBasis',
                                           value=varType1)
       
           Marker13        = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=TiltBoomFFRF['oFFRFreducedOrder'], 
                                                                             meshNodeNumbers=np.array(nodeListJoint1T), #these are the meshNodeNumbers
                                                                             weightingFactors=noodeWeightsJoint1T))
           
           Marker14        = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=TiltBoomFFRF['oFFRFreducedOrder'], 
                                                                             meshNodeNumbers=np.array(nodeListPist1T), #these are the meshNodeNumbers
                                                                             weightingFactors=noodeWeightsPist1T))  
       
           MarkerEx        = self.mbs.AddMarker(MarkerSuperElementRigid(bodyNumber=TiltBoomFFRF['oFFRFreducedOrder'], 
                                                                             meshNodeNumbers=np.array(nodeListExtT), #these are the meshNodeNumbers
                                                                             weightingFactors=noodeWeightsExt1T))  
       
           StrainPoint     = feT.GetNodeAtPoint(np.array([0.241222218,  0.347000003, 0.0390110984]))
       
       #Revolute Joint
       self.mbs.AddObject(GenericJoint(markerNumbers=[Marker4, Marker7],constrainedAxes=[1,1,1,1,1,0],
                                       visualization=VObjectJointGeneric(axesRadius=0.18*0.263342,axesLength=1.1*0.263342)))
       
       ###########################################
       if self.StaticCase or self.StaticInitialization:
           #compute reference length of distance constraint 
           self.mbs.Assemble()
           Bracket1L = self.mbs.GetMarkerOutput(Marker10, variableType=exu.OutputVariableType.Position, 
                                            configuration=exu.ConfigurationType.Initial)
           
           Bracket1B = self.mbs.GetMarkerOutput(Marker14, variableType=exu.OutputVariableType.Position, 
                                            configuration=exu.ConfigurationType.Initial)
           
           ExtensionP = self.mbs.GetMarkerOutput(MarkerEx, variableType=exu.OutputVariableType.Position, 
                                            configuration=exu.ConfigurationType.Initial)
           
           def AnalyticalEq(p,l3, l4,l2x,l2y, d1,h1):
               theta3_degrees, theta4_degrees = p
               
               # if theta3_degrees < 125:
               #     theta3_degrees = 125
               # elif theta3_degrees > 150:
               #     theta3_degrees = 150

               # if theta4_degrees < 120:
               #     theta4_degrees = 120
               # elif theta4_degrees > 225:
               #     theta4_degrees = 225
               
               theta3 = np.radians(theta3_degrees)
               theta4 = np.radians(theta4_degrees) 
                               
               eq1 =    l3 * cos(theta3) + l4 * cos(np.pi-theta4) + l2x +d1
               eq2 =    l3 * sin(theta3) + l4 * sin(np.pi-theta4) + l2y +h1
               
               return [eq1, eq2]
           
            
           l3         = NormL2(np.array([ 0,  0, 0]) - np.array([0.456, -0.0405, 0]))  
           l4         = NormL2(np.array([ 0,  0, 0]) - np.array([0.48, 0, 0]))  
           d1         = Bracket1L[0]-TiltP[0]
           h1         = Bracket1L[1]-TiltP[1]
           l2x        = -(Bracket1B[0]-TiltP[0])
           l2y        = -(Bracket1B[1]-TiltP[1])
           # alpha0     = degrees(atan2((Bracket1B[1] - TiltP[1]), (Bracket1B[0] - TiltP[0])))
            
           initialangles  = [130.562128044041, 180.26679642675617]
           solutions = fsolve(AnalyticalEq, initialangles,args=(l3, l4, l2x, l2y, d1, h1))
           self.theta3, self.theta4 = solutions
           
       pMid4           = np.array([0.257068, 0.004000 , 0])  # center of mass, body0,0.004000,-0.257068
       iCube4          = RigidBodyInertia(mass=11.524039, com=pMid4,
                                                     inertiaTensor=np.array([[0.333066, 0.017355, 0],
                                                                             [0.017355, 0.081849, 0],
                                                                             [0,              0, 0.268644]]),
                                                                               inertiaTensorAtCOM=True)

       graphicsBody4   = GraphicsDataFromSTLfile(fileName4, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
       graphicsBody4   = AddEdgesAndSmoothenNormals(graphicsBody4, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)
       graphicsCOM4    = GraphicsDataBasis(origin=iCube4.com, length=2*W4)
       [n4, b4]        = AddRigidBody(mainSys=self.mbs,inertia=iCube4,  # includes COM
                                                   nodeType=exu.NodeType.RotationEulerParameters,
                                                   position=Bracket1L,  # pMid2
                                                   rotationMatrix=RotationMatrixZ(mt.radians(157.11555963638386)), #-0.414835768117858,self.theta3+3.5
                                                   gravity= [0, -9.8066, 0],
                                                   graphicsDataList=[graphicsCOM4, graphicsBody4])
                 
        # # # # 5th Body: Bracket 2
       pMid5           = np.array([0.212792, 0, 0])  # center of mass, body0
       iCube5          = RigidBodyInertia(mass=7.900191, com=pMid5,
                                                       inertiaTensor=np.array([[0.052095, 0, 0],
                                                                               [0,  0.260808, 0],
                                                                               [0,              0,  0.216772]]),
                                                                               inertiaTensorAtCOM=True)
                 
       graphicsBody5   = GraphicsDataFromSTLfile(fileName5, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
       graphicsBody5   = AddEdgesAndSmoothenNormals(graphicsBody5, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)
       graphicsCOM5    = GraphicsDataBasis(origin=iCube5.com, length=2*W5)
       [n5, b5]        = AddRigidBody(mainSys=self.mbs,inertia=iCube5,  # includes COM
                                                   nodeType=exu.NodeType.RotationEulerParameters,
                                                   position=Bracket1B,  # pMid2
                                                   rotationMatrix=RotationMatrixZ(mt.radians(194.03103052691895)) ,   #-5, 140
                                                   gravity= [0, -9.8066, 0],graphicsDataList=[graphicsCOM5, graphicsBody5])
                 
             
                 
       Marker15        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b4, localPosition=[0, 0, 0]))                        #With LIft Boom 
       Marker16        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b4, localPosition=[0.456, -0.0405 , 0]))  
       Marker18        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b5, localPosition=[0, 0, 0]))                        #With LIft Boom 
       Marker19        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b5, localPosition=[0.475+5e-3, 0, 0]))                   #With LIft Boom,-0.475 

       # # # # 5th Body: Bracket 2
       pMid6           = np.array([1.15, 0.06, 0])  # center of mass, body0
       iCube6          = RigidBodyInertia(mass=58.63, com=pMid6,
                                                inertiaTensor=np.array([[0.13, 0, 0],
                                                                        [0,  28.66, 0],
                                                                        [0,      0,  28.70]]),
                                                                        inertiaTensorAtCOM=True)
          
       graphicsBody6   = GraphicsDataFromSTLfile(fileName6, color4blue,verbose=False, invertNormals=True,invertTriangles=True)
       graphicsBody6   = AddEdgesAndSmoothenNormals(graphicsBody6, edgeAngle=0.25*pi,addEdges=True, smoothNormals=True)
       graphicsCOM6    = GraphicsDataBasis(origin=iCube6.com, length=2*W5)
       [n6, b6]        = AddRigidBody(mainSys=self.mbs,inertia=iCube6,  # includes COM
                                            nodeType=exu.NodeType.RotationEulerParameters,
                                            position=ExtensionP+np.array([0, -0.10, 0]),  # pMid2
                                            rotationMatrix=RotationMatrixZ(mt.radians(self.theta2)) ,   #-5, 140
                                            gravity= [0, -9.8066, 0],graphicsDataList=[graphicsCOM6, graphicsBody6]) 
         
       Marker20        = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b6, localPosition=[0, 0.1, 0]))
         
       if self.mL != 0:
             TipLoadMarker     = self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=b6, localPosition=[2.45, 0.05, 0]))
             pos = self.mbs.GetMarkerOutput(TipLoadMarker, variableType=exu.OutputVariableType.Position, 
                                            configuration=exu.ConfigurationType.Reference)
             #print('pos=', pos)
             bMass = self.mbs.CreateMassPoint(physicsMass=self.mL, referencePosition=pos,gravity=[0, -9.8066, 0], show=True,
                                      graphicsDataList=[GraphicsDataSphere(radius=0.1, color=color4red)])
             mMass = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=bMass))
             self.mbs.AddObject(SphericalJoint(markerNumbers=[TipLoadMarker, mMass], visualization=VSphericalJoint(show=False)))
             
             #self.mbs.Assemble()
             #exu.StartRenderer()
             
       # #Add Revolute Joint btw Pillar and LiftBoom
       self.mbs.AddObject(GenericJoint(markerNumbers=[MarkerEx, Marker20],constrainedAxes=[1,1,1,1,1,1],
                                 visualization=VObjectJointGeneric(axesRadius=0.18*W2,axesLength=0.18)))   
               
       # # #Add Revolute Joint btw LiftBoom and TiltBoom
       self.mbs.AddObject(GenericJoint(markerNumbers=[Marker11, Marker13],constrainedAxes=[1,1,1,1,1,0],
                                   visualization=VObjectJointGeneric(axesRadius=0.22*W3,axesLength=0.16)))     
       
        
       # # # # #Add Revolute Joint btw LiftBoom and Bracket 1
       self.mbs.AddObject(GenericJoint(markerNumbers=[Marker10, Marker15],constrainedAxes=[1,1,1,1,1,0],
                                 visualization=VObjectJointGeneric(axesRadius=0.32*W4,axesLength=0.20)))   
               
       # # # #Add Revolute Joint btw Bracket 1 and Bracket 2
       self.mbs.AddObject(GenericJoint(markerNumbers=[Marker16, Marker19],constrainedAxes=[1,1,1,1,1,0],
                                   visualization=VObjectJointGeneric(axesRadius=0.28*W5,axesLength=2.0*W5)))  
               
       # # # # Revolute joint between Bracket 2 and TiltBoom
       self.mbs.AddObject(GenericJoint(markerNumbers=[Marker18, Marker14],constrainedAxes=[1,1,0,0,0,0],
                                       visualization=VObjectJointGeneric(axesRadius=0.23*W5,axesLength=0.20)))  

       
       
       #self.mbs.AddLoad(LoadForceVector(markerNumber=Marker9, bodyFixed=True, loadVector=[0,-self.mL,0]))
       colCyl              = color4orange
       colPis              = color4grey 
       LH1                 = L_Cyl1                        #zero length of actuator
       LH2                 = L_Cyl2                        #zero length of actuator
           
       #ODE1 for pressures:
       nODE1               = self.mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0],
                                           initialCoordinates=[self.p1,
                                                               self.p2],  # initialize with 20 bar
                                                               numberOfODE1Coordinates=2))

       nODE2               = self.mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0, 0],
                                           initialCoordinates=[self.p3,
                                                               self.p4],  # initialize with 20 bar
                                                               numberOfODE1Coordinates=2))
            
       def CylinderFriction1(mbs, t, itemNumber, u, v, k, d, F0):

           Ff = 1*StribeckFunction(v, muDynamic=1, muStaticOffset=1.5, regVel=1e-4)+(k*(u) + d*v + k*(u)**3-F0)
           #print(Ff)
           return Ff*1
                
       def CylinderFriction2(mbs, t, itemNumber, u, v, k, d, F0):

           Ff =  1*StribeckFunction(v, muDynamic=0.5, muStaticOffset=0.5, regVel=1e-2) - (k*(u) - d*v + k*(u)**3 -F0)
           #print(Ff)
           return Ff*1
                
                
       # def UFfrictionSpringDamper1(mbs, t, itemIndex, u, v, k, d, f0): 
       #     return   1*(Fc*tanh(4*(abs(v    )/vs))+(Fs-Fc)*((abs(v    )/vs)/((1/4)*(abs(v    )/vs)**2+3/4)**2))*np.sign(v )+sig2*v    *tanh(4)

                    
       # def UFfrictionSpringDamper2(mbs, t, itemIndex, u, v, k, d, f0): 
       #     return   1*(Fc*tanh(4*(abs(v    )/vs))+(Fs-Fc)*((abs(v    )/vs)/((1/4)*(abs(v    )/vs)**2+3/4)**2))*np.sign(v )+sig2*v    *tanh(4)
                    
                    
       oFriction1       = self.mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[Marker5, Marker8], referenceLength=0,stiffness=2000,
                                                                 damping=0, force=0, velocityOffset = 0., activeConnector = True,
                                                                 springForceUserFunction=CylinderFriction1   , #CylinderFriction1,
                                                                   visualization=VSpringDamper(show=False) ))
                             
       oFriction2       = self.mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[Marker9, Marker16], referenceLength=0,stiffness=1250,
                                                                   damping=0, force=0, velocityOffset = 0, activeConnector = True,
                                                                   springForceUserFunction= CylinderFriction2, #CylinderFriction2,
                                                                     visualization=VSpringDamper(show=False) ))
       
      
       oHA1 = None
       oHA2 = None
       
       
       if True:
           oHA1                = self.mbs.AddObject(HydraulicActuatorSimple(name='LiftCylinder', markerNumbers=[ Marker5, Marker8], 
                                                   nodeNumbers=[nODE1], offsetLength=L_Cyl1, strokeLength=L_Pis1, chamberCrossSection0=A[0], 
                                                   chamberCrossSection1=A[1], hoseVolume0=V1, hoseVolume1=V2, valveOpening0=0, 
                                                   valveOpening1=0, actuatorDamping=3e5, oilBulkModulus=Bo, cylinderBulkModulus=Bc, 
                                                   hoseBulkModulus=Bh, nominalFlow=Qn1, systemPressure=pS, tankPressure=pT, 
                                                   useChamberVolumeChange=True, activeConnector=True, 
                                                   visualization={'show': True, 'cylinderRadius': 50e-3, 'rodRadius': 28e-3, 
                                                                   'pistonRadius': 0.04, 'pistonLength': 0.001, 'rodMountRadius': 0.0, 
                                                                   'baseMountRadius': 20.0e-3, 'baseMountLength': 20.0e-3, 'colorCylinder': color4orange,
                                                               'colorPiston': color4grey}))
           
           oHA2 = self.mbs.AddObject(HydraulicActuatorSimple(name='TiltCylinder', markerNumbers=[Marker9, Marker16], 
                                                     nodeNumbers=[nODE2], offsetLength=LH2, strokeLength=L_Pis2, chamberCrossSection0=A[0], 
                                                     chamberCrossSection1=A[1], hoseVolume0=V1, hoseVolume1=V2, valveOpening0=0, 
                                                     valveOpening1=0, actuatorDamping=5e4, oilBulkModulus=Bo, cylinderBulkModulus=Bc, 
                                                     hoseBulkModulus=Bh, nominalFlow=Qn2, systemPressure=pS, tankPressure=pT, 
                                                     useChamberVolumeChange=True, activeConnector=True, 
                                                     visualization={'show': True, 'cylinderRadius': 50e-3, 'rodRadius': 28e-3, 
                                                                     'pistonRadius': 0.04, 'pistonLength': 0.001, 'rodMountRadius': 0.0, 
                                                                     'baseMountRadius': 0.0, 'baseMountLength': 0.0, 'colorCylinder': color4orange,
                                                                     'colorPiston': color4grey}))
       
           self.oHA1 = oHA1
           self.oHA2 = oHA2
           
       if self.StaticCase or self.StaticInitialization:
              #compute reference length of distance constraint 
              self.mbs.Assemble()
              mGHposition = self.mbs.GetMarkerOutput(Marker5, variableType=exu.OutputVariableType.Position, 
                                               configuration=exu.ConfigurationType.Initial)
              mRHposition = self.mbs.GetMarkerOutput(Marker8, variableType=exu.OutputVariableType.Position, 
                                               configuration=exu.ConfigurationType.Initial)
              
              mGHpositionT = self.mbs.GetMarkerOutput(Marker9, variableType=exu.OutputVariableType.Position, 
                                               configuration=exu.ConfigurationType.Initial)
              mRHpositionT = self.mbs.GetMarkerOutput(Marker16, variableType=exu.OutputVariableType.Position, 
                                               configuration=exu.ConfigurationType.Initial)
              
              
              dLH0 = NormL2(mGHposition - mRHposition)
              dLH1 = NormL2(mGHpositionT - mRHpositionT)
              
              if self.verboseMode:
                  print('dLH0=', dLH0)
              
              oDC = self.mbs.AddObject(DistanceConstraint(markerNumbers=[Marker5, Marker8], distance=dLH0))
              oDCT = self.mbs.AddObject(DistanceConstraint(markerNumbers=[Marker9, Marker16], distance=dLH1))

       
       self.mbs.variables['isStatics'] = False
       from exudyn.signalProcessing import GetInterpolatedSignalValue
       
       def PreStepUserFunction(mbs, t):
           if not mbs.variables['isStatics']: 
               Av0 = GetInterpolatedSignalValue(t, mbs.variables['inputTimeU1'], timeArray= [], dataArrayIndex= 1, 
                                       timeArrayIndex= 0, rangeWarning= False)
               Av2 = GetInterpolatedSignalValue(t, mbs.variables['inputTimeU2'], timeArray= [], dataArrayIndex= 1, 
                                              timeArrayIndex= 0, rangeWarning= False)
           
               # pP = GetInterpolatedSignalValue(t, mbs.variables['PUMP'], timeArray= [], dataArrayIndex= 1, 
               #                              timeArrayIndex= 0, rangeWarning= False)

               Av1 = -Av0
               Av3 = -Av2
      
               if oHA1 and oHA2 != None:
                  mbs.SetObjectParameter(oHA1, "valveOpening0", Av0)
                  mbs.SetObjectParameter(oHA1, "valveOpening1", Av1)
                  mbs.SetObjectParameter(oHA2, "valveOpening0", Av2)
                  mbs.SetObjectParameter(oHA2, "valveOpening1", Av3)
                  
                  # mbs.SetObjectParameter(oHA1, "systemPressure", pP)
                  # mbs.SetObjectParameter(oHA2, "systemPressure", pP)
           
           return True

       self.mbs.SetPreStepUserFunction(PreStepUserFunction)  
   
       if self.verboseMode:
           print('#joint nodes=',len(nodeListJoint3))

       if self.Flexible:
           StressNode  = feL.GetNodeAtPoint(np.array([0.639392078,  0.110807151, 0.0799999982]))
       
           if self.verboseMode:
               print("nMid=",nMid)
               print("nMid=",MarkerTip)

   
           # Add Sensor for deflection
           StrainF2             = self.mbs.AddSensor(SensorSuperElement(bodyNumber=TiltBoomFFRF['oFFRFreducedOrder'], meshNodeNumber=StrainPoint, storeInternal=True, outputVariableType=varType1))     
           StrainF1            = self.mbs.AddSensor(SensorSuperElement(bodyNumber=LiftBoomFFRF['oFFRFreducedOrder'], meshNodeNumber=StressNode, storeInternal=True, outputVariableType=varType1 ))
       
           def UFStressData(mbs, t, sensorNumbers, factors, configuration):
                val = mbs.GetSensorValues(sensorNumbers[0])
                StressVec = mat.StrainVector2StressVector(val)
                return StressVec
       
           
           self.dictSensors['StrainF1']       = StrainF1
           self.dictSensors['StrainF2']       = StrainF2
           self.dictSensors['Stress1']       = self.mbs.AddSensor(SensorUserFunction(sensorNumbers=[StrainF1], storeInternal=True, sensorUserFunction=UFStressData))
           self.dictSensors['Stress2']       = self.mbs.AddSensor(SensorUserFunction(sensorNumbers=[StrainF2], storeInternal=True, sensorUserFunction=UFStressData))
          
       if oHA1 and oHA2 != None:
           sForce1          = self.mbs.AddSensor(SensorObject(objectNumber=oHA1, storeInternal=True, 
                                                              outputVariableType=exu.OutputVariableType.Force))
           sForce2          = self.mbs.AddSensor(SensorObject(objectNumber=oHA2, storeInternal=True,
                                                              outputVariableType=exu.OutputVariableType.Force))

           self.dictSensors['sForce1']=sForce1
           self.dictSensors['sForce2']=sForce2

           sDistance1       = self.mbs.AddSensor(SensorObject(objectNumber=oHA1, storeInternal=True, 
                                                              outputVariableType=exu.OutputVariableType.Distance))
           sDistance2       = self.mbs.AddSensor(SensorObject(objectNumber=oHA2, storeInternal=True, 
                                                              outputVariableType=exu.OutputVariableType.Distance))

           self.dictSensors['sDistance1']=sDistance1
           self.dictSensors['sDistance2']=sDistance2

   
           sVelocity1       = self.mbs.AddSensor(SensorObject(objectNumber=oHA1, storeInternal=True, 
                                                              outputVariableType=exu.OutputVariableType.VelocityLocal))
           sVelocity2       = self.mbs.AddSensor(SensorObject(objectNumber=oHA2, storeInternal=True, 
                                                              outputVariableType=exu.OutputVariableType.VelocityLocal))

           self.dictSensors['sVelocity1']=sVelocity1
           self.dictSensors['sVelocity2']=sVelocity2
           
           sPressures1      = self.mbs.AddSensor(SensorNode(nodeNumber=nODE1, storeInternal=True,
                                                            outputVariableType=exu.OutputVariableType.Coordinates))   
           sPressures2      = self.mbs.AddSensor(SensorNode(nodeNumber=nODE2, storeInternal=True,
                                                            outputVariableType=exu.OutputVariableType.Coordinates))   

           self.dictSensors['sPressures1']=sPressures1
           self.dictSensors['sPressures2']=sPressures2


       #+++++++++++++++++++++++++++++++++++++++++++++++++++
       #assemble and solve    
       self.mbs.Assemble()
       
       self.simulationSettings = exu.SimulationSettings()   
       self.simulationSettings.solutionSettings.sensorsWritePeriod = self.endTime / (self.nStepsTotal)
       
       self.simulationSettings.timeIntegration.numberOfSteps            = self.GetNSimulationSteps()
       self.simulationSettings.timeIntegration.endTime                  = self.endTime
       self.simulationSettings.timeIntegration.verboseModeFile          = 0
       self.simulationSettings.timeIntegration.verboseMode              = self.verboseMode
       self.simulationSettings.timeIntegration.newton.useModifiedNewton = True
       self.simulationSettings.linearSolverType                         = exu.LinearSolverType.EigenSparse
       self.simulationSettings.timeIntegration.stepInformation         += 8
       self.simulationSettings.displayStatistics                        = True
       self.simulationSettings.displayComputationTime                   = True
       self.simulationSettings.linearSolverSettings.ignoreSingularJacobian=True
       self.simulationSettings.timeIntegration.generalizedAlpha.spectralRadius  = 0.7
       self.SC.visualizationSettings.nodes.show = False
       #self.SC.visualizationSettings.contour.outputVariable = varType1
       
       if self.Flexible:
           self.SC.visualizationSettings.contour.outputVariable = varType1
       
       if self.Visualization:
           self.SC.visualizationSettings.window.renderWindowSize            = [1600, 1200]        
           self.SC.visualizationSettings.openGL.multiSampling               = 4        
           self.SC.visualizationSettings.openGL.lineWidth                   = 3  
           self.SC.visualizationSettings.general.autoFitScene               = False      
           self.SC.visualizationSettings.nodes.drawNodesAsPoint             = False        
           self.SC.visualizationSettings.nodes.showBasis                    = True 
           #self.SC.visualizationSettings.markers.                    = True
           exu.StartRenderer()
       
       if self.StaticCase or self.StaticInitialization:
           self.mbs.variables['isStatics'] = True
           self.simulationSettings.staticSolver.newton.relativeTolerance = 1e-10
           # self.simulationSettings.staticSolver.stabilizerODE2term = 2
           self.simulationSettings.staticSolver.verboseMode = self.verboseMode
           self.simulationSettings.staticSolver.numberOfLoadSteps = 10
           self.simulationSettings.staticSolver.constrainODE1coordinates = True #constrain pressures to initial values
       
           exu.SuppressWarnings(True)
           self.mbs.SolveStatic(self.simulationSettings, 
                           updateInitialValues=True) #use solution as new initial values for next simulation
           exu.SuppressWarnings(False)
         
           force1 = self.mbs.GetObjectOutput(oDC, variableType=exu.OutputVariableType.Force)
           force2 = self.mbs.GetObjectOutput(oDCT, variableType=exu.OutputVariableType.Force)
           
           if self.verboseMode:
               print('initial force=', force)
           
           #deactivate distance constraint
           self.mbs.SetObjectParameter(oDC, 'activeConnector', False)
           self.mbs.SetObjectParameter(oDCT, 'activeConnector', False)
           
           #overwrite pressures:
           if oHA1 != None:
              dictHA1 = self.mbs.GetObject(oHA1)
              dictHA2 = self.mbs.GetObject(oHA2)
              
              nodeHA1 = dictHA1['nodeNumbers'][0]
              nodeHA2 = dictHA2['nodeNumbers'][0]
              
              A_HA1 = dictHA1['chamberCrossSection0']
              pDiff1 = force1/A_HA1
              pDiff2 = force2/A_HA1
          
              # #now we would like to reset the pressures:
              # #2) change the initial values in the system vector
          
              sysODE1 = self.mbs.systemData.GetODE1Coordinates(configuration=exu.ConfigurationType.Initial)
              
              nODE1index = self.mbs.GetNodeODE1Index(nodeHA1) #coordinate index for node nodaHA1
              nODE2index = self.mbs.GetNodeODE1Index(nodeHA2) #coordinate index for node nodaHA1
              
              if self.verboseMode:
                  # print('sysODE1=',sysODE1)
                  print('p0,p1=',sysODE1[nODE1index],sysODE1[nODE1index+1])
                  print('p2,p3=',sysODE1[nODE2index],sysODE1[nODE2index+1])
              
              sysODE1[nODE1index] += pDiff1 #add required difference to pressure
              sysODE1[nODE2index] += pDiff2 #add required difference to pressure

  
              #now write the updated system variables:
              self.mbs.systemData.SetODE1Coordinates(coordinates=sysODE1, 
                                                     configuration=exu.ConfigurationType.Initial)
  
              if self.verboseMode:
                  print('new p0,p1=',sysODE1[nODE1index],sysODE1[nODE1index+1])
                  print('new p2,p3=',sysODE1[nODE2index],sysODE1[nODE2index+1])

           self.mbs.variables['isStatics'] = False


       if not self.StaticCase:
           exu.SolveDynamic(self.mbs, simulationSettings=self.simulationSettings,
                               solverType=exu.DynamicSolverType.TrapezoidalIndex2)
           
           

       if self.Visualization:
           # self.SC.WaitForRenderEngineStopFlag()
           exu.StopRenderer()
   
    
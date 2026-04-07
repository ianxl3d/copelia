import math
# definition des codes d'etat
iIDLE           = 0       # mode attente
iPTP            = 1       # deplacement de type PTP en cours
iPTP_REL        = 2       # deplacement relatif de l'outil
iGRASP          = 3       # saisir un objet, s'il y a lieu
iDROP           = 4       # lacher un objet, s'il y a lieu
iWAIT           = 5       # attendre un certain temps
iSTOP           = 10      # mode stop

def sysCall_init():
    sim = require('sim')
    simIK = require('simIK')
    #---------------------------------------------------------------------------------------------------
    # Build a kinematic chain and 2 IK groups (undamped and damped) inside of the IK plugin environment,
    # based on the kinematics of the robot in the scene:
    #---------------------------------------------------------------------------------------------------
    self.hBase    = sim.getObject('.')
    self.hTool    = sim.getObject('./Joint/Joint/Joint/Joint/Joint/Joint/Link/tool')
    self.hTarget  = sim.getObject('./target')
    self.hContact = sim.getObject('./contact') 
    self.hObject  = -1  # DONNE LE HANDLE DE L OBJET DETECTE PAR LE CAPTEUR DE PROXIMITE
    self.hInHand  = -1  # HANDLE DE L'OBJET TENU PAR L'OUTIL
    # info de debug
    sim.addLog(1,"handle sur la base  = " + str(self.hBase))
    sim.addLog(1,"handle sur l outil  = " + str(self.hTool))
    sim.addLog(1,"handle sur la cible = " + str(self.hTarget))
    # Simple way:
    self.ikEnv = simIK.createEnvironment()
    self.ikGroup_undamped = simIK.createGroup(self.ikEnv)
    simIK.setGroupCalculation(self.ikEnv, self.ikGroup_undamped, simIK.method_pseudo_inverse, 0, 6)
    simIK.addElementFromScene(self.ikEnv, self.ikGroup_undamped, self.hBase, self.hTool, self.hTarget, simIK.constraint_pose)

    self.ikGroup_damped = simIK.createGroup(self.ikEnv)
    simIK.setGroupCalculation(self.ikEnv, self.ikGroup_damped, simIK.method_damped_least_squares, 1, 99)
    simIK.addElementFromScene(self.ikEnv, self.ikGroup_damped, self.hBase, self.hTool, self.hTarget, simIK.constraint_pose)
    # proprietes pour la gestion des mouvements 
    self.state = iIDLE                      # on ne fait rien au depart
    self.distance_parcoure    = 0.0    
    self.distance_a_parcourir = 0.0
    self.tolerance            = 0.0005      # tolerance sur la distance au point cible
    self.Vmax                 = 0.0
    self.goal                 = [0,0,0]
    self.Vmax                 = 0.3
    self.gain                 = 1.0
    self.waitTime             = 0.0         # duree d'attente restante (pour iWAIT)
    # proprietes pour la gestion du scripting simple : 
    self.iPC                  = -1          # pointeur d'instruction
    self.lInstructions = []
    self.lParameters   = []
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, 0.1, -0.1, 0.001, 0.05, 2])
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, -0.05, -0.02,0.001, 0.05, 2])
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, -0.05, 0.0,0.001, 0.05, 2])
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, 0.0, -0.05,0.001, 0.05, 2])
    self.lInstructions.append(iGRASP)
    self.lParameters.append(0)
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, 0.0, 0.1,0.001, 0.05, 2])
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, 0.0, -0.1,0.001, 0.05, 2])
    self.lInstructions.append(iDROP)
    self.lParameters.append(0)
    self.lInstructions.append(iPTP_REL)
    self.lParameters.append([0.0, 0.0, 0.1,0.001, 0.05, 2])
    self.lInstructions.append(iWAIT)
    self.lParameters.append(1)
    # test de deplacement 
    #PStart = sim.getObjectPosition( self.hTool )
    #PStart[0] += 0.1
    #PStart[2] += 0.1
    #demarre_PTP( PStart, 0.001, 0.05, 2)
    #PStart = [0.0, 0.1, -0.1]
    #demarre_PTP_REL( PStart, 0.001, 0.05, 2)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction "preparatoire" de deplacement 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def demarre_PTP( Pgoal , tolerance, Vmax, gain ):
    self.distance_parcourue = 0.0
    curPos = sim.getObjectPosition(self.hTool)  # hTool --> position de l'outil 
    dx = Pgoal[0] - curPos[0]
    dy = Pgoal[1] - curPos[1]
    dz = Pgoal[2] - curPos[2]
    self.distance_a_parcourir = math.sqrt(dx**2 + dy**2 + dz**2)
    self.goal[0]  = Pgoal[0]
    self.goal[1]  = Pgoal[1]
    self.goal[2]  = Pgoal[2]
    self.gain  = gain
    self.Vmax  = Vmax
    # mise a jour de l'etat 
    self.state = iPTP
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction "preparatoire" du deplacement
# en relatif
# ATTENTION : on voudrait que ce deplacement relatif
# fonctionne meme lorsque la base du robot se deplace.
# Il faut donc considerer les coordonnees relativement
# a celle-ci.
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def demarre_PTP_REL( Pgoal, tolerance, Vmax, gain):
    self.distance_parcourue = 0.0
    curPos = sim.getObjectPosition(self.hTool, self.hBase)  # hTool --> position de l'outil 
    dx = Pgoal[0] 
    dy = Pgoal[1] 
    dz = Pgoal[2]
    self.distance_a_parcourir = math.sqrt(dx**2 + dy**2 + dz**2)
    self.goal[0]  = curPos[0] + Pgoal[0]
    self.goal[1]  = curPos[1] + Pgoal[1]
    self.goal[2]  = curPos[2] + Pgoal[2]
    self.gain  = gain
    self.Vmax  = Vmax
    # mise a jour de l'etat 
    self.state = iPTP_REL
#&&&&&&&&&&&&&&&&&&&&&&&&
# demarrage d'une attente 
#&&&&&&&&&&&&&&&&&&&&&&&&
def demarre_WAIT( dt ):
    self.waitTime = dt;
    self.state = iWAIT
#-----------------------------------------------
def sysCall_actuation():
    # put your actuation code here
    # There is a simple way, and a more elaborate way (but which gives you more options/flexibility):
    
    # Simple way:
    res, *_ = simIK.handleGroup(self.ikEnv, self.ikGroup_undamped, {'syncWorlds': True})
    if res != simIK.result_success:
        simIK.handleGroup(self.ikEnv, self.ikGroup_damped, {'syncWorlds': True})
        sim.addLog(sim.verbosity_scriptwarnings, "ECHEC du solver de cinematique inverse.")
    # gestion des transitions entre etats
    #*********************
    # ATTENTE (IDLE) 
    #*********************
    if self.state == iIDLE:
        sim.addLog(1,"etat = IDLE")
        # decodage de l'instruction : 
        if self.iPC < (len(self.lInstructions)-1):
            self.iPC += 1
            iINSTR  = self.lInstructions[self.iPC]
            lParams = self.lParameters[self.iPC]
            #++++++++
            # PTP_REL
            #++++++++
            if iINSTR == iPTP_REL:
                Pgoal     = [lParams[0], lParams[1], lParams[2]]
                tolerance = lParams[3]
                vmax      = lParams[4]
                gain      = lParams[5]
                demarre_PTP_REL(Pgoal, tolerance, vmax, gain)
            #+++++++
            # GRASP
            #+++++++
            if iINSTR == iGRASP:
                if not self.hObject == -1:
                   # on supprime l'aspect dynamique de l'objet a saisir
                   sim.setBoolProperty(self.hObject,"dynamic", False)
                   #self.hObject.dynamic = False
                   # on le rend enfant de l'outil
                   sim.setObjectParent(self.hObject, self.hTool, True)
                   self.hInHand = self.hObject
                   sim.addLog(1,"Objet CAPTURE")
                else:
                    sim.addLog(1,"RIEN A CAPTURE")
                self.state = iIDLE
            #+++++
            # DROP
            #+++++
            if iINSTR == iDROP:
                if not self.hInHand == -1:
                    # on rend l'objet a l'environnement 
                    sim.setObjectParent(self.hInHand, 0, True)
                    # on rend a l'objet sa propriete dynamique
                    sim.setBoolProperty(self.hInHand, "dynamic", True)
                    sim.resetDynamicObject(self.hInHand)
                    self.hInHand = -1
                    sim.addLog(1,"Objet LACHE")
                    self.state = iIDLE
                else:
                    sim.addLog(1,"RIEN A LACHER")
                self.state = iIDLE
            #+++++
            # WAIT 
            #+++++
            if iINSTR == iWAIT:
                sim.addLog(1,"WAIT")
                dt = lParams
                demarre_WAIT( dt )
        else:
            sim.stopSimulation()
    #*************************
    # DEPLACEMENT ABSOLU (PTP)
    #*************************
    if self.state == iPTP:
        #   (1) lecture de la position courante de l'outil (self.hTool)
        curPos = sim.getObjectPosition( self.hTool )
        #   (2) calcul de la distance a la position cible (self.goal)
        dx = curPos[0] - self.goal[0]
        dy = curPos[1] - self.goal[1]
        dz = curPos[2] - self.goal[2]
        d  = math.sqrt( dx**2 + dy**2 + dz**2)
        ecart = d 
        sim.addLog(1,"etat = PTP . ecart = "  + str(ecart) )
        #       (2.1) declencher l'arret si < a la tolerance
        if math.fabs(ecart) < self.tolerance:
            # on arrete
            sim.setObjectPosition(self.hTarget, curPos )
            self.state = iIDLE
        else:
            #   sinon :
            #   (3) calcul du vecteur unitaire de deplacement (u)
            u = [-dx/d , -dy/d, -dz/d]
            #   (4) calcul de la vitesse a appliquer avec la loi de commande proportionnelle (v)
            v = self.gain*ecart
            if math.fabs(v) > self.Vmax:
                if v > 0.0:
                    v = self.Vmax
                else:
                    v = -self.Vmax
            #   (5) calcul de l'increment de deplacement a appliquer au hTarget (dp)
            dt = sim.getSimulationTimeStep()
            dp = v*dt
            Dx = curPos[0] + u[0] * dp
            Dy = curPos[1] + u[1] * dp
            Dz = curPos[2] + u[2] * dp
            #   (6) application a la position de self.hTarget
            sim.setObjectPosition( self.hTarget, [Dx,Dy,Dz] )
    #******************************
    # DEPLACEMENT RELATIF (PTP_REL)
    #******************************
    if self.state == iPTP_REL:
        #   (1) lecture de la position courante de l'outil (self.hTool)
        curPos = sim.getObjectPosition( self.hTool, self.hBase )
        #   (2) calcul de la distance a la position cible (self.goal)
        dx = curPos[0] - self.goal[0]
        dy = curPos[1] - self.goal[1]
        dz = curPos[2] - self.goal[2]
        d  = math.sqrt( dx**2 + dy**2 + dz**2)
        ecart = d 
        sim.addLog(1,"etat = PTP_REL . ecart = "  + str(ecart) )
        #       (2.1) declencher l'arret si < a la tolerance
        if math.fabs(ecart) < self.tolerance:
            # on arrete
            sim.setObjectPosition(self.hTarget, curPos, self.hBase )
            self.state = iIDLE
        else:
            #   sinon :
            #   (3) calcul du vecteur unitaire de deplacement (u)
            u = [-dx/d , -dy/d, -dz/d]
            #   (4) calcul de la vitesse a appliquer avec la loi de commande proportionnelle (v)
            v = self.gain*ecart
            if math.fabs(v) > self.Vmax:
                if v > 0.0:
                    v = self.Vmax
                else:
                    v = -self.Vmax
            #   (5) calcul de l'increment de deplacement a appliquer au hTarget (dp)
            dt = sim.getSimulationTimeStep()
            dp = v*dt
            Dx = curPos[0] + u[0] * dp
            Dy = curPos[1] + u[1] * dp
            Dz = curPos[2] + u[2] * dp
            #   (6) application a la position de self.hTarget
            sim.setObjectPosition( self.hTarget, [Dx,Dy,Dz], self.hBase )
    #********
    # ATTENTE 
    #********
    if self.state == iWAIT:
        if self.waitTime > 0:
            self.waitTime -= sim.getSimulationTimeStep()
        else:
            self.waitTime = 0
            self.state = iIDLE
#_______________________________________________________________________
def sysCall_sensing():
    # put your sensing code here
    # detection d'un eventuel objet proximite
    #res, d, points, hObj, _  = sim.readProximitySensor(self.hContact )
    res, d, points, hObj, _  = sim.handleProximitySensor(self.hContact )
    sim.addLog(1, "contact : resultat = " + str( res ) )
    if res == 1:
        sim.addLog(1,"handle de l objet detecte = " + str(hObj) + " distance = " + str(d))
        self.hObject = hObj
    else:
        self.hObject = -1
def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details

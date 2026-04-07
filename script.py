# importation de OpenCV (traitement d'image)
import cv2
import numpy as np
import math
# constantes codant les etats geres du robot
iIDLE       = 0
iGO         = 1         # avancee en ligne droite
iTURN       = 2         # rotation sur place
iSEARCH     = 3         # recherche du cylindre
iTRACK      = 4         # tracking du cylindre
iARRIVED    = 5         # arrive a proximite du cylindre
# vitesse minimale 
Vmin  = 0.001
# geometrie du robot (faire une version auto-adaptative)
E = 0.3
R = 0.05
#==============================================================================
def sysCall_init():
    sim = require('sim')
    # on recupere les HANDLES sur les moteurs : 
    self.moteur_gauche = sim.getObject('../moteur_gauche')
    self.moteur_droit  = sim.getObject('../moteur_droit')
    # handle sur la base pour localiser le robot
    self.base          = sim.getObject('../../base')
    if self.moteur_droit < 0:
        sim.addLog(1,"ERREUR : probleme HANDLE moteur DROIT")
    else:
        sim.addLog(1,"HANDLE moteur droit = " + str(self.moteur_droit))
    if self.moteur_gauche < 0:
        sim.addLog(1,"ERREUR : probleme HANDLE moteur GAUCHE")
    else:
        sim.addLog(1,"HANDLE moteur gauche = " + str(self.moteur_gauche))
    # recuperation du handle de la camera
    self.hCam = sim.getObject('../camera')
    if self.moteur_gauche < 0:
        sim.addLog(1,"ERREUR : probleme HANDLE CAMERA")
    else:
        sim.addLog(1,"HANDLE CAMERA = " + str(self.hCam))
    # information de la localistion du cylindre
    self.CylDetected = False
    self.Cyl_xc      = -1
    self.Cyl_yc      = -1
    self.Cyl_npix    = -1
    # remise a zero de la vitesse des moteurs 
    sim.setJointTargetVelocity( self.moteur_gauche, 0.0)
    sim.setJointTargetVelocity( self.moteur_droit, 0.0)
    # reservation d'un "dummy" pour avoir une reference de 
    # depart lors des primitives de deplacement
    self.hDum  = sim.createDummy( 0.05 )
    sim.setObjectPosition( self.hDum, sim.getObjectPosition(self.base))
    sim.setObjectOrientation( self.hDum, sim.getObjectOrientation(self.base))
    self.lInstructions = [iSEARCH,iTRACK]     # liste des codes instructions
    self.lOperandes    = [-15.0,0.0]      # liste des operandes associees aux instructions
    self.iPC           = -1              # compteur de programme (-1 ==> pas demarre )
    # etat initial du robot
    self.etat = iIDLE # au depart le robot ne fait rien
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# "wrapper" pour convertir l'image V-REP en quelque chose de 
#  manipulable par OpenCV
# IN : 
#   imgRes  : resolution de l'image [lignes, colonnes]
#   imgData : donnees image (en ligne)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def VREP2CV2ImageWrapper( imgRes, imgData ):
    imgShape = [imgRes[0],imgRes[1],3]
    cvImg = np.zeros(imgShape, dtype = np.uint8 )
    index = 0
    for i in range(imgRes[0]):
        for j in range(imgRes[1]):
            # V-REP travaille en RGB, et OpenCV en BGR...
            # il faut de plus inverser l'ordre des lignes
            cvImg[imgRes[0] -1 - i,j,2] = imgData[index]
            cvImg[imgRes[0] -1 - i,j,1] = imgData[index+1]
            cvImg[imgRes[0] -1 - i,j,0] = imgData[index+2]
            index = index + 3
    return cvImg
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction permettant de lancer le deplacement vers l'avant (GO)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def commencer_Go( distance, tolerance):
    self.distance2Go = distance  # distance a parcourir
    self.tolerance    = tolerance # tolerance sur la fin de deplacement
    sim.setObjectPosition(self.hDum, sim.getObjectPosition(self.base))
    sim.setObjectOrientation(self.hDum, sim.getObjectOrientation(self.base))
    self.etat = iGO
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# donne la distance parcourue depuis le debut de Go
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def DistanceParcourue():
    if not(self.hDum == None):
        pos =  sim.getObjectPosition(self.base, self.hDum)
        d = math.sqrt( pos[0]**2 + pos[1]**2 )
    else:
        d = -1.0
    return d 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction permettant de lancer la rotation sur place (TURN)
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def commencer_turn( angle, tolerance_angle):
    self.angle2Turn = angle
    self.tolerance_angle = tolerance_angle
    sim.setObjectPosition(self.hDum, sim.getObjectPosition(self.base))
    sim.setObjectOrientation(self.hDum, sim.getObjectOrientation(self.base))
    self.etat = iTURN
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# donne l'angle parcourue depuis le debut de la rotation
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def AngleParcouru():
    if not(self.hDum == None):
        ori =  sim.getObjectOrientation(self.base, self.hDum)
        a = ori[2]  # angle d'Euler : rotation autour de z
        a = (a/3.141592654)*180.0
        sim.addLog(1,"angle parcouru = " + str(a) )
    else:
        a = None
    return a
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction permettant de mettre a jour les variables de 
# detection du cyclindre depuis un script externe
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def setCylinderDetectionInfo(detected, xc, yc, npix ):
    self.CylDetected = detected
    self.Cyl_xc      = xc
    self.Cyl_yc      = yc
    self.Cyl_npix    = npix
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction permettant de lancer la recherche
# IN  : vitesse : vitesse de rotation souhaitee pour la recherche
#       E       : entraxe
#       R       : rayon des roues
#       MinPix  : nombre minimal de pixel a trouver
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def commencer_search( vitesse, E, R, MinPix ):
    Wc = (3.141592654/180.0)* vitesse
    Vc = 0
    wG, wD = InverseKinematics(Vc, Wc, E, R )
    # ceci entame la rotation du robot sur place
    sim.setJointTargetVelocity( self.moteur_droit, wD)
    sim.setJointTargetVelocity( self.moteur_gauche, wG)
    self.etat = iSEARCH
def AfficherPosition():
    pos = sim.getObjectPosition(self.base)
    szMsg = "x = " + str(pos[0]) + " y = " + str(pos[1]) + " z = " + str(pos[2])
    sim.addLog(1, szMsg)
def AfficherDistance():
    d = DistanceParcourue()
    szMsg = "distance parcourue = " + str(d) + " m"
    sim.addLog(1,szMsg)
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# dans sysCallAction on gere les deplacements et les transitions entre etats 
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def sysCall_actuation():
    if self.etat == iIDLE:
        sim.addLog(1,"[BASE]----->etat = IDLE")
        # on incremente le compteur de programme 
        if self.iPC < len(self.lInstructions) - 1:
            self.iPC += 1
            iINSTR  = self.lInstructions[self.iPC]
            lParams = self.lOperandes[self.iPC] 
            # instruction a realiser : 
            if iINSTR == iGO:
                sim.addLog(1, "instruction a realiser  = GO")
                commencer_go( lParams, 0.001 )
            if iINSTR == iTURN:
                sim.addLog(1, "instruction a realiser = TURN")
                commencer_turn( lParams, 0.01 )
            if iINSTR == iSEARCH:
                sim.addLog(1, "instruction a realiser = SEARCH")
                self.etat = iSEARCH
                commencer_search(lParams, E, R, 20) 
        else:
            sim.stopSimulation()
    #.......................
    # gestion de la rotation
    #.......................
    if self.etat == iTURN:
        sim.addLog(1," [BASE]----->etat = TURN")
        a = AngleParcouru()
        ecart = self.angle2Turn - a 
        if math.fabs(ecart) < self.tolerance_angle:
            # on arrete 
            sim.setJointTargetVelocity( self.moteur_droit, 0.0)
            sim.setJointTargetVelocity( self.moteur_gauche, 0.0)
            self.etat = iIDLE # CORRIGER !!!
        else:
            Wc = (3.141592654/180.0)*0.1 * ecart
            Wa = math.fabs(Wc)
            # seuil 
            if Wa > 10.0:
                Wa = 10.0
            # calcul des vitesses angulaires des roues
            w = 10.0 * Wa
            if Wc > 0.0:
                sim.setJointTargetVelocity( self.moteur_droit, -w)
                sim.setJointTargetVelocity( self.moteur_gauche, w)
            else:
                sim.setJointTargetVelocity( self.moteur_droit, w)
                sim.setJointTargetVelocity( self.moteur_gauche, -w)
    #..........................
    # gestion de la translation
    #..........................
    if self.etat == iGO:
        sim.addLog(1,"[BASE]------>etat = GO")
        d  = DistanceParcourue()
        ecart = self.distance2Go - d
        if math.fabs(ecart) < self.tolerance:
            # on arrete 
            sim.setJointTargetVelocity( self.moteur_droit, 0.0)
            sim.setJointTargetVelocity( self.moteur_gauche, 0.0)
            self.etat = iIDLE
        else:
            Vc = 0.1 * ecart
            # seuil 
            if Vc > 1.0:
                Vc = 1.0
            # calcul des vitesses angulaires des roues
            w = 20.0 * Vc
            sim.setJointTargetVelocity( self.moteur_droit, w)
            sim.setJointTargetVelocity( self.moteur_gauche, w)
    #........................
    # gestion de la recherche
    #........................
    if self.etat == iSEARCH:
        sim.addLog(1,"[BASE]------>etat = SEARCH")
        if self.CylDetected == True:
            # on arrete la rotation sur place
            sim.setJointTargetVelocity( self.moteur_droit, 0.0)
            sim.setJointTargetVelocity( self.moteur_gauche, 0.0)
            sim.addLog(1,"CYLINDRE DETECTE")
            self.etat = iIDLE
#_________________________________________________________________________________
def sysCall_sensing():
    sim.addLog(1, "xc = " + str(self.Cyl_xc) + " yc = " + str(self.Cyl_yc) + " npix = " + str(self.Cyl_npix))
    # calcul de la loi de commande
    pass
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# cinematique inverse
# IN  : Vc :  vitesse lineaire
#       Wc :  vitesse angulaire
#       E  :  entraxe
#       R  :  rayon des roues
# OUT : wG :  vitesse angulaire roue gauche
#       wD :  vitesse angulaire roue droite
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def InverseKinematics( Vc, Wc, E, R):
    wG = Vc - E*Wc/(2.0*R)
    wD = Vc + E*Wc/(2.0*R)
    return wG, wD
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# loi de commande permettant de rejoindre le cylindre 
# IN : Dxc  : difference entre l'abscisse du centre de gravite et le centre 
#             de l'image
#      nPix : nombre de pixels detectes pour le cylindre 
#      G1   : gain pour le controle en vitesse lineaire
#      G2   : gain pour le controle en vitesse angulaire
# OUT: Vc   : vitesse lineaire
#      Wc   : vitesse angulaire
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def control( Dxc, nPix, G1, G2):
    Vc = G1/nPix
    if Vc < Vmin:
        Vc = 0.0
    Wc = G2*Dxc
    return Vc, Wc
#____________________________________________________________________________
def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details

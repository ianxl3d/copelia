import cv2
import numpy as np
#-----------------------------------------
def sysCall_init():
    sim = require('sim')
    self.hCam = sim.getObject('../../camera')
    if self.hCam < 0:
        sim.addLog(1,"vision_processor() ---> ERREUR sur l obtention du HANDLE de la CAMERA")
    else:
        sim.addLog(1,"vision_processor() ---> HANDLE de la CAMERA = " + str(self.hCam))
    self.hMainScript = sim.getObject('../../script')
    if self.hMainScript < 0:
        sim.addLog(1,"vision_processor() ---> ERREUR sur l obtention du HANDLE du SCRIPT PRINCIPAL")
    else:
        sim.addLog(1,"vision_processor() ---> HANDLE du SCRIPT principal = " + str(self.hMainScript))
#------------------------------------------
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
#********************************************************
# fonction de localisation du cylindre dans l'image
# IN : img (image couleur acquise par la camera)
#          (np.array)
#      minpix : nombre minimal de pixels "rouge" pour que 
#               la detection soit consideree comme valide
# OUT : xc    : abscisse du centre de gravite du cylindre 
#       yc    : ordonnee du centre de gravite du cylindre
#       npix  : nombre de pixels trouves
#********************************************************
def localise_cylindre( img, minpix ):
    # (1) on filtre les pixels rouge
    # (2) on calcul le centre de gravite des pixels rouge
    rMin = 128
    gMax = 30
    bMax = 30
    xc   = 0
    yc   = 0
    npix = 0
    r = img[:,:,2]
    g = img[:,:,1]
    b = img[:,:,0]
    rf = np.where(r > rMin, True, False)
    gf = np.where(g < gMax, True, False)
    bf = np.where(b < bMax, True, False)
    work2 = np.logical_and(rf, gf)
    work3 = np.logical_and(work2, bf)
    work4 = 255 * work3.astype(int)
    work = work4.astype(np.uint8)
    locations = np.where(work4 == 255)
    ly = locations[0]
    lx = locations[1]
    yc = np.mean(ly)
    xc = np.mean(lx)
    npix = len(lx.tolist())
    if npix > minpix:
        xc  = xc / npix
        yc  = yc / npix
    else:
        xc = -1         # indique pas de detection 
        yc = -1         # ...
        npix = -1       # ...
    
    """
    [lignes, colonnes, plans] = img.shape
    work = np.zeros([lignes, colonnes], dtype=np.uint8)
    lCylPix = []
    for i in range(lignes):
        for j in range(colonnes):
            r = img[i,j,2]
            g = img[i,j,1]
            b = img[i,j,0]
            if (r >= rMin) and (g < gMax) and (b < bMax):
                lCylPix.append([i,j])
                work[i,j] = 255
                xc   += j
                yc   += i
                npix += 1
    if npix > minpix:
        xc  = xc / npix
        yc  = yc / npix
    else:
        xc = -1         # indique pas de detection 
        yc = -1         # ...
        npix = -1       # ...
    """
    cv2.imshow("FILTRAGE", work )
    return xc, yc, npix
#--------------------------------------------
def sysCall_thread():
    # Put your main code here, e.g.:
    while not sim.getSimulationStopping():
        # acquisition de l'image de la camera 
        RawImage, resolution = sim.getVisionSensorImg(self.hCam)
        imageData = sim.unpackUInt8Table(RawImage)
        # conversion en un format manipulable par OpenCV : 
        img = VREP2CV2ImageWrapper( resolution, imageData )
        # affichage pour verification : 
        cv2.imshow("CAMERA", img )
        # exemple de traitement
        r = img[:,:,2]
        g = img[:,:,1]
        b = img[:,:,0]
        cv2.imshow("ROUGE-2", r )
        cv2.imshow("VERT-2", g )
        cv2.imshow("BLEU-2", b )
        cv2.waitKey(2)
        # localisation du cylindre
        xc2, yc2, npix2 = localise_cylindre(img, 50)
        if npix2 >= 50:
            Cyl_detected = True
        else:
            Cyl_detected = False
        sim.callScriptFunction('setCylinderDetectionInfo', self.hMainScript, Cyl_detected, xc2, yc2, npix2 )
        #xc2, yc2, npix2 = sim.callScriptFunction('localise_cylindre', self.hMainScript, img, 50)
        """
        if npix > 0:
            self.CylDetected = True
            self.Cyl_xc      = xc
            self.Cyl_yc      = yc
            self.Cyl_npix    = npix
        else:
            self.CylDetected = False
            self.Cyl_xc      = -1
            self.Cyl_yc      = -1
            self.Cyl_npix    = -1
        """
        sim.step()
        #     p = sim.getObjectPosition(objHandle, -1)
        #     p[0] += 0.001
        #     sim.setObjectPosition(objHandle, -1, p)
        #     sim.step() # resume in next simulation step
    pass

# See the user manual or the available code snippets for additional callback functions and details

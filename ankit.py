###################################################################################
### Functions for general use
### Ankit Kaushik
### Wheego Technologies
### Version 1: June 18, 2017
##################################################################################

import os, time, sys
import h5py as h5
import cv2
import numpy as np
import utm
import scipy
from math import *
from Quaternion import Quat, normalize
# from easydict import EasyDict as edict

def getFrameNumber(fileName):
    return int((fileName.split('/')[-1]).split('_', 1)[0])

def getFrameDirectory(fileName):
    return fileName.split('/')[-4]

def getIndex(name):
    return name[0]
    
def getIndexedFrameNumber(name):
    return int((name[1].split('/')[-1]).split('_', 1)[0])

def getCacheInfo(name):
    cacheIndex = int(name.split('streetData_')[1].split('_created')[0].split('_')[0])
    cacheCreatedBy = name.split('streetData_')[1].split('_created')[0].split('_')[1]
    return cacheIndex, cacheCreatedBy 

def getOneDirUp():
    return os.getcwd()[:-len(os.getcwd().split('/')[-1])]

def walkH5(name):
    print name

def switchFileNameHeader(fileNames,switchType):
    if switchType == 1:
        return ['/home/ankit/QNAP'+fn.split('QNAP')[1] for fn in fileNames]
    if switchType == 2:
        return ['/home/wheego/QNAP'+fn.split('QNAP')[1] for fn in fileNames]

def getDictionaryRange(dictionary, begin, end):
    print 'This will not give you the same results with every call unless it is an OrderedDict'
    rangeDictionary = {}
    for (k,v) in dictionary.iteritems():
        rangeDictionary[k] = v[begin:end]
    return rangeDictionary
    
def computeSteering(steeringData):
    
    if steeringData.shape[0] >= 1: 
        hasSteering = True

        if steeringData.shape[0] == 1: 
            steeringAngle = steeringData[0][0]
            steeringCurrent = steeringData[0][1]
            steeringTime = steeringData[0][2]

        else: 
            steeringAngle = np.mean(steeringData[:,0])
            steeringCurrent = np.mean(steeringData[:,1])
            steeringTime = np.mean(steeringData[:,2])
    
    else:
        hasSteering = False
        steeringAngle = np.NaN
        steeringCurrent = np.NaN
        steeringTime = np.NaN
            
    return hasSteering, steeringAngle, steeringCurrent, steeringTime

def computeSpeed(wheegoData):
        
    if wheegoData.shape[0] >= 1: 
        hasSpeed = True
        if wheegoData.shape[0] == 1: 
            speed = wheegoData[0][0]

        else: 
            speed = np.mean(wheegoData[:,0])

    else:
        hasSpeed = False
        speed = np.NaN
            
    return hasSpeed, speed
        
def computeIMUMultiple(imuData):

    utmLoc = np.zeros((len(imuData['x134_Position']), 2))
    
    for i in range(len(imuData['x134_Position'])):
        u = utm.from_latlon(imuData['x134_Position'][i, 0], imuData['x134_Position'][i, 1])
        utmLoc[i, 0] = u[0]
        utmLoc[i, 1] = u[1]
    
    qNorm = normalize(imuData['x131_Quaternion'][0, :])    
    eulerAngles = np.zeros((len(imuData['x131_Quaternion']), 3))
    
    for i in range(len(imuData['x131_Quaternion'])):
        qNorm = normalize(imuData['x131_Quaternion'][i, :])
        Q = Quat(qNorm)
        eulerAngles[i, :] = [Q.ra, Q.dec, Q.roll]
    
    relPath = utmLoc-utmLoc[0, :]    
    rotAngle = (180-eulerAngles[0 ,2])*np.pi/180.    
    R2 = np.array([[cos(rotAngle), -1*sin(rotAngle)],[sin(rotAngle), cos(rotAngle)]])    
    pathR = np.dot(R2, relPath.T)

    # else:
    #     utmLoc = np.full((5,2),np.NaN)
    #     eulerAngles = np.full((5,3),np.NaN)
    #     pathR = np.full((2,5),np.NaN)
    #     velocity = np.full((5,3),np.NaN)
    #     time = np.full((5,8),np.NaN)
        
    return utmLoc, eulerAngles, pathR

def computeIMU(imuData):

    utmLoc = np.zeros(2)  
    u = utm.from_latlon(imuData['x134_Position'][0], imuData['x134_Position'][1])
    utmLoc[0] = u[0]
    utmLoc[1] = u[1]       
    eulerAngles = np.zeros(3)    
    qNorm = normalize(imuData['x131_Quaternion'][:])
    Q = Quat(qNorm)
    eulerAngles[:] = [Q.ra, Q.dec, Q.roll]         
    return utmLoc, eulerAngles

def rotateIMU(imuPath, rotAngle):

    R2 = np.array([[cos(rotAngle), -1*sin(rotAngle)], \
                   [sin(rotAngle), cos(rotAngle)]])    
    return np.dot(R2, imuPath)

def generateIMUSteeringSpeedData(fileNamesGlobal):

    numFiles = len(fileNamesGlobal)
    fileNames = []
    steeringAngles = np.zeros(numFiles)
    steeringCurrents = np.zeros(numFiles)
    steeringTimes = np.zeros(numFiles)
    speeds = np.zeros(numFiles)
    imuVelocities = np.zeros((numFiles,3))
    imuTimes = np.zeros((numFiles,8))
    imuUtmLocs = np.zeros((numFiles,2))
    imuEulerAngles = np.zeros((numFiles,3))
    imuKeys = ['x134_Position', 'x131_Quaternion', 'x137_Velocity', 'x111_Time']
    frameNum = 0
    exceptions = []

    startTime = time.time()

    for i,fileName in enumerate(fileNamesGlobal):
        print i
        try:
            with h5.File(fileName, 'r') as data:
                hasIMU = set(imuKeys).issubset(set(data['IMU'].keys()))
                if hasIMU:
                    hasSteering, steeringAngle, steeringCurrent, steeringTime = computeSteering(data['STEERING/Steering_Data'])
                    if hasSteering:
                        steeringAngles[frameNum] = steeringAngle
                        steeringCurrents[frameNum] = steeringCurrent
                        steeringTimes[frameNum] = steeringTime
                        hasSpeed, speed = computeSpeed(data['WHEEGO/Wheego_Data'])
                        speeds[frameNum] = speed
                        imuVelocities[frameNum] = np.mean(data['IMU/x137_Velocity'][:], axis=0)
                        imuTimes[frameNum] = np.mean(data['IMU/x111_Time'][:], axis=0)
                        imuData = {}
                        imuData['x134_Position'] = np.mean(data['IMU/x134_Position'][:], axis=0)
                        imuData['x131_Quaternion'] = np.mean(data['IMU/x131_Quaternion'][:], axis=0)                
                        utmLoc, eulerAngles = computeIMU(imuData)
                        imuUtmLocs[frameNum] = utmLoc
                        imuEulerAngles[frameNum] = eulerAngles
                        fileNames.append(fileName)
                        frameNum = frameNum+1
                    else:
                        print 'no Steering!'
                else:
                    print 'no IMU!'
        except:
            the_type, the_value, the_traceback = sys.exc_info()
            exceptionInfo = the_type, the_value
            exceptions.append([i,fileName,exceptionInfo])

    print str(frameNum)+' frames completed in ' + str(time.time()-startTime) + ' s' 
    print 'These were the exceptions'
    print exceptions
    return fileNames, steeringAngles[:frameNum], steeringCurrents[:frameNum], steeringTimes[:frameNum], speeds[:frameNum], imuVelocities[:frameNum,:], imuTimes[:frameNum,:], imuUtmLocs[:frameNum,:], imuEulerAngles[:frameNum,:], exceptions

def generateVOData(fileNamesVO):

    voFileNamesAll = []
    frameNum = 0
    exceptions = []

    startTime = time.time()

    for i,fileName in enumerate(fileNamesVO):
        print i
        try:
            if i == 0:
                with h5.File(fileName, 'r') as data:
                    voData = data['Poses'][:]
                    voFileNames = data['Keys'][:]
                    voFileNames = [fileName.split('vo_trajectory_CENTER_CAMERA.h5')[0]+'processed/H5/'+fn for fn in voFileNames]
                    voFileNamesAll.extend(voFileNames)
                    frameNum = frameNum+1
            else:
                with h5.File(fileName, 'r') as data:
                    voData = np.vstack((voData, data['Poses'][:]))
                    voFileNames = data['Keys'][:]
                    voFileNames = [fileName.split('vo_trajectory_CENTER_CAMERA.h5')[0]+'processed/H5/'+fn for fn in voFileNames]
                    voFileNamesAll.extend(voFileNames)
                    frameNum = frameNum+1 
        except:
            the_type, the_value, the_traceback = sys.exc_info()
            exceptionInfo = the_type, the_value
            exceptions.append([i,fileName,exceptionInfo])

    print 'EXCEPTIONS'
    print exceptions
    return voData, voFileNamesAll

# def generateImageIMUSteeringSpeedData(fileNamesGlobal):

#     numFiles = len(fileNamesGlobal)
#     with h5.File(fileNamesGlobal[0], 'r') as data:
#         imageShape = data['CAMERA_1/Cam_1_Data'].shape
#     images = np.zeros((numFiles,imageShape[0],imageShape[1],imageShape[2]))
#     fileNames = []
#     steeringAngles = np.zeros(numFiles)
#     steeringCurrents = np.zeros(numFiles)
#     steeringTimes = np.zeros(numFiles)
#     speeds = np.zeros(numFiles)
#     imuVelocities = np.zeros((numFiles,3))
#     imuTimes = np.zeros((numFiles,8))
#     imuUtmLocs = np.zeros((numFiles,2))
#     imuEulerAngles = np.zeros((numFiles,3))
#     imuKeys = ['x134_Position', 'x131_Quaternion', 'x137_Velocity', 'x111_Time']
#     frameNum = 0
#     exceptions = []

#     for i,fileName in enumerate(fileNamesGlobal):
#         print i
#         try:
#             with h5.File(fileName, 'r') as data:
#                 hasIMU = set(imuKeys).issubset(set(data['IMU'].keys()))
#                 if hasIMU:
#                     hasSteering, steeringAngle, steeringCurrent, steeringTime = computeSteering(data['STEERING/Steering_Data'])
#                     if hasSteering:
#                         images[frameNum,:,:,:] = data['CAMERA_1/Cam_1_Data']                
#                         fileNames.append(fileName)
#                         steeringAngles[frameNum] = steeringAngle
#                         steeringCurrents[frameNum] = steeringCurrent
#                         steeringTimes[frameNum] = steeringTime
#                         hasSpeed, speed = computeSpeed(data['WHEEGO/Wheego_Data'])
#                         speeds[frameNum] = speed
#                         imuVelocities[frameNum] = np.mean(data['IMU/x137_Velocity'][:], axis=0)
#                         imuTimes[frameNum] = np.mean(data['IMU/x111_Time'][:], axis=0)
#                         imuData = {}
#                         imuData['x134_Position'] = np.mean(data['IMU/x134_Position'][:], axis=0)
#                         imuData['x131_Quaternion'] = np.mean(data['IMU/x131_Quaternion'][:], axis=0)                
#                         utmLoc, eulerAngles = computeIMU(imuData)
#                         imuUtmLocs[frameNum] = utmLoc
#                         imuEulerAngles[frameNum] = eulerAngles
#                         frameNum = frameNum+1
#         except:
#             the_type, the_value, the_traceback = sys.exc_info()
#             exceptionInfo = the_type, the_value
#             exceptions.append([i,fileName,exceptionInfo])

#     return images, fileNames, steeringAngles, steeringCurrents, steeringTimes, speeds, imuVelocities, imuTimes, imuUtmLocs, imuEulerAngles, exceptions
#     print str(frameNum)+' frames completed' 

def getSyntheticPath(utmLoc,thetas,numSteps,lookAheadIndex,initialShift):

    shift = initialShift
    deltaShift = shift/numSteps
    numRefPoints = numSteps+lookAheadIndex
    xPath = np.zeros(numRefPoints)
    yPath = np.zeros(numRefPoints)

    for i in range(numSteps):
        [x1,y1] = utmLoc[i]
        angle = (thetas[i]-180)*np.pi/180       
       	xPath[i] = x1+shift*cos(angle)
        yPath[i] = y1+shift*sin(angle)
        shift = shift-deltaShift

    xPath[numSteps:numRefPoints] = utmLoc[numSteps:numRefPoints][:,0]
    yPath[numSteps:numRefPoints] = utmLoc[numSteps:numRefPoints][:,1]

    return xPath, yPath

def makeStagingParams():

    stagingParams = edict()

    stagingParams.cropA = [948, [430, 760, 164, 1164]]
    stagingParams.cropB = [1048, [530, 860, 164, 1164]]
    stagingParams.steeringOffsetA = -613.392
    stagingParams.steeringOffsetB = -60.0
    stagingParams.centerLeftDistance = -1*22.1*25.4 #mm
    stagingParams.centerRightDistance = 22.25*25.4
    stagingParams.imageResizeSize = [200,66]

    #Standard deviations of gaussian distributions used for creating synthetic data
    stagingParams.wheelBase = 2.0
    stagingParams.steeringRatio = 20.0
    stagingParams.offsetSTD = 1.0 #m
    stagingParams.thetaSTD = 1.0 #Degrees
    
    #Thresholds and ratios used for data rebalancing:
    stagingParams.thresholds = [0, 50, 120]
    stagingParams.ratios = [1, 2, 9]
    
    stagingParams.numFutureFrames = 45
    stagingParams.numFutureImages = 5
    
    ### --------------- Batch Sizes and Number of Batches --------------------------- ###
    stagingParams.chunkSize = 4096
    stagingParams.chunkSizeSynthetic = 1024 
    
    stagingParams.numTrainingBatches = 5
    stagingParams.numTrainingBatchesSynthetic = 100    

    return stagingParams

def computeSteeringAngle(xref, yref, pr, theta, velocity, wheelBase, steeringRatio, i):

    trackPoint = xref[i+9], yref[i+9]
    xDistance = trackPoint[0]-pr[0]
    yDistance = trackPoint[1]-pr[1]
    L = np.sqrt(xDistance**2 + yDistance**2)
    alpha1 = atan2(yDistance,xDistance)

    if alpha1<0:
        alpha2 = alpha1+(2*pi) - theta
    else:
        alpha2 = alpha1 - theta

    omega = 2*velocity*sin(alpha2)/L
    delta = atan(omega*wheelBase/velocity)
    delta = delta*steeringRatio*180./np.pi
    delta = -delta
    return delta

class kinematicModel(object):
    def __init__(self, thetaInitial, deltaInitial, positionInitial, wheelBase, steeringRatio):
        
        self.steeringRatio = steeringRatio
        self.wheelBase = wheelBase
        
        thetaDegrees = thetaInitial-90
        theta0= thetaDegrees*np.pi/180.

        #Position Center of Rear Axle Initial
        pr0 = np.array([positionInitial[0], positionInitial[1]])

        #Position Center of Front Axle Initial
        pf0 = np.array([pr0[0] + self.wheelBase*cos(theta0), pr0[1] + self.wheelBase*sin(theta0)])

        self.pf0 = pf0.copy()
        self.pr0 = pr0.copy()
        self.theta0 = theta0
    
    def stepForward(self, deltaT, delta, pr, theta, v):
        #Where do we end up after steering at an angle of delta for deltaT seconds?
        
        deltaDegrees = -1*(delta)/self.steeringRatio
        delta = deltaDegrees*np.pi/180.
        
        xPrime = v*cos(theta)
        yPrime = v*sin(theta)
        thetaPrime = (v/self.wheelBase)*tan(delta)

        #Compute new pr, pf, and theta!
        pr = pr + deltaT*np.array([xPrime, yPrime])

        #Compute new pf before updating theta! Car hasn't turned yet.
        pf = pr + np.array([self.wheelBase*cos(theta), self.wheelBase*sin(theta)])

        theta = theta + deltaT*thetaPrime

        return pr, pf, theta

def runKinematicModel(runTime, deltaT, deltaInitial, thetaInitial, velocities, utmLoc, thetas, initialShift, wheelBase, steeringRatio):

    numSteps = int(ceil(runTime/deltaT))
    lookAheadIndex = 10
    xref, yref = getSyntheticPath(utmLoc, thetas, numSteps, lookAheadIndex, initialShift=initialShift)
    positionInitial = [xref[0],yref[0]]
    
    KM = kinematicModel(thetaInitial = thetaInitial, deltaInitial = deltaInitial, \
                        positionInitial = positionInitial,\
                        wheelBase = wheelBase, steeringRatio = steeringRatio)

    pr = KM.pr0
    syntheticTheta = KM.theta0
    prs = np.zeros((numSteps+1, 2))
    prs[0,:] = pr
    pfs = np.zeros((numSteps+1, 2))
    pfs[0,:] = pr + np.array([KM.wheelBase*cos(syntheticTheta), KM.wheelBase*sin(syntheticTheta)])
    syntheticThetas = np.zeros(numSteps+1)    
    syntheticThetas[0] = syntheticTheta
    syntheticDeltas = np.zeros(numSteps+1)
    syntheticDeltas[0] = deltaInitial

    for i in range(numSteps):
        velocity = velocities[i]
        syntheticDelta = computeSteeringAngle(xref, yref, pr, syntheticTheta, velocity, wheelBase, steeringRatio, i)
        pr, pf, syntheticTheta = KM.stepForward(deltaT = deltaT, delta = syntheticDelta, pr = pr, theta = syntheticTheta, v = velocity)
        prs[i+1, :] = pr
        pfs[i+1, :] = pf
        syntheticThetas[i+1] = syntheticTheta
        syntheticDeltas[i+1] = syntheticDelta
    
    # print 'Final Error ', np.sqrt((utmLoc[:,0] - pr[0])**2 + (utmLoc[:,1] - pr[1])**2)

    return prs, syntheticThetas, syntheticDeltas, xref, yref
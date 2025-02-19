import sys
from math import sqrt, cos, sin, atan, atan2, pi, tan, fabs
import numpy as np
from copy import deepcopy
from Vertex import Vertex
from Obstacle import Obstacle
import time
import matplotlib.pyplot as plt
from utils import ccw
import matplotlib.pylab as pylab
params = {'legend.fontsize': 'xx-large',
 'figure.figsize': (15, 5),
 'axes.labelsize': 'xx-large',
 'axes.titlesize': 'xx-large',
 'xtick.labelsize': 'xx-large',
 'ytick.labelsize': 'xx-large'}
pylab.rcParams.update(params)

class RRT(object):

    def __init__(self,variables,plotStore):

        # Yaml variables        
        self.alpha = variables['alpha']
        self.controlledSteering = variables['controlledSteering']
        self.dt = variables['dt']
        self.goalDist = variables['goalDist']
        self.lastSteerOnly = variables['lastSteerOnly']
        self.numStepsSteering = variables['numStepsSteering']
        self.obstacleType = variables['obstacleType']
        self.plottingInterval = variables['plottingInterval']   
        self.r = variables['r']
        self.velocity = variables['velocity']
        self.vInit = Vertex(*variables['vInit'])
        self.vGoal = Vertex(*variables['vGoal'])

        # Derived variables
        self.minSearchRadius = self.dt*self.numStepsSteering*self.velocity 
        self.plotStore = plotStore
        self.searchSpace = [min(self.vInit.x, self.vInit.y), max(self.vGoal.x, self.vGoal.y)]
        self.createObstacles()
        self.sampledPoints = []
        self.vertices = [self.vInit]
        self.verticesSteered = [self.vInit]        
        print 'rrt initialized with ' + str(self.vInit.getState())

    def assignControlSpline(self, controlSpline, maxInterpolationTime=np.inf):
        self.controlSpline = controlSpline
        self.maxInterpolationTime = maxInterpolationTime
    
    def createObstacles(self):
        self.obstacles = []
        if self.obstacleType == 'single':
            self.obstacles.append(Obstacle(center=[-2, 1.25], size=[0.5, 2]))
            self.obstacles.append(Obstacle(center=[-2, -1.25], size=[0.5, 2]))
        elif self.obstacleType == 'double':
            self.obstacles.append(Obstacle(center=[-2, 0.0], size=[3.5, 1.2]))
            self.obstacles.append(Obstacle(center=[-2, 1.75], size=[3.5, 1.5]))
            self.obstacles.append(Obstacle(center=[-2, -1.75], size=[3.5, 1.5])) 

    def computeSteeringAngle(self, trackVertex, currentVertex):
        xDistance = trackVertex.x - currentVertex.x
        yDistance = trackVertex.y - currentVertex.y
        L = np.sqrt(xDistance ** 2 + yDistance ** 2)
        alpha = atan2(yDistance, xDistance)-currentVertex.theta
        omega = 2*self.velocity*sin(alpha)/L
        return omega

    def extend(self, stopCount = 1000):
        obstacleFreeVertices = False
        count = 0
        successFlag = True
        while obstacleFreeVertices == False:
            if count < stopCount:
                newVertices = None
                vRand = self.sample()
                self.sampledPoints.append(vRand)
                # if self.plotStore is not None:
                #     self.plotStore.sampledPoints.append(vRand)
                vNearest, vNearestIndex = self.getNN(vRand)
                if self.controlledSteering is False:
                    # print 'using uncontrolled steering'
                    newVertices = self.steerUncontrolled(vNearest, vNearestIndex, vRand)
                elif self.controlledSteering is True:
                    # print 'using controlled steering'
                    newVertices = self.steerControlled(vNearest, vNearestIndex, vRand)
                if newVertices is not None:
                    obstacleFreeStart = self.obstacleFree(vNearest,Vertex(*newVertices[0]))
                    obstacleFreeVertices = self.obstacleFreeVertices(newVertices)
                    if obstacleFreeStart and obstacleFreeVertices:
                        if self.lastSteerOnly is False:
                            for i in range(newVertices.shape[0]):
                                self.vertices.append(Vertex(*newVertices[i]))
                                self.verticesSteered.append(Vertex(*newVertices[i]))
                                # if self.plotStore is not None:
                                #     self.plotStore.allRRTVertices.append(Vertex(*newVertices[i]))
                        else:
                            for i in range(newVertices.shape[0]):
                                self.vertices.append(Vertex(*newVertices[i]))
                            self.verticesSteered.append(Vertex(*newVertices[-1]))

                    if self.plotStore is not None:
                        if self.plottingInterval != 'end':
                            if self.iterationCount % 1 == 0:
                                # print 'plotting!'
                                self.plotAll()
                    count += 1
                    print 'extend count is ' + str(count)
                    print self.verticesSteered[-1].getState()
            else:
                print stopCount
                successFlag = False
                break

        return successFlag

    def extractPath(self, stopCount = np.inf, stopAtGoal = True):
        successFlag = True
        self.path = []
        self.iterationCount = 0
        lastVertex = self.vertices[-1]
        if stopAtGoal == True:
            startTime = time.time()
            # while self.reachedGoal(lastVertex) == False:
            while np.any([self.reachedGoal(v) for v in self.vertices]) == False:
                if self.iterationCount > stopCount:
                    print 'RRT failed with ' + str(self.vInit.getState())
                    successFlag = False
                    return successFlag
                if self.extend():
                    lastVertex = self.vertices[-1]
                    print 'RRT iteration count is: ' + str(self.iterationCount)
                    self.iterationCount += 1
                else:
                    successFlag = False
                    return successFlag

            self.plotStore.RRTcompletionIterations.append(self.iterationCount)
            self.plotStore.RRTcompletionTimes.append(time.time() - startTime)
            print 'path found in ' + str(self.iterationCount) + ' iterations'
            print 'path found in ' + str(time.time() - startTime) + ' s'
            print 'last added vertex is ' + str(lastVertex.getState())
            self.path.append(lastVertex)
            j = -1
            while self.vertices[j].parent is not 0:
                print self.vertices[int(self.vertices[j].parent)].getState()
                self.path.append(self.vertices[int(self.vertices[j].parent)])
                j = self.vertices[j].parent
            self.path.append(self.vInit)

            self.pathReversed = []
            for v in reversed(self.path):
                self.pathReversed.append(v)

            if self.plotStore is not None:
                self.plotAll()
                self.plotStore.RRTpaths.append(self.pathReversed)
            return successFlag
        else:
            for i in range(stopCount):
                self.extend()
                print 'RRT iteration count is: ' + str(self.iterationCount)
                self.iterationCount += 1

            return successFlag   

    def generateNoise(self):
        dW_1 = self.alpha / self.r * np.random.normal(0.0, np.sqrt(self.dt))
        dW_2 = self.alpha / self.r * np.random.normal() / np.sqrt(self.dt)

        # Why is this here?
        # dw_1 = 0.0

        return dW_1 

    def getDistance(self, v1, v2):
        return sqrt((v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2)

    def getNN(self, vRand):        
        vNearest = self.vertices[0]
        vNearestIndex = 0
        xVertices = [v.x for v in self.vertices]
        for i, v in enumerate(self.verticesSteered):
            if self.getDistance(v, vRand) < self.getDistance(vNearest, vRand):
                vNearest = v
                vNearestIndex = xVertices.index(v.x)
        return (vNearest, vNearestIndex)   

    def onObstacle(self, v):
        onObstacle = False
        boundaryOffset = 0.0
        for obstacle in self.obstacles:
            x, y = v.getState()[0:2]
            if x >= obstacle.center[0] - obstacle.size[0] / 2 - boundaryOffset and x <= obstacle.center[0] + obstacle.size[0] / 2 + boundaryOffset:
                if y >= obstacle.center[1] - obstacle.size[1] / 2 - boundaryOffset and y <= obstacle.center[1] + obstacle.size[1] / 2 + boundaryOffset:
                    onObstacle = True

        return onObstacle

    def obstacleFree(self, v1, v2):
        A = [v1.x, v1.y]
        B = [v2.x, v2.y]
        for obstacle in self.obstacles:
            x1, x2, y1, y2 = obstacle.getCorners()
            C1 = [x1, y1]
            D1 = [x1, y2]
            C2 = [x1, y1]
            D2 = [x2, y1]
            C3 = [x2, y1]
            D3 = [x2, y2]
            C4 = [x1, y2]
            D4 = [x2, y2]
            intersect1 = ccw(A, C1, D1) != ccw(B, C1, D1) and ccw(A, B, C1) != ccw(A, B, D1)
            intersect2 = ccw(A, C2, D2) != ccw(B, C2, D2) and ccw(A, B, C2) != ccw(A, B, D2)
            intersect3 = ccw(A, C3, D3) != ccw(B, C3, D3) and ccw(A, B, C3) != ccw(A, B, D3)
            intersect4 = ccw(A, C4, D4) != ccw(B, C4, D4) and ccw(A, B, C4) != ccw(A, B, D4)
            if intersect1 == True or intersect2 == True or intersect3 == True or intersect4 == True:
                return False

        return True

    def obstacleFreeVertices(self, newVertices,vertex=False):
        obstacleFree = []
        if vertex == False:
            for i in range(1,len(newVertices)):
                obstacleFree.append(self.obstacleFree(Vertex(*newVertices[i]), Vertex(*newVertices[i-1])))
            return np.all(obstacleFree)
        elif vertex == True:
            for i in range(1,len(newVertices)):
                obstacleFree.append(self.obstacleFree(newVertices[i], newVertices[i-1]))
            return np.all(obstacleFree)

    def plotPath(self, path):
        plt.plot([ v.x for v in path ], [ v.y for v in path ], '-b', linewidth=7.0)

    def plotAll(self):
        fig = plt.figure(figsize=(20, 20))
        plt.title('Sampling-based path planning using stochastic optimal control \n Alpha = ' + str(self.alpha), fontsize=20)
        plt.axis('equal')
        for obstacle in self.obstacles:
            x = []
            y = []
            x.extend([obstacle.center[0] - obstacle.size[0] / 2])
            x.extend([obstacle.center[0] - obstacle.size[0] / 2])
            x.extend([obstacle.center[0] + obstacle.size[0] / 2])
            x.extend([obstacle.center[0] + obstacle.size[0] / 2])
            x.extend([obstacle.center[0] - obstacle.size[0] / 2])
            y.extend([obstacle.center[1] - obstacle.size[1] / 2])
            y.extend([obstacle.center[1] + obstacle.size[1] / 2])
            y.extend([obstacle.center[1] + obstacle.size[1] / 2])
            y.extend([obstacle.center[1] - obstacle.size[1] / 2])
            y.extend([obstacle.center[1] - obstacle.size[1] / 2])
            obstaclePlot = plt.plot(x, y, 'r')

        try:
            rrtPathPlot = plt.plot([ v.x for v in self.pathReversed ], [ v.y for v in self.pathReversed ], '-b', linewidth=3.0)
            for p in self.plotStore.RRTpaths:
                plt.plot([ v.x for v in p ], [ v.y for v in p ], linewidth=3.0)
        except:
            pass

        pirrtPathPlot = plt.plot([ v.x for v in self.plotStore.path ], [ v.y for v in self.plotStore.path ], '-r', linewidth=7.0)
        # rrtVerticesPlot = plt.scatter([ v.x for v in self.plotStore.allRRTVertices ], [ v.y for v in self.plotStore.allRRTVertices ], c='cyan')
        rrtVerticesPlot = plt.scatter([ v.x for v in self.vertices ], [ v.y for v in self.vertices ], c='green')
        # rrtVerticesPlot = None
        # rrtSampledPointsPlot = plt.scatter([ v.x for v in self.plotStore.sampledPoints ], [ v.y for v in self.plotStore.sampledPoints ], c='orange')
        rrtSampledPointsPlot = None
        initPlot = plt.scatter(self.plotStore.vInit.x, self.plotStore.vInit.y, c='r')
        goalPlot = plt.scatter(self.plotStore.vGoal.x, self.plotStore.vGoal.y, c='g')
        plt.legend([initPlot,
         goalPlot,
         pirrtPathPlot,
         rrtVerticesPlot,
         rrtSampledPointsPlot], ['Start',
         'Goal',
         'PIRRT Path',
         'Vertices',
         'Sampled Points'], loc=3)
        plt.grid()
        plt.xlim(-10., 10.)
        plt.ylim(-10., 10.)
        plt.savefig(self.plotStore.plotSaveDir + 'RRT_alpha_' + str(self.alpha) + '_obstacle_' + self.obstacleType + '_' + str(self.plotStore.plotIndex) + '.png')
        self.plotStore.plotIndex += 1

    def reachedGoal(self, v):
        if sqrt((v.x - self.vGoal.x) ** 2 + (v.y - self.vGoal.y) ** 2) <= self.goalDist:
            return True
        else:
            return False   

    def returnPlot(self, ax, n):
        for obstacle in self.obstacles:
            x = []
            y = []
            x.extend([obstacle.center[0] - obstacle.size[0] / 2])
            x.extend([obstacle.center[0] - obstacle.size[0] / 2])
            x.extend([obstacle.center[0] + obstacle.size[0] / 2])
            x.extend([obstacle.center[0] + obstacle.size[0] / 2])
            x.extend([obstacle.center[0] - obstacle.size[0] / 2])
            y.extend([obstacle.center[1] - obstacle.size[1] / 2])
            y.extend([obstacle.center[1] + obstacle.size[1] / 2])
            y.extend([obstacle.center[1] + obstacle.size[1] / 2])
            y.extend([obstacle.center[1] - obstacle.size[1] / 2])
            y.extend([obstacle.center[1] - obstacle.size[1] / 2])
            obstaclePlot = ax.plot(x, y, 'r')

        if n == 1:
            rrtPathPlot1 = ax.plot([ v.x for v in self.pathReversed ], [ v.y for v in self.pathReversed ], linewidth=3.0, color='#ff1493')
        if n == 2:
            rrtPathPlot2 = ax.plot([ v.x for v in self.pathReversed ], [ v.y for v in self.pathReversed ], linewidth=3.0, color='#0000ff')
        if n == 3:
            rrtPathPlot3 = ax.plot([ v.x for v in self.pathReversed ], [ v.y for v in self.pathReversed ], linewidth=3.0, color='#000000')
        pirrtPathPlot = ax.plot([ v.x for v in self.plotStore.path ], [ v.y for v in self.plotStore.path ], '-r', linewidth=7.0)
        rrtVerticesPlot = ax.scatter([ v.x for v in self.plotStore.allRRTVertices ], [ v.y for v in self.plotStore.allRRTVertices ], c='cyan')
        rrtSampledPointsPlot = ax.scatter([ v.x for v in self.plotStore.sampledPoints ], [ v.y for v in self.plotStore.sampledPoints ], c='orange')
        initPlot = ax.scatter(self.plotStore.vInit.x, self.plotStore.vInit.y, c='r')
        goalPlot = ax.scatter(self.plotStore.vGoal.x, self.plotStore.vGoal.y, c='g')
        if n == 1:
            ax.legend([initPlot,
             goalPlot,
             pirrtPathPlot,
             rrtVerticesPlot,
             rrtSampledPointsPlot,
             rrtPathPlot1], ['Start',
             'Goal',
             'PIRRT Path',
             'Vertices',
             'Sampled Points',
             'Some1 '], loc=3)
        if n == 2:
            ax.legend([initPlot,
             goalPlot,
             pirrtPathPlot,
             rrtVerticesPlot,
             rrtSampledPointsPlot,
             rrtPathPlot2], ['Start',
             'Goal',
             'PIRRT Path',
             'Vertices',
             'Sampled Points',
             'Some2'], loc=3)
        if n == 3:
            ax.legend([initPlot,
             goalPlot,
             pirrtPathPlot,
             rrtVerticesPlot,
             rrtSampledPointsPlot,
             rrtPathPlot3], ['Start',
             'Goal',
             'PIRRT Path',
             'Vertices',
             'Sampled Points',
             'Some3'], loc=3)
        return ax 

    def sample(self):
        vRand = deepcopy(self.vGoal)
        while (vRand.x == self.vGoal.x) is True and (vRand.y == self.vGoal.y) is True:
            vRand.x = np.random.uniform(self.searchSpace[0], self.searchSpace[1])
            vRand.y = np.random.uniform(self.searchSpace[0], self.searchSpace[1])
            if self.onObstacle(vRand) == True:
                vRand.x = self.vGoal.x
                vRand.y = self.vGoal.y
        return vRand 

    def steerControlled(self, vNearest, vNearestIndex, vRand):
        numSteps = 10
        startTime = time.time()
        # print 'vNearest.time: ' + str(vNearest.time)
        # print 'vNearestIndex: ' + str(vNearestIndex)
        newVertices = np.zeros((numSteps + 1, 10))
        if hasattr(self, 'controlSpline'):
            dtheta = self.controlSpline(vNearest.time + self.dt) / self.r
            dtheta += self.generateNoise()
        else:
            dtheta = self.computeSteeringAngle(vRand, vNearest) * self.dt / self.r
            dtheta += self.generateNoise()
        dx = self.velocity * cos(vNearest.theta)
        dy = self.velocity * sin(vNearest.theta)
        newVertices[0, 0:2] = np.array([vNearest.x, vNearest.y]) + self.dt * np.array([dx, dy])
        newVertices[0, 2] = vNearest.theta + dtheta        
        newVertices[0, 3] = vNearest.time + self.dt
        newVertices[0, 4] = dtheta * self.r
        newVertices[0, 5] = vNearestIndex
        # print 'newVertices[0,0:6]: ' + str(newVertices[0,0:6])        
        newVertexIndex = len(self.vertices)
        for i in range(1, numSteps + 1):
            dx = self.velocity * cos(newVertices[i - 1, 2])
            dy = self.velocity * sin(newVertices[i - 1, 2])
            newVertices[i, 0:2] = newVertices[i - 1, 0:2] + self.dt * np.array([dx, dy])
            if hasattr(self, 'controlSpline'):
                dtheta = self.controlSpline(newVertices[i - 1, 3] + self.dt) / self.r
                # dtheta = self.computeSteeringAngle(vRand, Vertex(*newVertices[i - 1])) * self.dt / self.r                
                # dtheta += self.generateNoise()
                noise = self.generateNoise()
            else:
                dtheta = self.computeSteeringAngle(vRand, Vertex(*newVertices[i - 1])) * self.dt / self.r
                # print 'dtheta: ' + str(dtheta)
                # dtheta += self.generateNoise()
                noise = self.generateNoise()
                # print 'dtheta: ' + str(dtheta)
            newVertices[i, 2] = newVertices[i - 1, 2] + dtheta + noise
            newVertices[i, 3] = newVertices[i - 1, 3] + self.dt
            # newVertices[i, 4] = self.computeSteeringAngle(vRand, Vertex(*newVertices[i - 1]))
            if hasattr(self, 'controlSpline'):
                newVertices[i, 4] = noise*self.r
            else:
                newVertices[i, 4] = dtheta*self.r
            # print 'newVertices[i, 4]: ' + str(newVertices[i, 4])
            newVertices[i, 5] = newVertexIndex + i - 1
            # print 'newVertices[i,0:6]: ' + str(newVertices[i,0:6])

        return newVertices   

    def steerUncontrolled(self, vNearest, vNearestIndex, vRand):
        numSteps = 10
        numTries = 5
        minDist = float('inf')
        startTime = time.time()
        newVertices = np.zeros((numTries, numSteps + 1, 6))
        tryIndex = 0
        for n in range(numTries):
            dx = self.velocity * cos(vNearest.theta)
            dy = self.velocity * sin(vNearest.theta)
            if hasattr(self, 'controlSpline'):
                dtheta = self.controlSpline(0.0) / self.r
                dtheta += self.generateNoise()
            else:
                dtheta = self.generateNoise()
            newVertices[n, 0, 0:2] = np.array([vNearest.x, vNearest.y]) + self.dt * np.array([dx, dy])
            newVertices[n, 0, 2] = vNearest.theta + dtheta
            newVertices[n, 0, 3] = vNearest.time + self.dt
            newVertices[n, 0, 4] = dtheta * self.r
            newVertices[n, 0, 5] = vNearestIndex
            newVertexIndex = len(self.vertices)
            for i in range(1, numSteps + 1):
                dx = self.velocity * cos(newVertices[n, i - 1, 2])
                dy = self.velocity * sin(newVertices[n, i - 1, 2])
                if hasattr(self, 'controlSpline'):
                    dtheta = self.controlSpline(self.dt * i) / self.r
                    dtheta += self.generateNoise()
                else:
                    dtheta = self.generateNoise()
                # print 'dtheta: ' + str(dtheta)
                # print 'newVertices[n,i-1,2] + dtheta: ' + str(newVertices[n, i - 1, 2] + dtheta)
                newVertices[n, i, 0:2] = newVertices[n, i - 1, 0:2] + self.dt * np.array([dx, dy])
                newVertices[n, i, 2] = newVertices[n, i - 1, 2] + dtheta
                newVertices[n, i, 3] = vNearest.time + i * self.dt
                newVertices[n, i, 4] = dtheta * self.r
                newVertices[n, i, 5] = newVertexIndex + i - 1

            dist = sqrt((newVertices[n, -1, 0] - vRand.x) ** 2 + (newVertices[n, -1, 1] - vRand.y) ** 2)
            if dist < minDist:
                tryIndex = n

        return newVertices[tryIndex, :, :]
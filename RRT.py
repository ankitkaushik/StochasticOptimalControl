import sys
from math import sqrt, cos, sin, atan, atan2, pi, tan, fabs
import numpy as np
from copy import deepcopy
from Vertex import Vertex
from Obstacle import Obstacle
import time
import matplotlib.pyplot as plt

import matplotlib.pylab as pylab
params = {'legend.fontsize': 'xx-large',
          'figure.figsize': (15, 5),
         'axes.labelsize': 'xx-large',
         'axes.titlesize':'xx-large',
         'xtick.labelsize':'xx-large',
         'ytick.labelsize':'xx-large'}
pylab.rcParams.update(params)

class RRT(object):

    def __init__(self, vInit, vGoal, dt, velocity, wheelBase, steeringRatio, alpha, r, plotStore=None,plottingInterval='end'):

        self.vInit = vInit 
        self.vGoal = vGoal
        self.goalDist = 1
        # vertex: [x,y,theta,time,parent,control input]
        self.vertices = [vInit]
        # self.edges = []
        self.searchSpace = [min(vInit.x, vInit.y), max(vGoal.x, vGoal.y)]
        self.searchSpace = [-9, 9]
        # self.time = zInit[2]
        self.dt = dt
        self.velocity = velocity
        self.wheelBase = wheelBase
        self.steeringRatio = steeringRatio
        self.alpha = alpha
        self.r = r

        self.sampledPoints = []

        self.createObstacles(obstacleType='single')

        print 'rrt initialized with ' + str(self.vInit.getState())

        self.plotStore = plotStore
        self.plottingInterval = plottingInterval

    def plotPath(self,path):
        plt.plot([v.x for v in path], [v.y for v in path], '-b',linewidth=7.0)

    def plotAll(self):
        fig = plt.figure(figsize=(20,20))
        plt.title('Sampling-based path planning using stochastic optimal control',fontsize=20)
        plt.axis('equal')  
        plt.grid(1)
        initPlot = plt.scatter(self.plotStore.vInit.x, self.plotStore.vInit.y, c = 'r')     
        goalPlot = plt.scatter(self.plotStore.vGoal.x, self.plotStore.vGoal.y, c = 'g')    
        for obstacle in self.obstacles:    
            x = []
            y = []
        #     print obstacle.center[0] - obstacle.size[0]/2
            x.extend([obstacle.center[0] - obstacle.size[0]/2])
            x.extend([obstacle.center[0] - obstacle.size[0]/2])
            x.extend([obstacle.center[0] + obstacle.size[0]/2])
            x.extend([obstacle.center[0] + obstacle.size[0]/2])
            x.extend([obstacle.center[0] - obstacle.size[0]/2])
            y.extend([obstacle.center[1] - obstacle.size[1]/2])
            y.extend([obstacle.center[1] + obstacle.size[1]/2])
            y.extend([obstacle.center[1] + obstacle.size[1]/2])
            y.extend([obstacle.center[1] - obstacle.size[1]/2])
            y.extend([obstacle.center[1] - obstacle.size[1]/2])
            obstaclePlot = plt.plot(x,y,'r')
        try:
            rrtPathPlot = plt.plot([v.x for v in self.pathReversed], [v.y for v in self.pathReversed], '-b',linewidth=3.0)
            # rrtPathPlot = plt.scatter([v.x for v in self.pathReversed], [v.y for v in self.pathReversed], c='blue')
        except:
            pass
        pirrtPathPlot = plt.plot([v.x for v in self.plotStore.path], [v.y for v in self.plotStore.path], '-r',linewidth=7.0)
        rrtVerticesPlot = plt.scatter([v.x for v in self.plotStore.allRRTVertices],[v.y for v in self.plotStore.allRRTVertices],c='cyan')
        rrtSampledPointsPlot = plt.scatter([v.x for v in self.plotStore.sampledPoints],[v.y for v in self.plotStore.sampledPoints],c='orange')
        plt.legend([initPlot,goalPlot,pirrtPathPlot,rrtVerticesPlot,rrtSampledPointsPlot], ['Start', 'Goal','PIRRT Path','Vertices','Sampled Points'], loc=3)
        plt.grid()
        plt.savefig(self.plotStore.plotSaveDir+str(self.plotStore.plotIndex)+'.png')
        self.plotStore.plotIndex += 1

    def createObstacles(self,obstacleType='single'):
        self.obstacles = []
        # Single slit
        if obstacleType == 'single':            
            self.obstacles.append(Obstacle(center=[-2,1.25], size=[0.5,2]))
            self.obstacles.append(Obstacle(center=[-2,-1.25], size=[0.5,2]))
            # self.obstacles.append([[-2.25,2.25],[-2.25,0.25],[-1.75,2.25],[-1.75,0.25]])
            # self.obstacles.append([[-2.25,-0.25],[-2.25,-2.25],[-1.75,-0.25],[-1.75,-2.25]])
        # Double slit
        elif obstacleType == 'double':
            self.obstacles.append(Obstacle(center=[-2,0.], size=[3.5,1.2]))
            self.obstacles.append(Obstacle(center=[-2,1.75], size=[3.5,1.5]))
            self.obstacles.append(Obstacle(center=[-2,-1.75], size=[3.5,1.5]))
    
    def reachedGoal(self, v):
        print sqrt((v.x - self.vGoal.x)**2 + (v.y - self.vGoal.y)**2)
        if sqrt((v.x - self.vGoal.x)**2 + (v.y - self.vGoal.y)**2) <= self.goalDist:
            return True
        else:
            return False
            
    def getDistance(self, v1, v2):
        return sqrt((v1.x - v2.x)**2 + (v1.y - v2.y)**2)

    def getNN(self, vRand, path = False):

        if path == False:
            vNearest = self.vertices[0]
            vNearestIndex = 0
            # print 'nearest distance: ' + str(self.getDistance(vNearest,vRand))
            for i,v in enumerate(self.vertices):
                # print 'vertex: ' + str(v.getState())
                # print 'vertex distance: ' + str(self.getDistance(v,vRand))
                if self.getDistance(v,vRand) < self.getDistance(vNearest,vRand):
                    vNearest = v
                    vNearestIndex = i
            # print 'nearest distance: ' + str(self.getDistance(vNearest,vRand))
            return vNearest, vNearestIndex
            
        if path == True:
            vNearest = self.path[0]
            vNearestIndex = 0
            for i,v in enumerate(self.path):
                if self.getDistance(v,vRand) < self.getDistance(vNearest, vRand):
                    vNearest = v
                    vNearestIndex = i
            return vNearest, vNearestIndex

    def extend(self):

        obstacleFreeVertices = False
        while obstacleFreeVertices == False:        
            vRand = self.sample()
            self.sampledPoints.append(vRand)
            if self.plotStore is not None:
                self.plotStore.sampledPoints.append(vRand)
            # print 'vRand: ' + str(vRand.getState())
            vNearest, vNearestIndex = self.getNN(vRand)
            # print 'vNearest: ' + str(vNearest.getState())
            newVertices = self.steer2(vNearest, vNearestIndex, vRand)
            # print newVertices
            obstacleFreeVertices = self.obstacleFreeVertices(newVertices)
            print obstacleFreeVertices
            if obstacleFreeVertices == True: 
                for i in range(newVertices.shape[0]):
                    # print 'these are supposed to be obstacle free vertices'
                    # print Vertex(*newVertices[i]).getState()
                    self.vertices.append(Vertex(*newVertices[i]))
                    if self.plotStore is not None:
                        self.plotStore.allRRTVertices.append(Vertex(*newVertices[i]))
                    # print [v.getState() for v in self.vertices]
            if self.plotStore is not None:
                if self.plottingInterval != 'end':
                    if self.iterationCount % 100 == 0:
                        print 'plotting!'
                        self.plotAll()
                # print 'newly steered to vertex is ' + str(Vertex(*newVertices[-1]).getState())

            # If we don't want to consider obstacles
            # for i in range(1,newVertices.shape[0]):
            #     self.vertices.append(Vertex(*newVertices[i]))

    def sample(self):

        vRand = deepcopy(self.vGoal)
        while (vRand.x == self.vGoal.x) is True and (vRand.y == self.vGoal.y) is True:
            # print 'searchSpace: ' + str(self.searchSpace)
            vRand.x = np.random.uniform(self.searchSpace[0], self.searchSpace[1])
            vRand.y = np.random.uniform(self.searchSpace[0], self.searchSpace[1])
            if self.obstacleFree(vRand) == False:
                vRand.x = self.vGoal.x
                vRand.y = self.vGoal.y
        # print 'newly sampled point is ' + str(vRand.getState())
        return vRand

    def steer(self, vNearest, vNearestIndex, vRand):
        
        numSteps = np.random.randint(1,10)
        numTries = 5

        # endState = np.zeros(3)
        minDist = float('inf')
        startTime = time.time()
        newVertices = np.zeros((numTries,numSteps+1,6))
        # randomOffsets = np.zeros((numTries,numSteps))
        tryIndex = 0
        for n in range(numTries):

            # If we want to set the first new vertex to the near vertex
            # newVertices[n,0,:] = np.array([vNearest.getState()])

            # First new vertex
            dx = self.velocity*cos(vNearest.theta)
            dy = self.velocity*sin(vNearest.theta)              
            randomOffset = np.random.normal(0.0, np.sqrt(self.dt))
            dtheta = (self.alpha/self.r)*randomOffset
            newVertices[n,0,0:2] = np.array([vNearest.x,vNearest.y]) + self.dt*np.array([dx, dy])
            newVertices[n,0,2] = vNearest.theta + dtheta
            newVertices[n,0,3] = vNearest.time+self.dt
            newVertices[n,0,4] = randomOffset
            newVertices[n,0,5] = vNearestIndex
            newVertexIndex = len(self.vertices)

            for i in range(1,numSteps+1):                
                # print newVertices[n,i-1,2]
                dx = self.velocity*cos(newVertices[n,i-1,2])
                # print 'dx: '+str(dx)
                dy = self.velocity*sin(newVertices[n,i-1,2])
                # print 'dy: '+str(dy)                
                randomOffset = np.random.normal(0.0, np.sqrt(self.dt))
                # print 'randomOffset: ' + str(randomOffset)                           

                # Checking if future vertices are in collision path, deprecated though

                # futureVertex = np.array([-2,1.25,0,0,0,0])
                # print futureVertex
                # print self.obstacleFree(Vertex(*futureVertex))            
                # while self.obstacleFree(Vertex(*futureVertex)) == False:
                #   print 'futureVertex: ' + str(Vertex(*futureVertex).getState())
                #   randomOffset = np.random.randn()
                #   print 'randomOffset: ' + str(randomOffset)
                #   dtheta = (1/self.r)*self.alpha*sqrt(self.dt)*randomOffset
                #   print 'dtheta: ' + str(dtheta) 
                #   future_dx = self.velocity*cos(newVertices[n,i-1,2] + dtheta)
                #   future_dy = self.velocity*sin(newVertices[n,i-1,2] + dtheta)
                #   futureVertex[0:2] = newVertices[n,i-1,0:2] + self.dt*np.array([dx, dy]) + self.dt*np.array([future_dx, future_dy])

                dtheta = (self.alpha/self.r)*randomOffset
                # print 'dtheta: '+str(dtheta)
                # print 'states-1: ' + str(states[n,i-1,:])
                # print self.dt*np.array([dx, dy, dtheta])
                # print 'states[i,0]: ' + str(states[i,0])
                # print 'states[-1,0]: ' + str(states[-1,0])
                # print 'self.dt*dx: ' + str(self.dt*dx)
                # print newVertices[n,i,0:2]
                # print newVertices[n,i-1,0:2]
                newVertices[n,i,0:2] = newVertices[n,i-1,0:2] + self.dt*np.array([dx, dy])
                newVertices[n,i,2] = newVertices[n,i-1,2] + dtheta
                newVertices[n,i,3] = vNearest.time+i*self.dt
                newVertices[n,i,4] = randomOffset
                newVertices[n,i,5] = newVertexIndex+i-2

                # if self.obstacleFree(Vertex(*newVertices[n,i,:])) == False:
                #   print newVertices[n,i-1,:]
                #   print newVertices[n,i,:]
                #   sys.exit()

                # print 'states[i,0]: ' + str(states[i,0])
                # states[i,1] = states[i-1,1] + self.dt*dy
                # states[i,2] = states[i-1,2] + self.dt*dtheta
                # print 'states: ' + str(states[n,i,:])
                # randomOffsets[n,i] = randomOffset
                # print newVertices

            dist = sqrt((newVertices[n,-1,0] - vRand.x)**2 + (newVertices[n,-1,1] - vRand.y)**2)
            if dist < minDist:
                tryIndex = n
                # minDist = dist
                # endState = states[n,-1,:]
            # print 'states for statesAll: ' +str(states)
            # statesAll[i,:,:] = deepcopy(states)

        # print 'steering completed in ' + str(time.time()-startTime) + ' s'
        # return Vertex(endState[0],endState[1],zNearest.parent,endState[2])
        # return states, Vertex(endState[0], endState[1], zNearestIndex, endState[2]), randomOffsets
        return newVertices[tryIndex,:,:]

    def computeSteeringAngle(self,trackVertex,currentVertex):
        xDistance = trackVertex.x-currentVertex.x
        yDistance = trackVertex.y-currentVertex.y
        L = np.sqrt(xDistance**2 + yDistance**2)
        alpha1 = atan2(yDistance,xDistance)

        if alpha1<0:
            alpha2 = alpha1+(2*pi) - currentVertex.theta
        else:
            alpha2 = alpha1 - currentVertex.theta

        omega = 2*self.velocity*sin(alpha2)/L
        return alpha1

    def steer2(self, vNearest, vNearestIndex, vRand):

        numSteps = 10
        startTime = time.time()
        newVertices = np.zeros((numSteps+1,6))

        # First new vertex
        dx = self.velocity*cos(vNearest.theta)
        dy = self.velocity*sin(vNearest.theta)              
        dtheta = self.computeSteeringAngle(vRand,vNearest)
        newVertices[0,0:2] = np.array([vNearest.x,vNearest.y]) + self.dt*np.array([dx, dy])
        # newVertices[0,2] = vNearest.theta+dtheta
        newVertices[0,2] = dtheta
        newVertices[0,3] = vNearest.time+self.dt
        newVertices[0,4] = dtheta
        newVertices[0,5] = vNearestIndex
        newVertexIndex = len(self.vertices)
        currentVertex = Vertex(*newVertices[0])

        for i in range(1,numSteps+1):  
            print i              
            dx = self.velocity*cos(newVertices[i-1,2])
            dy = self.velocity*sin(newVertices[i-1,2])               
            dtheta = self.computeSteeringAngle(vRand,currentVertex)
            newVertices[i,0:2] = newVertices[i-1,0:2] + self.dt*np.array([dx, dy])
            # newVertices[i,2] = newVertices[i-1,2]+dtheta3
            newVertices[i,2] = dtheta
            newVertices[i,3] = vNearest.time+i*self.dt
            newVertices[i,4] = dtheta
            newVertices[i,5] = newVertexIndex+i-2
            currentVertex = Vertex(*newVertices[i])

        # print 'steering completed in ' + str(time.time()-startTime) + ' s'
        return newVertices

    def extractPath(self, stopCount=np.inf, stopAtGoal=True):
        self.path = []
        self.iterationCount = 0
        lastVertex = self.vertices[-1]
        if stopAtGoal == True:
            startTime = time.time()
            while self.reachedGoal(lastVertex) == False:
                if self.iterationCount > stopCount:
                    break
                self.extend()
                # print 'vertices length:' + str(len(self.vertices))
                lastVertex = self.vertices[-1]
                print lastVertex.getState()
                # print 'lastVertex updated'
                print 'RRT iteration count is: ' + str(self.iterationCount)
                self.iterationCount += 1            

            print 'path found in ' + str(self.iterationCount) + ' iterations'
            print 'path found in ' + str(time.time()-startTime) + ' s'
            # for i in range(0,10):
            #   j = int(self.vertices[i].parent)
            #   print self.vertices[j].parent
            print 'last added vertex is ' + str(lastVertex.getState())
            self.path.append(lastVertex)
            j = -1
            while self.vertices[j].parent is not 0:
                self.path.append(self.vertices[int(self.vertices[j].parent)])
                j = self.vertices[j].parent

            self.pathReversed = []
            for v in reversed(self.path):
                self.pathReversed.append(v)

            if self.plotStore is not None:
                self.plotAll()

            # self.controls = []
            # for i,v in enumerate(self.pathReversed[:-1]):
            #   currentPosition = np.array([v.x,v.y])
            #   trackPoint = np.array([self.pathReversed[i+1].x,self.pathReversed[i+1].y])
            #   delta = self.computeControlInput(currentPosition, trackPoint, velocity=self.velocity, wheelBase=self.wheelBase, steeringRatio=self.steeringRatio)
            #   self.controls.append(delta)
            # self.controls.append(0)

        else:
            for i in range(stopCount):
                self.extend()
                print 'RRT iteration count is: ' + str(self.iterationCount)
                self.iterationCount += 1    


    # def generateTrajectory(self):

    #   trajectory = []
    #   vertexList = []
    #   lastVertex = self.zInit

    #   while self.reachedGoal(lastVertex) == False:
    #       newVertex = self.extend()
    #       vertexList.append(newVertex)
    #       lastVertex = newVertex

    #   print 'len(vertexList): ' + str(len(vertexList))
    #   trajectory.append(lastVertex)
    #   j = -1
    #   while vertexList[j].parent is not -1:
    #       try:
    #           trajectory.append(vertexList[vertexList[j].parent])
    #           j = vertexList[j].parent
    #       except:
    #           print vertexList[j].parent
    #           break

    #   trajectoryOutput = []
    #   for v in reversed(trajectory):
    #       trajectoryOutput.append(v)

    #   return trajectoryOutput


    def obstacleFree(self, v):

        obstacleFree = True
        boundaryOffset = 0.0
        for obstacle in self.obstacles:
            x,y = v.getState()[0:2]
            if x>=(obstacle.center[0]-obstacle.size[0]/2)-boundaryOffset and  x<=(obstacle.center[0]+obstacle.size[0]/2)+boundaryOffset:
                if y>=(obstacle.center[1]-obstacle.size[1]/2)-boundaryOffset and  y<=(obstacle.center[1]+obstacle.size[1]/2)+boundaryOffset:
                    obstacleFree = False
                    
        return obstacleFree

    def obstacleFreeVertices(self, newVertices):

        obstacleFreeVertices = True
        # samplingPoints = 10
    
        # for i in range(2,len(newVertices)):
        #     xPoints = np.linspace(newVertices[i-1,0],newVertices[i,0],samplingPoints)
        #     yPoints = np.linspace(newVertices[i-1,1],newVertices[i,1],samplingPoints)
        #     for j in range(samplingPoints):
        #         if self.obstacleFree(Vertex(xPoints[j],yPoints[j])) == False:
        #             obstacleFreeVertices = False

        for v in newVertices:
            if self.obstacleFree(Vertex(*v)) == False:
                obstacleFreeVertices = False
        
        return obstacleFreeVertices
    
    def generateRRTControls(self):      

        xCurrent = self.zInit.x
        yCurrent = self.zInit.y
        thetaCurrent = self.zInit.theta
        states = []
        states.append([deepcopy(xCurrent),deepcopy(yCurrent),deepcopy(thetaCurrent)])
        controls = []

        for trackPoint in reversed(self.path[:-1]):

            print 'xCurrent: ' + str(xCurrent)
            print 'yCurrent: ' + str(yCurrent)

            print trackPoint.show()
            print sqrt((xCurrent - trackPoint.x)**2 + (yCurrent - trackPoint.y)**2)

            while sqrt((xCurrent - trackPoint.x)**2 + (yCurrent - trackPoint.y)**2) >= self.goalDist:

                # print 'yo'

                dx = self.velocity*cos(thetaCurrent)*self.dt
                dy = self.velocity*sin(thetaCurrent)*self.dt
                
                xDistance = trackPoint.x-xCurrent
                yDistance = trackPoint.y-yCurrent
                L = np.sqrt(xDistance**2 + yDistance**2)
                alpha1 = atan2(yDistance,xDistance)
                # if alpha1<0:
                #   alpha2 = alpha1+(2*pi) - thetaCurrent
                # else:
                #   alpha2 = alpha1 - thetaCurrent
                omega = 2*self.velocity*sin(alpha1)/L
                delta = atan(omega*self.wheelBase/self.velocity)
                delta = delta*self.steeringRatio*180./np.pi
                delta = -delta              
                dtheta = (self.velocity/self.wheelBase)*tan(delta)

                xCurrent += dx*self.dt
                yCurrent += dy*self.dt
                thetaCurrent += dtheta*self.dt
                states.append([deepcopy(xCurrent),deepcopy(yCurrent),deepcopy(thetaCurrent)])
                controls.append(delta)

        return np.array(states), np.array(controls)

    def computeControlInput(self,currentPosition,trackPoint,velocity,wheelBase,steeringRatio):
        xDistance = trackPoint[0]-currentPosition[0]
        yDistance = trackPoint[1]-currentPosition[1]
        L = np.sqrt(xDistance**2 + yDistance**2)
        alpha1 = atan2(yDistance,xDistance)
        omega = 2*velocity*sin(alpha1)/L
        delta = atan(omega*wheelBase/velocity)
        delta = delta*steeringRatio*180./np.pi
        delta = -delta
        return delta

    def runKinematicModel(self,initialState,numSteps,dt,delta,velocity,wheelBase,steeringRatio):
        states = np.zeros((numSteps,3))
        states[0,:] = initialState

        for i in range((1,numSteps)):

            deltaDegrees = -1*(delta)/steeringRatio
            delta = deltaDegrees*np.pi/180.
            
            dx = velocity*cos(states[i,2])
            dy = velocity*sin(states[i,2])
            dtheta = (velocity/wheelBase)*tan(delta)

            states[i,:] = states[-1,:] + dt*np.array([dx, dy, dtheta])

        return states

    def runKinematicModelControls(self,initialState,numSteps,dt,spline,velocity,wheelBase,steeringRatio):
        states = np.zeros((numSteps,3))
        states[0,:] = initialState

        for i in range(1,numSteps):

            delta = spline(i)
            print 'delta: ' + str(delta)
            deltaDegrees = -1*(delta)/steeringRatio
            delta = deltaDegrees*np.pi/180.
            
            dx = velocity*cos(states[i-1,2])
            print 'dx: ' + str(dx)
            dy = velocity*sin(states[i-1,2])
            print 'states[i,2]: ' + str(states[i,2])
            print 'sin(states[i,2]): ' + str(sin(states[i,2]))
            print 'dy: ' + str(dy)
            dtheta = (velocity/wheelBase)*tan(delta)
            print 'dtheta: ' + str(dt*dtheta)
            print 'states[i,2]: ' + str(states[i,2])

            states[i,:] = states[i-1,:] + dt*np.array([dx, dy, dtheta])
            print 'states[i,2]: ' + str(states[i,2])

        return states
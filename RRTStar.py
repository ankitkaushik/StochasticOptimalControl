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
         'axes.titlesize':'xx-large',
         'xtick.labelsize':'xx-large',
         'ytick.labelsize':'xx-large'}
pylab.rcParams.update(params)

class RRTStar(object):

    def __init__(self, vInit, vGoal, dt, velocity, wheelBase, steeringRatio, alpha, r, plotStore=None,plottingInterval='end'):

        self.vInit = vInit 
        self.vGoal = vGoal
        self.goalDist = 1
        # vertex: [x,y,theta,time,parent,control input]
        self.vertices = [vInit]
        # self.edges = []
        self.searchSpace = [-9,9]
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

        self.searchRadius = 5

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
            rrtPathPlot, = plt.plot([v.x for v in self.pathReversed], [v.y for v in self.pathReversed], '-b',linewidth=3.0, label='RRT Path')
            # rrtPathPlot = plt.scatter([v.x for v in self.pathReversed], [v.y for v in self.pathReversed], c='blue')
        except:
            pass
        pirrtPathPlot, = plt.plot([v.x for v in self.plotStore.path], [v.y for v in self.plotStore.path], '-r',linewidth=7.0, label='PIRRT Path')
        rrtVerticesPlot = plt.scatter([v.x for v in self.plotStore.allRRTVertices],[v.y for v in self.plotStore.allRRTVertices],c='cyan')
        rrtSampledPointsPlot = plt.scatter([v.x for v in self.plotStore.sampledPoints],[v.y for v in self.plotStore.sampledPoints],c='orange')
        plt.legend([initPlot,goalPlot,rrtPathPlot,pirrtPathPlot,rrtVerticesPlot,rrtSampledPointsPlot], ['Start', 'Goal','RRT Path','PIRRT Path','Vertices','Sampled Points'], loc=3)
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
            # print [v.getState() for v in newVertices]
            # print len(newVertices)
            obstacleFreeVertices = self.obstacleFreeVertices(newVertices)
            print obstacleFreeVertices
            # if obstacleFreeVertices == False:
            # print 'old new vertices[-1]: ' + str(newVertices[-1].parent)

            if obstacleFreeVertices:
                for i,v in enumerate(self.vertices):
                    if self.obstacleFree(v,newVertices[-1]):
                        if self.getDistance(v,newVertices[-1]) < self.searchRadius:
                            if v.cost+self.getDistance(v,newVertices[-1]) < vNearest.cost+self.getDistance(vNearest,newVertices[-1]):
                                vNearest = v
                                vNearestIndex = i
                newVertices[-1].parent = vNearestIndex
                newVertices[-1].cost = vNearest.cost+self.getDistance(vNearest,newVertices[-1])

                self.vertices.append(newVertices[-1])
                if self.plotStore is not None:
                    self.plotStore.allRRTVertices.append(newVertices[-1])

                if self.plotStore is not None:
                    if self.plottingInterval != 'end':
                        self.plotAll()

                for i,v in enumerate(self.vertices):
                    if i != newVertices[-1].parent:
                        if self.obstacleFree(v,newVertices[-1]):
                            if self.getDistance(v,newVertices[-1]) < self.searchRadius:
                                if newVertices[-1].cost+self.getDistance(v,newVertices[-1]) < v.cost:
                                    v.parent = len(self.vertices)-1
                                    v.cost = newVertices[-1].cost+self.getDistance(v,newVertices[-1])
              
                # print 'new new vertices[-1]: ' + str(newVertices[-1].parent)

                
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
            if self.onObstacle(vRand) == True:
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
        newVertices = []

        # First new vertex
        dx = self.velocity*cos(vNearest.theta)
        dy = self.velocity*sin(vNearest.theta)              
        dtheta = self.computeSteeringAngle(vRand,vNearest)
        newVertex = np.zeros(7)
        newVertex[0:2] = np.array([vNearest.x,vNearest.y]) + self.dt*np.array([dx, dy])
        newVertex[2] = dtheta
        newVertex[3] = vNearest.time+self.dt
        newVertex[4] = dtheta
        newVertex[5] = vNearestIndex
        newVertex[6] = vNearest.cost+sqrt((vNearest.x - newVertex[0])**2 + (vNearest.y - newVertex[1])**2)
        newVertices.append(Vertex(*newVertex))
        newVertexIndex = len(self.vertices)

        for i in range(1,numSteps+1):                
            dx = self.velocity*cos(newVertices[-1].theta)
            dy = self.velocity*sin(newVertices[-1].theta)               
            dtheta = self.computeSteeringAngle(vRand,newVertices[-1])
            newVertex = np.zeros(7)
            newVertex[0:2] = np.array([newVertices[-1].x,newVertices[-1].y]) + self.dt*np.array([dx, dy])
            newVertex[2] = dtheta
            newVertex[3] = newVertices[-1].time+self.dt
            newVertex[4] = dtheta
            newVertex[5] = newVertexIndex+i-2
            newVertex[6] = newVertices[-1].cost+sqrt((newVertices[-1].x - newVertex[0])**2 + (newVertices[-1].y - newVertex[1])**2)
            # print newVertex
            newVertices.append(Vertex(*newVertex))

        # print 'steering completed in ' + str(time.time()-startTime) + ' s'
        # sys.exit()
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
                print 'vertices length:' + str(len(self.vertices))
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
            # while self.vertices[j].parent == 0:
            #     j -= 1

            while self.vertices[j].parent is not 0:
                print self.vertices[j].parent 
                self.path.append(self.vertices[int(self.vertices[j].parent)])
                j = self.vertices[j].parent
                # print 'j is: ' + str(j)
                # print self.vertices[j].parent
                # print [v.getState() for v in self.vertices[j-5:j+5]]
                # sys.exit()
            self.path.append(self.vInit)
            self.pathReversed = []
            for v in reversed(self.path):
                self.pathReversed.append(v)

            if self.plotStore is not None:
                self.plotAll()

        else:
            for i in range(stopCount):
                self.extend()
                print 'RRT iteration count is: ' + str(self.iterationCount)
                self.iterationCount += 1   

    def onObstacle(self, v):

        onObstacle = False
        boundaryOffset = 0.0
        for obstacle in self.obstacles:
            x,y = v.getState()[0:2]
            if x>=(obstacle.center[0]-obstacle.size[0]/2)-boundaryOffset and  x<=(obstacle.center[0]+obstacle.size[0]/2)+boundaryOffset:
                if y>=(obstacle.center[1]-obstacle.size[1]/2)-boundaryOffset and  y<=(obstacle.center[1]+obstacle.size[1]/2)+boundaryOffset:
                    onObstacle = True

        return onObstacle 


    def obstacleFree(self, v1, v2):

        A = [v1.x, v1.y]
        B = [v2.x, v2.y]
        for obstacle in self.obstacles:
            x1,x2,y1,y2 = obstacle.getCorners()
            C1 = [x1,y1]
            # print C1
            D1 = [x1,y2]
            # print D1
            C2 = [x1,y1]
            # print C2
            D2 = [x2,y1]
            # print D2
            C3 = [x2,y1]
            # print C3
            D3 = [x2,y2]
            # print D3
            C4 = [x1,y2]
            # print C4
            D4 = [x2,y2]
            # print D4
            intersect1 = ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1)
            # print intersect1 
            intersect2 = ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
            # print intersect2 
            intersect3 = ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
            # print intersect3 
            intersect4 = ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
            # print intersect4 

            if intersect1==True or intersect2==True or intersect3==True or intersect4==True: 
                return False

        return True

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
            if self.onObstacle(v) == True:
                obstacleFreeVertices = False
        
        return obstacleFreeVertices
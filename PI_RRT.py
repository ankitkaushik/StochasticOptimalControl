import sys
import time
from copy import deepcopy
from math import sqrt, cos, sin, atan, atan2, pi, tan, fabs, exp
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import interp1d
from Vertex import Vertex
from RRT import RRT
from RRTStar import RRTStar
from plotStore import plotStore
import multiprocessing as mp

class PI_RRT(object):

    def __init__(self, vInit, vGoal, alpha, plotSaveDir, useRRTStar, controlledSteering, obstacleType):
        self.vInit = vInit
        self.vGoal = vGoal
        self.searchSpace = [min(vInit.x, vInit.y), max(vGoal.x, vGoal.y)]
        self.goalDist = 1
        self.path = [vInit]
        self.alpha = alpha
        self.r = 4.0
        self.Lambda = 10
        self.M = 5
        self.p = -1 / self.alpha ** 2
        self.Q = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.Qf = np.array([[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]])
        self.R = self.Lambda / self.alpha ** 2
        self.tHorizon = 1
        self.dt = 0.1
        self.velocity = 2.3
        self.wheelBase = 2.0
        self.steeringRatio = 1
        self.controlDiscretation = self.dt / 1
        self.allRRTVertices = []
        self.sampledPoints = []
        print 'pi_rrt initialized'
        self.plotSaveDir = plotSaveDir
        self.plotStore = plotStore(self.vInit, self.vGoal, self.plotSaveDir)
        self.useRRTStar = useRRTStar
        self.controlledSteering = controlledSteering
        self.obstacleType = obstacleType
        self.plottingInterval = 'end'

    def reachedGoal(self, v):
        if sqrt((v.x - self.vGoal.x) ** 2 + (v.y - self.vGoal.y) ** 2) <= self.goalDist:
            return True
        else:
            return False

    def runRRT(self):
        successFlag = True
        print 'runRRT method called'
        if self.useRRTStar:
            self.RRT = RRTStar(self.path[-1], self.vGoal, self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r, self.plotStore)
        else:
            self.RRT = RRT(self.path[-1], self.vGoal, self.searchSpace, self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r, self.controlledSteering, self.plotStore, self.obstacleType, self.plottingInterval)
        print 'RRT initialized'
        if self.RRT.extractPath():
            print 'RRT path extracted'
            self.allRRTVertices.extend(self.RRT.vertices)
            self.sampledPoints.extend(self.RRT.sampledPoints)
            print 'len of allRRTVertices is ' + str(len(self.allRRTVertices))
            self.rrtStates = self.constructStatesMatrix(self.RRT.pathReversed)
            self.controlSplineRRT = interp1d(self.rrtStates[:, 3], self.rrtStates[:, 4], fill_value='extrapolate')
            return successFlag
        else:
            successFlag = False
            return successFlag

    def runRRTMP(self, trajNum):
        print 'trajectory ' + str(trajNum)
        if self.useRRTStar:
            RRT = RRTStar(self.path[-1], self.vGoal, self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r, self.plotStore)
        else:
            RRT = RRT(self.path[-1], self.vGoal, self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r, self.controlledSteering, self.plotStore)
        RRT.extractPath()
        return RRT.pathReversed

    def generateTrajectory(self, numSteps, dt, alpha, velocity, wheelBase, steeringRatio, r):
        zNew = self.RRT.zInit
        zOrigin = deepcopy(zNew)
        trajectory = [zOrigin]
        dws = []
        for i in range(len(self.RRT.pathReversed) - 1):
            dw = np.random.uniform(0, 1)
            dx = velocity * cos(zNew.theta) * self.RRT.dt
            dy = velocity * sin(zNew.theta) * self.RRT.dt
            dtheta = 1 / r * (self.RRT.controls[i] * self.RRT.dt + self.alpha * dw)
            zNew = Vertex(trajectory[-1].x + dx, trajectory[-1].y + dy, i, theta=trajectory[-1].theta + dtheta)
            trajectory.append(zNew)
            dws.append(dw)

        dws.append(0)
        return (trajectory, dws)

    def generateTrajectories(self):
        self.trajectories = []
        self.dws = []
        for i in range(self.M):
            trajectory, dws = self.generateTrajectory(numSteps=10, dt=0.5, alpha=0.25, velocity=10, wheelBase=2.0, steeringRatio=1, r=0.5)
            self.trajectories.append(trajectory)
            self.dws.append(dws)

    def generateTrajectories2(self):
        successFlag = True
        self.trajectories = []
        for i in range(self.M):
            print 'doing rrt'
            if self.useRRTStar:
                newRRT = RRTStar(self.RRT.vInit, self.RRT.vGoal, self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r, self.plotStore)
            else:
                newRRT = RRT(self.RRT.vInit, self.RRT.vGoal, self.searchSpace, self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r, self.controlledSteering, self.plotStore, self.obstacleType, self.plottingInterval)
                newRRT.assignControlSpline(self.controlSplineRRT)
            if newRRT.extractPath():
                print 'RRT path extracted'
                self.trajectories.append(newRRT.pathReversed)
                self.allRRTVertices.extend(newRRT.vertices)
                self.sampledPoints.extend(self.RRT.sampledPoints)
                print 'len of allRRTVertices is ' + str(len(self.allRRTVertices))
            else:
                successFlag = False
                return successFlag

        return successFlag

    def generateTrajectories3(self):
        successFlag = True
        self.trajectories = []
        for i in range(self.M):
            newRRT = []
            newRRT.append(self.RRT.vInit)
            numSteps = (self.RRT.pathReversed[-1].time-self.RRT.vInit.time)/self.dt
            # for i,v in enumerate(self.RRT.pathReversed): 
            for i in range(int(numSteps)):            
                dx = self.RRT.velocity * cos(newRRT[-1].theta)
                dy = self.RRT.velocity * sin(newRRT[-1].theta)
                newX = newRRT[-1].x + self.dt*dx
                newY = newRRT[-1].y + self.dt*dy
                randomControlShift = self.RRT.generateNoise()
                # newControl = self.controlSplineRRT(v.time)+randomControlShift
                newControl = self.controlSplineRRT(i*self.dt)+randomControlShift
                newTheta = newRRT[-1].theta + (self.controlSplineRRT(i*self.dt)*self.dt/self.r) + (randomControlShift/self.r)
                newTime = i*self.dt
                newVertex = [newX,newY,newTheta,newTime,newControl]
                newRRT.append(Vertex(*newVertex))
            self.trajectories.append(newRRT)
            self.plotStore.RRTpaths.append(newRRT)
        self.RRT.plotAll()
                
        return successFlag

    def generateTrajectoriesMP(self):
        pool = mp.Pool()
        print pool.map(self.runRRTMP, range(5))
        # self.trajectories = pool.map(self.runRRTMP, range(5))

    def computeVariation(self):
        S = np.zeros(len(self.trajectories))
        for k, trajectory in enumerate(self.trajectories):
            dws = self.dws[k]
            trajectoryCost = 0
            for i, v in enumerate(trajectory):
                trajectoryCost += ((v.x - self.RRT.pathReversed[i].x) ** 2 + (v.y - self.RRT.pathReversed[i].y) ** 2) * self.RRT.dt
                trajectoryCost += 0.5 * self.RRT.controls[i] ** 2 * self.RRT.dt
                trajectoryCost += self.alpha * (self.RRT.controls[i] * dws[i])

            S[k] = trajectoryCost

        variation = 0
        for k, trajectory in enumerate(self.trajectories):
            pK = exp(-fabs(self.p) * S[k]) / np.mean(S)
            variation += self.alpha * pK

        return variation

    def constructStatesMatrix(self, path):
        if len(path) == 2:
            print 'len path 2'
            vertices = [ v for v in path ]
            states = np.zeros((4, 5))
            states[(0, 0)] = vertices[0].x
            states[(0, 1)] = vertices[0].y
            states[(0, 2)] = vertices[0].theta
            states[(0, 3)] = 0
            states[(0, 4)] = vertices[0].controlInput
            states[(3, 0)] = vertices[1].x
            states[(3, 1)] = vertices[1].y
            states[(3, 2)] = vertices[1].theta
            states[(3, 3)] = self.RRT.dt * 3
            states[(3, 4)] = vertices[1].controlInput
            states[1:3, 0] = np.linspace(states[(0, 0)], states[(3, 0)], num=4)[1:3]
            states[1:3, 1] = np.linspace(states[(0, 1)], states[(3, 1)], num=4)[1:3]
            states[1:3, 2] = np.linspace(states[(0, 2)], states[(3, 2)], num=4)[1:3]
            states[1:3, 3] = self.RRT.dt * np.array([1, 2])
            states[1:3, 4] = np.linspace(states[(0, 4)], states[(3, 4)], num=4)[1:3]
        if len(path) == 3:
            print 'len path 3'
            vertices = [ v for v in path ]
            if self.RRT.getDistance(vertices[0], vertices[1]) > self.RRT.getDistance(vertices[0], vertices[2]):
                if self.RRT.getDistance(vertices[0], vertices[1]) > self.RRT.getDistance(vertices[1], vertices[2]):
                    vertices = [vertices[0], vertices[1]]
                else:
                    vertices = [vertices[1], vertices[2]]
            elif self.RRT.getDistance(vertices[0], vertices[2]) > self.RRT.getDistance(vertices[1], vertices[2]):
                vertices = [vertices[0], vertices[2]]
            else:
                vertices = [vertices[1], vertices[2]]
            states = np.zeros((4, 5))
            states[(0, 0)] = vertices[0].x
            states[(0, 1)] = vertices[0].y
            states[(0, 2)] = vertices[0].theta
            states[(0, 3)] = 0
            states[(0, 4)] = vertices[0].controlInput
            states[(3, 0)] = vertices[1].x
            states[(3, 1)] = vertices[1].y
            states[(3, 2)] = vertices[1].theta
            states[(3, 3)] = self.RRT.dt * 3
            states[(3, 4)] = vertices[1].controlInput
            states[1:3, 0] = np.linspace(states[(0, 0)], states[(3, 0)], num=4)[1:3]
            states[1:3, 1] = np.linspace(states[(0, 1)], states[(3, 1)], num=4)[1:3]
            states[1:3, 2] = np.linspace(states[(0, 2)], states[(3, 2)], num=4)[1:3]
            states[1:3, 3] = self.RRT.dt * np.array([1, 2])
            states[1:3, 4] = np.linspace(states[(0, 4)], states[(3, 4)], num=4)[1:3]
        elif len(path) > 3:
            print 'len path is ' + str(len(path))
            states = np.zeros((len(path), 5))
            for i, v in enumerate(path):
                states[i, 0] = v.x
                states[i, 1] = v.y
                states[i, 2] = v.theta
                states[i, 3] = v.time
                states[i, 4] = v.controlInput

        return states

    def computeVariation2(self):
        startTime = time.time()
        regCoef = 3.0
        totalCosts = np.zeros(len(self.trajectories))
        trajectoryLengths = [ len(t) for t in self.trajectories ]
        if max(trajectoryLengths) < 4:
            trajectoryStates = np.zeros((len(self.trajectories), 4, 5))
        else:
            trajectoryStates = np.zeros((len(self.trajectories), max(trajectoryLengths), 5))
        if min(trajectoryLengths) < 4:
            trajectoryLengths = [ 4 for t in self.trajectories ]
        for k, trajectory in enumerate(self.trajectories):
            print 'len(trajectory) is: ' + str(len(trajectory))
            trajectoryStates[k, :max(len(trajectory), 4), :] = self.constructStatesMatrix(trajectory)
            pathCosts = np.zeros(len(trajectory))
            noiseCosts = np.zeros(len(trajectory))
            for i, v in enumerate(trajectory[:-1]):
                pathCosts[i] += trajectoryStates[k, i, 0:3].dot(self.Q).dot(trajectoryStates[k, i, 0:3].T)
                pathCosts[i] += 0.5 * self.controlSplineRRT(trajectoryStates[k, i, 3]).T * self.R * self.controlSplineRRT(trajectoryStates[k, i, 3])
                noiseCosts[i] += self.controlSplineRRT(trajectoryStates[k, i, 3]) * trajectoryStates[k, i, 4]

            totalCost = np.trapz(pathCosts[:-1], trajectoryStates[k, :len(trajectory) - 1, 3])
            endStateDiff = trajectoryStates[k, len(trajectory)-1, 0:3] - np.array(self.RRT.vGoal.getState()[0:3])
            totalCost += endStateDiff.dot(self.Qf).dot(endStateDiff.T)
            totalCost += np.trapz(noiseCosts[:-1], trajectoryStates[k, :len(trajectory) - 1, 3])
            totalCosts[k] = totalCost / regCoef

        self.totalCosts = totalCosts

        trajectoryDesirability = np.exp(-totalCosts / self.Lambda)
        print 'trajectoryDesirability: ' + str(trajectoryDesirability)
        while sum(trajectoryDesirability) == 0:
            regCoef = regCoef * 3
            totalCosts = totalCosts / regCoef
            trajectoryDesirability = np.exp(-totalCosts / self.Lambda)

        print 'computed variation in ' + str(time.time() - startTime) + ' s'
        weights = trajectoryDesirability / sum(trajectoryDesirability)
        dU = np.zeros(min(trajectoryLengths))
        for k, trajectory in enumerate(self.trajectories):
            dU += weights[k] * trajectoryStates[k, :min(trajectoryLengths), 4]
            print 'dU: ' + str(dU)

        minTrajectoryIndex = trajectoryLengths.index(min(trajectoryLengths))
        U = np.zeros(min(trajectoryLengths))
        t = np.zeros(min(trajectoryLengths))
        for i in range(min(trajectoryLengths)):
            t[i] = trajectoryStates[minTrajectoryIndex,i,3]
            U[i] = self.controlSplineRRT(t[i]) + dU[i]
            # U[i] = dU[i]  

        return (t, U)

    def computeVariationGPU(self):
        S = np.zeros(len(self.trajectories))
        a_gpu = gpuarray.to_gpu(S.astype(np.float32))
        for k, trajectory in enumerate(self.trajectories):
            dws = self.dws[k]
            trajectoryCost = 0
            for i, v in enumerate(trajectory):
                trajectoryCost += ((v.x - self.RRT.pathReversed[i].x) ** 2 + (v.y - self.RRT.pathReversed[i].y) ** 2) * self.RRT.dt
                trajectoryCost += 0.5 * self.RRT.controls[i] ** 2 * self.RRT.dt
                trajectoryCost += self.alpha * (self.RRT.controls[i] * dws[i])
                a_gpu[i] = np.asarray(trajectoryCost).astype(np.float32)

        variation = 0
        for k, trajectory in enumerate(self.trajectories):
            pK = GPUexp(-fabs(self.p) * a_gpu[k]) / (pycuda.gpuarray.sum(a_gpu) / a_gpu.shape[0])
            variation += self.alpha * pK

        return variation

    def computeVariation(self):
        S = np.zeros(len(self.trajectories))
        Qf = np.fill_diagonal(np.zeros((3, 3)), 10)
        for k, trajectory in enumerate(self.trajectories):
            dws = self.dws[k]
            goalDistance = np.array(trajectory[-1].getState() - self.RRT.zGoal.getState())
            trajectoryCost = np.matrix.transpose(goalDistance) * Qf * goalDistance
            for i, v in enumerate(trajectory):
                trajectoryCost += ((v.x - self.RRT.pathReversed[i].x) ** 2 + (v.y - self.RRT.pathReversed[i].y) ** 2) * self.RRT.dt
                trajectoryCost += 0.5 * self.RRT.controls[i] ** 2 * self.RRT.dt
                trajectoryCost += self.alpha * (self.RRT.controls[i] * dws[i])

            S[k] = trajectoryCost

        variation = 0
        for k, trajectory in enumerate(self.trajectories):
            pK = exp(-fabs(self.p) * S[k]) / np.mean(S)
            variation += self.alpha * pK

        return variation

    def executeControl(self):
        times = range(len(self.RRT.controls))
        variation = self.computeVariation()
        self.RRT.controls += variation
        spline = UnivariateSpline(times, self.RRT.controls)
        states = self.RRT.runKinematicModelControls(self.RRT.zInit.getState(), numSteps=10, dt=self.RRT.dt, spline=spline, velocity=self.RRT.velocity, wheelBase=self.RRT.wheelBase, steeringRatio=self.RRT.steeringRatio)
        return states

    def executeControl2(self, t, U):
        self.t = t
        self.U = U
        self.controlSplinePathIntegral = interp1d(t, U, fill_value='extrapolate')
        self.newPathVertices = [self.path[-1]]
        for i in range(int(self.tHorizon / self.controlDiscretation)):
            dx = self.velocity * cos(self.newPathVertices[-1].theta)
            dy = self.velocity * sin(self.newPathVertices[-1].theta)
            dtheta = self.controlSplinePathIntegral(self.path[-1].time + self.controlDiscretation * i) / self.r
            dtheta += self.RRT.generateNoise()
            newVertex = Vertex(self.newPathVertices[-1].x + self.controlDiscretation * dx, self.newPathVertices[-1].y + self.controlDiscretation * dy, self.newPathVertices[-1].theta + dtheta, self.newPathVertices[-1].time + self.controlDiscretation * i, dtheta)
            if self.RRT.obstacleFree(newVertex, self.newPathVertices[-1]) == True:
                self.newPathVertices.append(newVertex)
            else:
                self.newPathVertices = []
                break

        self.path.extend(self.newPathVertices[1:])
        self.plotStore.path = self.path

    def run(self):
        self.runRRT()
        print 'RRT run'
        self.generateTrajectories2()
        self.executeControl2(*self.computeVariation2())
        print 'new pi_rrt point: ' + str(self.path[-1].getState())

import sys
import time
from copy import deepcopy
from math import sqrt, cos, sin, atan, atan2, pi, tan, fabs, exp
import numpy as np
from scipy.interpolate import UnivariateSpline

from Vertex import Vertex
from RRT import RRT
from plotStore import plotStore

# import pycuda.gpuarray as gpuarray
# import pycuda.driver as cuda
# import pycuda.autoinit
# from pycuda.cumath import exp as GPUexp

class PI_RRT(object):

	def __init__(self,vInit,vGoal,plotSaveDir):

		self.vInit = vInit
		self.vGoal = vGoal
		self.path = [vInit]
		self.alpha = 0.25
		self.r = 4.0
		self.Lambda = 10
		self.M = 5
		self.p = -1/(self.alpha**2)
		self.Q = np.array([[  0.,   0.,   0.],
							[  0.,   0.,   0.],
							[  0.,   0.,   0.]])
		self.Qf = np.array([[  10.,   0.,   0.],
							[  0.,   10.,   0.],
							[  0.,   0.,   10.]])
		self.R = self.Lambda/(self.alpha**2)
		self.tHorizon = 0.5
		self.regCoef = 1000
		
		#RRT params
		self.dt = 0.1
		self.velocity = 2.3
		self.wheelBase = 2.0
		self.steeringRatio = 1

		self.allRRTVertices = []
		self.sampledPoints = []

		print 'pi_rrt initialized'

		self.plotSaveDir = plotSaveDir
		self.plotStore = plotStore(self.vInit,self.vGoal,self.plotSaveDir)

	def runRRT(self):
		print 'runRRT method called'
		self.RRT = RRT(self.path[-1],self.vGoal,self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r,self.plotStore)
		print 'RRT initialized'
		self.RRT.extractPath()
		print 'RRT path extracted'
		self.allRRTVertices.extend(self.RRT.vertices)
		self.sampledPoints.extend(self.RRT.sampledPoints)
		print 'len of allRRTVertices is ' + str(len(self.allRRTVertices))

	def generateTrajectory(self, numSteps, dt, alpha, velocity, wheelBase, steeringRatio, r):

		zNew = self.RRT.zInit
		# zOrigin = deepcopy(zNew)
		zOrigin = deepcopy(zNew)
		trajectory = [zOrigin]
		dws = []

		for i in range(len(self.RRT.pathReversed)-1):
			dw = np.random.uniform(0,1)
			# print 'dw: ' + str(dw)
			dx = velocity*cos(zNew.theta)*self.RRT.dt
			# print 'dx: ' + str(dx)
			dy = velocity*sin(zNew.theta)*self.RRT.dt
			# print 'dy: ' + str(dy)

			# trackPoint, trackPointIndex = self.getNN(zNew, path = True)
			# print 'trackPoint ' + str(trackPoint.show())
			# print 'zNew ' + str(zNew.show())
			# xDistance = trackPoint.x-zNew.x
			# print 'xDistance: ' + str(xDistance)
			# yDistance = trackPoint.y-zNew.y
			# print 'yDistance: ' + str(yDistance)
			# L = np.sqrt(xDistance**2 + yDistance**2)
			# alpha1 = atan2(yDistance,xDistance)
			# print 'alpha1: ' + str(alpha1)
			# if alpha1<0:
			# 	alpha2 = alpha1+(2*pi) - zNew.theta
			# else:
			# 	alpha2 = alpha1 - zNew.theta
			# print 'alpha2: ' +str(sin(alpha2))
			# omega = 2*velocity*sin(alpha1)/L
			# print 'omega: ' + str(omega)			
			# delta = atan(omega*wheelBase/velocity)
			# print 'delta: ' + str(delta)
			# delta = delta*steeringRatio*180./np.pi
			# delta = -delta

			dtheta = (1/r)*((self.RRT.controls[i]*self.RRT.dt)+self.alpha*dw)

			zNew = Vertex(trajectory[-1].x+dx,trajectory[-1].y+dy,i,theta=trajectory[-1].theta+dtheta)
			# print 'zNew: ' + str(zNew.show())
			# print 'dtheta: ' + str(dtheta)
			trajectory.append(zNew)
			dws.append(dw)
			# print 'trajectory: ' + str([t.show() for t in trajectory])
		dws.append(0)

		return trajectory, dws		

	def generateTrajectories(self):

		self.trajectories = []
		self.dws = []
		for i in range(self.M):
			trajectory, dws = self.generateTrajectory(numSteps=10, dt=0.5, alpha=0.25, velocity=10, wheelBase=2.0, steeringRatio = 1, r = 0.5)
			self.trajectories.append(trajectory)
			self.dws.append(dws)

	def generateTrajectories2(self):
		
		self.trajectories = []
		for i in range(self.M):
			print 'doing rrt'
			newRRT = RRT(self.RRT.vInit,self.RRT.vGoal,self.dt, self.velocity, self.wheelBase, self.steeringRatio, self.alpha, self.r,self.plotStore)
			newRRT.extractPath()
			self.trajectories.append(newRRT.pathReversed)
			self.allRRTVertices.extend(newRRT.vertices)
			self.sampledPoints.extend(self.RRT.sampledPoints)
			print 'len of allRRTVertices is ' + str(len(self.allRRTVertices))

	def computeVariation(self):

		S = np.zeros(len(self.trajectories))
		for k, trajectory in enumerate(self.trajectories):
			dws = self.dws[k]
			trajectoryCost = 0
			for i,v in enumerate(trajectory):
				trajectoryCost += ((v.x-self.RRT.pathReversed[i].x)**2+(v.y-self.RRT.pathReversed[i].y)**2)*self.RRT.dt
				trajectoryCost += 0.5*(self.RRT.controls[i]**2)*self.RRT.dt
				trajectoryCost += self.alpha*(self.RRT.controls[i]*dws[i])
			S[k] = trajectoryCost

		variation = 0
		for k, trajectory in enumerate(self.trajectories):
			 pK = exp(-fabs(self.p)*S[k])/np.mean(S) 
			 variation += self.alpha*pK

		return variation

	def contructStatesMatrix(self,path):

		if len(path) == 2:			
			vertices = [v for v in path]
			states = np.zeros((4,5))
			states[0,0] = vertices[0].x
			states[0,1] = vertices[0].y
			states[0,2] = vertices[0].theta
			states[0,3] = 0
			states[0,4] = vertices[0].controlInput
			states[3,0] = vertices[1].x
			states[3,1] = vertices[1].y
			states[3,2] = vertices[1].theta
			states[3,3] = self.RRT.dt*3
			states[3,4] = vertices[1].controlInput
			states[1:3,0] = np.linspace(states[0,0], states[3,0], num=4)[1:3]
			states[1:3,1] = np.linspace(states[0,1], states[3,1], num=4)[1:3]
			states[1:3,2] = np.linspace(states[0,2], states[3,2], num=4)[1:3]
			states[1:3,3] = self.RRT.dt*np.array([1,2])
			states[1:3,4] = np.linspace(states[0,4], states[3,4], num=4)[1:3]

		if len(path) == 3:
			vertices = [v for v in path]
			if self.RRT.getDistance(vertices[0],vertices[1]) > self.RRT.getDistance(vertices[0],vertices[2]):
				if self.RRT.getDistance(vertices[0],vertices[1]) > self.RRT.getDistance(vertices[1],vertices[2]):
					vertices = [vertices[0],vertices[1]]
				else:
					vertices = [vertices[1],vertices[2]]
			elif self.RRT.getDistance(vertices[0],vertices[2]) > self.RRT.getDistance(vertices[1],vertices[2]):
				vertices = [vertices[0],vertices[2]]
			else:
				vertices = [vertices[1],vertices[2]]

			states = np.zeros((4,5))
			states[0,0] = vertices[0].x
			states[0,1] = vertices[0].y
			states[0,2] = vertices[0].theta
			states[0,3] = 0
			states[0,4] = vertices[0].controlInput
			states[3,0] = vertices[1].x
			states[3,1] = vertices[1].y
			states[3,2] = vertices[1].theta
			states[3,3] = self.RRT.dt*3
			states[3,4] = vertices[1].controlInput
			states[1:3,0] = np.linspace(states[0,0], states[3,0], num=4)[1:3]
			states[1:3,1] = np.linspace(states[0,1], states[3,1], num=4)[1:3]
			states[1:3,2] = np.linspace(states[0,2], states[3,2], num=4)[1:3]
			states[1:3,3] = self.RRT.dt*np.array([1,2])
			states[1:3,4] = np.linspace(states[0,4], states[3,4], num=4)[1:3]

		else:
			states = np.zeros((len(path),5))
			for i,v in enumerate(path):
				states[i,0] = v.x
				states[i,1] = v.y
				states[i,2] = v.theta
				states[i,3] = self.RRT.dt*i
				states[i,4] = v.controlInput

		return states

# 		if len(self.RRT.pathReversed) == 2:			
# 			vertices = [v for v in self.RRT.pathReversed]
# 			rrtStates = np.zeros((4,5))
# 			rrtStates[0,0] = vertices[0].x
# 			rrtStates[0,1] = vertices[0].y
# 			rrtStates[0,2] = vertices[0].theta
# 			rrtStates[0,3] = 0
# 			rrtStates[0,4] = vertices[0].controlInput
# 			rrtStates[3,0] = vertices[1].x
# 			rrtStates[3,1] = vertices[1].y
# 			rrtStates[3,2] = vertices[1].theta
# 			rrtStates[3,3] = self.RRT.dt*3
# 			rrtStates[3,4] = vertices[1].controlInput
# 			rrtStates[1:3,0] = np.linspace(rrtStates[0,0], rrtStates[3,0], num=4)[1:3]
# 			rrtStates[1:3,1] = np.linspace(rrtStates[0,1], rrtStates[3,1], num=4)[1:3]
# 			rrtStates[1:3,2] = np.linspace(rrtStates[0,2], rrtStates[3,2], num=4)[1:3]
# 			rrtStates[1:3,3] = self.RRT.dt*np.array([1,2])
# 			rrtStates[1:3,4] = np.linspace(rrtStates[0,4], rrtStates[3,4], num=4)[1:3]
# 			# p = np.poly1d(np.polyfit([rrtStates[0,0],rrtStates[3,0]], [rrtStates[0,1],rrtStates[3,1]], 1))
# 			# m = (rrtStates[3,1]-rrtStates[0,1])/(rrtStates[3,0]-rrtStates[0,0])
# 			# for i in range(1,3):
# 			# 	rrtStates[i,0] = v.x
# 			# 	rrtStates[i,1] = v.y
# 			# 	rrtStates[i,2] = v.theta
# 			# 	rrtStates[i,3] = self.RRT.dt*i
# 			# 	rrtStates[i,4] = v.controlInput
# 		if len(self.RRT.pathReversed) == 3:
# 			vertices = [v for v in self.RRT.pathReversed]
# 			if self.RRT.getDistance(vertices[0],vertices[1]) > self.RRT.getDistance(vertices[0],vertices[2]):
# 				if self.RRT.getDistance(vertices[0],vertices[1]) > self.RRT.getDistance(vertices[1],vertices[2]):
# 					vertices = [vertices[0],vertices[1]]
# 				else:
# 					vertices = [vertices[1],vertices[2]]
# 			elif self.RRT.getDistance(vertices[0],vertices[2]) > self.RRT.getDistance(vertices[1],vertices[2]):
# 				vertices = [vertices[0],vertices[2]]
# 			else:
# 				vertices = [vertices[1],vertices[2]]

# 			rrtStates = np.zeros((4,5))
# 			rrtStates[0,0] = vertices[0].x
# 			rrtStates[0,1] = vertices[0].y
# 			rrtStates[0,2] = vertices[0].theta
# 			rrtStates[0,3] = 0
# 			rrtStates[0,4] = vertices[0].controlInput
# 			rrtStates[3,0] = vertices[1].x
# 			rrtStates[3,1] = vertices[1].y
# 			rrtStates[3,2] = vertices[1].theta
# 			rrtStates[3,3] = self.RRT.dt*3
# 			rrtStates[3,4] = vertices[1].controlInput
# 			rrtStates[1:3,0] = np.linspace(rrtStates[0,0], rrtStates[3,0], num=4)[1:3]
# 			rrtStates[1:3,1] = np.linspace(rrtStates[0,1], rrtStates[3,1], num=4)[1:3]
# 			rrtStates[1:3,2] = np.linspace(rrtStates[0,2], rrtStates[3,2], num=4)[1:3]
# 			rrtStates[1:3,3] = self.RRT.dt*np.array([1,2])
# 			rrtStates[1:3,4] = np.linspace(rrtStates[0,4], rrtStates[3,4], num=4)[1:3]

# 		else:
# 			rrtStates = np.zeros((len(self.RRT.pathReversed),5))
# 			for i,v in enumerate(self.RRT.pathReversed):
# 				rrtStates[i,0] = v.x
# 				rrtStates[i,1] = v.y
# 				rrtStates[i,2] = v.theta
# 				rrtStates[i,3] = self.RRT.dt*i
# 				rrtStates[i,4] = v.controlInput

	def computeVariation2(self):

		startTime = time.time()

		rrtStates = self.contructStatesMatrix(self.RRT.pathReversed)

		# try:		
		controlSpline = UnivariateSpline(rrtStates[:,3], rrtStates[:,4])
		# except:
		# 	print len(self.RRT.pathReversed)
		# 	sys.exit()
		totalCosts = np.zeros(len(self.trajectories))
		
		trajectoryLengths = [len(t) for t in self.trajectories]
		print 'max trajectory length is ' + str(max(trajectoryLengths))
		print 'min trajectory length is ' + str(min(trajectoryLengths))
		if max(trajectoryLengths) < 4:
			print 'max(trajectoryLengths) < 4'
			trajectoryStates = np.zeros((len(self.trajectories),4,5))
			print trajectoryStates.shape
		else:
			print 'nothing'
			trajectoryStates = np.zeros((len(self.trajectories),max(trajectoryLengths),5))
			print trajectoryStates.shape
		if min(trajectoryLengths) < 4:
			print 'min(trajectoryLengths) < 4'
			trajectoryLengths = [4 for t in self.trajectories]
		# else:
		# 	trajectoryStates = np.zeros((len(self.trajectories),max(trajectoryLengths),5))

		# print trajectoryStates.shape

		for k, trajectory in enumerate(self.trajectories):	
			# print 'trajectory ' + str(k)
			try:		
				trajectoryStates[k,:len(trajectory),:] = self.contructStatesMatrix(trajectory)
			except:
				print trajectoryStates[k,:len(trajectory),:].shape
				print self.contructStatesMatrix(trajectory).shape
				print len(trajectory)
			# for i,v in enumerate(trajectory):
				# trajectoryStates[k,i,0] = v.x
				# trajectoryStates[k,i,1] = v.y
				# trajectoryStates[k,i,2] = v.theta
				# trajectoryStates[k,i,3] = self.RRT.dt*i
				# trajectoryStates[k,i,4] = v.controlInput
			# print 'trajectoryStates ' + str(trajectoryStates[k])		
			pathCosts = np.zeros(len(trajectory))
			noiseCosts = np.zeros(len(trajectory))
			for i,v in enumerate(trajectory[:-1]):
				pathCosts[i] += trajectoryStates[k,i,0:3].dot(self.Q).dot(trajectoryStates[k,i,0:3].T)
				pathCosts[i] += 0.5*controlSpline(trajectoryStates[k,i,3])*self.R*controlSpline(trajectoryStates[k,i,3])
				noiseCosts[i] += controlSpline(trajectoryStates[k,i,3])*self.alpha*trajectoryStates[k,i,4]/sqrt(self.RRT.dt)
			# print 'pathCosts: ' + str(pathCosts.shape)
			# print 'noiseCosts: ' + str(noiseCosts.shape)
			# totalCost = np.trapz(trajectoryStates[k,:len(trajectory)-1,3], pathCosts[:-1])
			totalCost = np.trapz(pathCosts[:-1], trajectoryStates[k,:len(trajectory)-1,3])
			# print totalCost
			endStateDiff = trajectoryStates[k,-1,0:3]-np.array(self.RRT.vGoal.getState()[0:3])
			totalCost += endStateDiff.dot(self.Qf).dot(endStateDiff.T)
			# print totalCost
			# totalCost += np.trapz(trajectoryStates[k,:len(trajectory)-1,3], noiseCosts[:-1])
			totalCost += np.trapz(noiseCosts[:-1], trajectoryStates[k,:len(trajectory)-1,3])
			# print totalCost
			# if totalCost<0
			totalCosts[k] = totalCost/self.regCoef

		# print 'totalCosts: ' + str(totalCosts)
		# print 'Lambda: ' + str(self.Lambda)

		trajectoryDesirability = np.exp(-totalCosts/self.Lambda)
		print 'trajectoryDesirability: ' + str(trajectoryDesirability)
		# print sum(trajectoryDesirability)
		print 'computed variation in ' + str(time.time()-startTime) + ' s'

		weights = trajectoryDesirability/sum(trajectoryDesirability)
		# print weights

		dU = np.zeros(min(trajectoryLengths))
		# print 'dU: ' + str(dU)
		for k, trajectory in enumerate(self.trajectories):
			dU += weights[k]*self.alpha*trajectoryStates[k,:min(trajectoryLengths),4]/sqrt(self.RRT.dt)
			# print 'dU: ' + str(dU)

		U = np.zeros(min(trajectoryLengths))
		t = np.zeros(min(trajectoryLengths))
		print 'min trajectory length is ' + str(min(trajectoryLengths))
		for i in range(min(trajectoryLengths)):
			U[i] = controlSpline(self.RRT.dt*i) + dU[i]
			t[i] = self.RRT.dt*i

		return U,t

	def computeVariationGPU(self):

		S = np.zeros(len(self.trajectories))
		a_gpu = gpuarray.to_gpu(S.astype(np.float32))
		for k, trajectory in enumerate(self.trajectories):
			dws = self.dws[k]
			trajectoryCost = 0
			for i,v in enumerate(trajectory):
				trajectoryCost += ((v.x-self.RRT.pathReversed[i].x)**2+(v.y-self.RRT.pathReversed[i].y)**2)*self.RRT.dt
				trajectoryCost += 0.5*(self.RRT.controls[i]**2)*self.RRT.dt
				trajectoryCost += self.alpha*(self.RRT.controls[i]*dws[i])
				a_gpu[i] = np.asarray(trajectoryCost).astype(np.float32)

		variation = 0
		for k, trajectory in enumerate(self.trajectories):
			 pK = GPUexp(-fabs(self.p)*a_gpu[k])/(pycuda.gpuarray.sum(a_gpu)/a_gpu.shape[0])
			 variation += self.alpha*pK

		return variation

	def computeVariation(self):

		S = np.zeros(len(self.trajectories))
		Qf = np.fill_diagonal(np.zeros((3, 3)), 10)

		for k, trajectory in enumerate(self.trajectories):
			dws = self.dws[k]			
			goalDistance = np.array(trajectory[-1].getState()-self.RRT.zGoal.getState())
			trajectoryCost = np.matrix.transpose(goalDistance)*Qf*goalDistance
			for i,v in enumerate(trajectory):
				trajectoryCost += ((v.x-self.RRT.pathReversed[i].x)**2+(v.y-self.RRT.pathReversed[i].y)**2)*self.RRT.dt
				trajectoryCost += 0.5*(self.RRT.controls[i]**2)*self.RRT.dt
				trajectoryCost += self.alpha*(self.RRT.controls[i]*dws[i])
			S[k] = trajectoryCost

		variation = 0
		for k, trajectory in enumerate(self.trajectories):
			 pK = exp(-fabs(self.p)*S[k])/np.mean(S) 
			 variation += self.alpha*pK

		return variation

	def executeControl(self):	
		times = range(len(self.RRT.controls))		
		variation = self.computeVariation()
		self.RRT.controls += variation
		# print self.RRT.controls
		spline = UnivariateSpline(times, self.RRT.controls)
		states = self.RRT.runKinematicModelControls(self.RRT.zInit.getState(), numSteps=10, dt=self.RRT.dt, spline=spline, velocity=self.RRT.velocity, wheelBase=self.RRT.wheelBase, steeringRatio=self.RRT.steeringRatio)
		return states

	def executeControl2(self,U,t):	

		print t
		print U
		controlSpline = UnivariateSpline(t, U)		

		for i in range(int(self.tHorizon/self.RRT.dt)):				
			dx = self.RRT.velocity*cos(self.path[-1].theta)
			dy = self.RRT.velocity*sin(self.path[-1].theta)			
			dtheta = (1/self.r)*controlSpline(self.RRT.dt*i)
			randomOffset = np.random.randn()*10
			dtheta += (1/self.r)*self.alpha*sqrt(self.RRT.dt)*randomOffset
			self.path.append(Vertex(self.path[-1].x+self.dt*dx,self.path[-1].y+self.dt*dy,self.path[-1].theta+self.dt*dtheta))
			self.plotStore.path = self.path

	def run(self):
		self.runRRT()
		print 'RRT run'
		self.generateTrajectories2()
		self.executeControl2(*self.computeVariation2())
		print 'new pi_rrt point: ' + str(self.path[-1].getState())		
import os, errno
import sys
import time
from copy import deepcopy
import cPickle, dill
from subprocess import call

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.pyplot import plot, scatter, savefig
import matplotlib.pylab as pylab
params = {'legend.fontsize': 'xx-large',
          'figure.figsize': (15, 5),
         'axes.labelsize': 'xx-large',
         'axes.titlesize':'xx-large',
         'xtick.labelsize':'xx-large',
         'ytick.labelsize':'xx-large'}
pylab.rcParams.update(params)
import numpy as np

from Vertex import Vertex
from RRT import RRT
from RRTStar import RRTStar
from PI_RRT import PI_RRT
from plotStore import plotStore

def plotPath(path):
    plt.plot([v.x for v in path], [v.y for v in path], '-b')


def scatterPath(path):
    scatter([v.x for v in path], [v.y for v in path], '-b')

def plotStates(states):
    for s in states:
        plt.plot(s[:,0],s[:,1])

def plotRRTVertices(RRT):
    scatter([v.x for v in RRT.vertices], [v.y for v in RRT.vertices],color='blue')
    scatter(RRT.vGoal.x, RRT.vGoal.y, c = 'g')
    plotObstacles(RRT)

def saveRRTVerticesPlot(RRT,saveDir):
    fig = plt.figure(figsize=(20,20))  
    plt.axis('equal')  
    plt.grid(1)
#     plt.ion()
    plt.title('Sampling-based path planning using stochastic optimal control',fontsize=20)    
    scatter(RRT.vInit.x, RRT.vInit.y, c = 'r')     
    scatter(RRT.vGoal.x, RRT.vGoal.y, c = 'g')
    print len(RRT.vertices)
    plt.legend(['start', 'goal'], loc=3)
    plotObstacles(RRT)
    for i,v in enumerate(RRT.vertices[1:]):   
        print i
        scatter(v.x, v.y ,color='blue')  
        savefig(saveDir+str(i)+'.png')
#         disp.display(gcf())
#         disp.clear_output(wait=True)

def saveRRTPathPlot(RRT,saveDir):
    plotObstacles(RRT)
    scatter(RRT.vInit.x, RRT.vInit.y, c = 'r')     
    scatter(RRT.vGoal.x, RRT.vGoal.y, c = 'g')
    for i,v in enumerate(RRT.pathReversed):
        scatter(v.x, v.y ,color='blue')        
        savefig(saveDir+str(i)+'.png')

def plotTrajectoryVertices(trajectory):
    scatter([v.x for v in trajectory], [v.y for v in trajectory],color='blue')
    scatter(trajectory[-1].x,trajectory[-1].y,color='red')
    scatter(trajectory[0].x,trajectory[0].y,color='green')

def plotAllVertices(PI_RRT):
    for trajectory in PI_RRT.trajectories:
        plt.plot([v.x for v in trajectory], [v.y for v in trajectory])
        scatter(trajectory[-1].x,trajectory[-1].y)
        scatter(trajectory[0].x,trajectory[0].y)
    plt.plot([v.x for v in PI_RRT.RRT.path], [v.y for v in PI_RRT.RRT.path],color='r')
#     plt.plot(states[:,0],states[:,1],color='g')

def plotTree(RRT):    
    plt.ion()
    scatter(RRT.vertices[0].x, RRT.vertices[0].y, c = 'r')
#     scatter(RRT.vGoal.x, RRT.vGoal.y, c = 'g')
    plotObstacles(RRT)
#     scatter([v.x for v in RRT.vertices],[v.y for v in RRT.vertices])
    for i,v in enumerate(RRT.vertices):
        v_p = RRT.vertices[v.parent]
        plot([v_p.x, v.x], [v_p.y, v.y],'b')  
        plt.grid(True)  
        disp.display(gcf())
        disp.clear_output(wait=True)

def plotObstacles(RRT):
    for obstacle in RRT.obstacles:    
        x = []
        y = []
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
        obstaclePlot = plot(x,y,'r')

# Variables
vInit = Vertex(-9.,0.,0.,0.,0.,0)
vGoal = Vertex(9.,0.,0.,10.,0.,0)
alphas = [0.25,0.5,1.0]
alpha = alphas[0]
obstacleTypes = ['single', 'double']
obstacleType = obstacleTypes[0]
runTypes = ['rrt','rrtloop','pirrt']
runType = runTypes[2]
useRRTStar = False
controlledSteering = True
plottingInterval='notend'

# Create directory to save files
saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/March20_'+obstacleType+'_obstacle_alpha_'+str(alpha)+'_controlledSteering_generateTrajectories2/'
try:
    os.makedirs(saveDir)
except OSError as e:
    if e.errno != errno.EEXIST:
        raise
call('rm '+saveDir+'*',shell=True)

if runType == 'rrt':
	pS = plotStore(vInit,vGoal,saveDir)
	rrt = RRT(vInit,vGoal,alpha=alpha,plotStore=pS,controlledSteering=controlledSteering,plottingInterval=plottingInterval,obstacleType=obstacleType)
	rrt.extractPath()

if runType == 'rrtloop':
    if useRRTStar:
        for o in obstacleTypes:
            for a in alphas:
                pS = plotStore(vInit,vGoal,saveDir)
                rrt = RRTStar(vInit,vGoal,alpha=a,plotStore=pS,plottingInterval=plottingInterval,obstacleType=o)
                rrt.extractPath()
    else:
        for o in obstacleTypes:
            for a in alphas:
                pS = plotStore(vInit,vGoal,saveDir)
                for i in range(100):                    
                    rrt = RRT(vInit,vGoal,alpha=a,plotStore=pS,plottingInterval=plottingInterval,obstacleType=o)
                    rrt.extractPath()

if runType == 'pirrt':
    pi_rrt = PI_RRT(vInit,vGoal,alpha,saveDir,useRRTStar,controlledSteering,obstacleType)
    # pi_rrt.plottingInterval = 'notend'
    # pi_rrt.generateTrajectoriesMP()
    pirrtTimes = []
    i = 0
    while pi_rrt.reachedGoal(pi_rrt.path[-1]) is False:
        # if i == 1:
        #     sys.exit()
        startTime = time.time()    
        print 'COUNT '+ str(i)
        if pi_rrt.runRRT():
            # if i == 1:
            #     pi_rrt.plottingInterval = 'notend'
            # if pi_rrt.generateTrajectories3():
            if pi_rrt.generateTrajectories2():
                pi_rrt.executeControl2(*pi_rrt.computeVariation2())
                print 'pirrt iteration completed in ' + str(time.time()-startTime) + ' s'
                pirrtTimes.append(time.time()-startTime)
                print 'average pirrt iteration time is ' + str(sum(pirrtTimes)/len(pirrtTimes)) + ' s'
                i += 1
                print pi_rrt.path[-1].getState()
        pi_rrt.RRT.plotAll()
        pi_rrt.plotStore.RRTpaths = []
        #     else:
        #         del pi_rrt.path[-1]
        # else:
        #     del pi_rrt.path[-1]
        dill.dump(pi_rrt,open(saveDir+'PI_RRT_'+str(i)+'.p','wb'))

    pi_rrt.RRT.plotAll()
# rc = call(".//home/ankit/Documents/Thesis/createVideo.sh",shell=True)    

# for obstacleType in obstacleTypes:
#     pls = plotStore(vInit,vGoal,saveDir)
#     fig, ax = plt.subplots()
#     fig.set_size_inches(20.0, 20.0)
#     ax.set_title('Sampling-based path planning using stochastic optimal control',fontsize=20)
#     ax.axis('equal')  
#     ax.grid()    
#     for i,alpha in enumerate(alphas):
#         # rrt = RRT(vInit,vGoal,alpha=alpha,plotStore=pls,plottingInterval='notend',obstacleType=obstacleType)
#         rrt = RRTStar(vInit,vGoal,alpha=alpha,plotStore=pls,plottingInterval='notend',obstacleType=obstacleType)        
#         rrt.extractPath()
#         ax = rrt.returnPlot(ax,i+1)
#         cPickle.dump(rrt,open(saveDir+'RRT_'+str(alpha)+'_'+obstacleType+'.p','wb'))
#     patch1 = mpatches.Patch(color='#ff1493', label='Alpha = 0.25')    
#     patch2 = mpatches.Patch(color='#0000ff', label='Alpha = 0.5')    
#     patch3 = mpatches.Patch(color='#000000', label='Alpha = 1.0') 
#     ax.legend(handles=[patch1,patch2,patch3])   
#     # ax = plt.gca().add_artist(path_legend)
#     fig.savefig(saveDir+'RRT_'+obstacleType+'.png')
#     ax.clear()

# from contextlib import contextmanager
# import sys, os

# @contextmanager
# def suppress_stdout():
#     with open(os.devnull, "w") as devnull:
#         old_stdout = sys.stdout
#         sys.stdout = devnull
#         try:  
#             yield
#         finally:
#             sys.stdout = old_stdout

# RRTcompletionIterations = []
# RRTcompletionTimes = []
# for obstacleType in obstacleTypes: 
#     for i,alpha in enumerate(alphas):
#         pls = plotStore(vInit,vGoal,saveDir)
#         with suppress_stdout():
#             for i in range(10):
                # rrt = RRT(vInit,vGoal,alpha=alpha,plotStore=pls,plottingInterval='notend',obstacleType=obstacleType)
                # rrt = RRTStar(vInit,vGoal,alpha=alpha,plotStore=pls,plottingInterval='notend',obstacleType=obstacleType)        
                # rrt.extractPath()
                # print pls.RRTcompletionIterations
                # sys.exit()
        # RRTcompletionIterations.append(np.mean(pls.RRTcompletionIterations))
        # RRTcompletionTimes.append(np.mean(pls.RRTcompletionTimes))
        # cPickle.dump(pls,open(saveDir+'plotStore_'+str(alpha)+'_'+obstacleType+'.p','wb'))
#         print RRTcompletionIterations
#         print RRTcompletionTimes
# print RRTcompletionIterations
# print RRTcompletionTimes 


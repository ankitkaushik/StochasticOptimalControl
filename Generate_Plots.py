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
# vInit = Vertex(-6.701104461907802, 0.03323463812282749, 0.023855731871954694, 4.500000000000001, -0.01316966949165381, -1)
# vInit = Vertex(-6.705127720699081, 0.10337839256046491, 0.019522340268920083, 4.500000000000001, 0.039278823874604928, -1)
#Fail for double
# vInit = Vertex(-5.003824576201526, -0.013088832596684832, 0.053600875067695924, 0.0, 0.0, -1)

# vInit = Vertex(3.556400689218416, 1.1122687085435525, 0.27198546370193616, 0.0, 0.0, -1)
# vInit = Vertex(-2.322802078480477, -1.5682812432844735, -0.43168166012273512, 0.0, 0.0, -1)
# Latest fail on XPS
# vInit = Vertex(2.333759289549664, -1.0325296926557328, -0.35328948476828181, 0.0, 0.0, -1)

# v = vInit
# v = Vertex(-2.2,1.25, -0.35328948476828181, 0.0, 0.0, -1)
# vInit = Vertex(-4.405433256434646, 0.1731660975960875, 0.15843846552639818, 0.0, 0.0, -1)
# vInit = Vertex(-2.3075439748551645, -1.2146455385001258, -0.12745521375963603, 0.0, 0.0, -1)
# vInit = Vertex(-2.1438493535428647, -0.6990585839364634, -0.023160198189374023, 0.0, 0.0, -1)
# vInit = Vertex(-3.274496147453313, -0.4380171488751353, -0.072003545741959094, 0.0, 0.0, -1)
# vInit = Vertex(-2.8, -1.2, -0.072003545741959094, 0.0, 0.0, -1)
# vInit = Vertex(-2.1230469516087718, -0.28321651336500553, 0.093621076239903567, 0.0, 0.0, -1)
# vInit = Vertex(3.5579065654977633, -0.19329584063790506, -0.025624273325923833, 0.0, 0.0, -1)
# vInit = Vertex(-2.1230469516087718, -0.28321651336500553, 0.093621076239903567, 0.0, 0.0, -1)
# vInit = Vertex(3.5579065654977633, -0.19329584063790506, -0.025624273325923833, 0.0, 0.0, -1)
# vInit = Vertex(-2.22621023679345, -1.0965525803342375, 0.095902618444505039, 0.0, 0.0, -1)
# vInit = Vertex(2.191991582749325, 1.0115514363335742, 0.62800887258853244, 0.0, 0.0, -1)
# vInit = Vertex(-7.,2.,0.,0.,0.,0)
# vInit = Vertex(-7.,-2.,0.,0.,0.,0)
# vInit = Vertex(-5.,0.,0.,0.,0.,0)
vGoal = Vertex(9.,0.,0.,10.,0.,0)

alphas = [0.25,0.5,1.0]
obstacleTypes = ['single', 'double']
runTypes = ['rrt','pirrt']
runType = runTypes[1]
useRRTStar = False

# Create directory to save files

# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/reportImages_RRT_steer_uncontrolled/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/reportImages_RRT_steer_controlled/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/reportImages_RRTStar_steer_uncontrolled/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/reportImages_RRTStar_steer_controlled/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_single_obstacle_alpha_0.25_with_noise/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_single_obstacle_alpha_0.5_with_noise/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_single_obstacle_alpha_1.0_with_noise/'
saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_double_obstacle_alpha_0.25_with_noise_2/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_double_obstacle_alpha_0.5_with_noise/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_double_obstacle_alpha_1.0_with_noise/'

# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/FINAL_RRT_2/'
# saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/dummy/'

try:
    os.makedirs(saveDir)
except OSError as e:
    if e.errno != errno.EEXIST:
        raise
call('rm '+saveDir+'*',shell=True)

# pS = plotStore(vInit,vGoal,saveDir)
# rrt = RRT(vInit,vGoal,alpha=0.25,plotStore=pS,plottingInterval='end',obstacleType='double')
# rrt.extractPath()
# sys.exit()

if runType == 'rrt':
    if useRRTStar:
        for o in obstacleTypes:
            for a in alphas:
                pS = plotStore(vInit,vGoal,saveDir)
                rrt = RRTStar(vInit,vGoal,alpha=a,plotStore=pS,plottingInterval='end',obstacleType=o)
                rrt.extractPath()
    else:
        for o in obstacleTypes:
            for a in alphas:
                pS = plotStore(vInit,vGoal,saveDir)
                for i in range(100):                    
                    rrt = RRT(vInit,vGoal,alpha=a,plotStore=pS,plottingInterval='end',obstacleType=o)
                    rrt.extractPath()

if runType == 'pirrt':
    pi_rrt = PI_RRT(vInit,vGoal,alphas[0],saveDir,obstacleType = 'double')
    # pi_rrt.generateTrajectoriesMP()
    pirrtTimes = []
    i = 0
    while pi_rrt.reachedGoal(pi_rrt.path[-1]) is False:
        print pi_rrt.path[-1].getState()
        startTime = time.time()    
        print 'COUNT '+ str(i)
        if pi_rrt.runRRT():
            if pi_rrt.generateTrajectories2():
                pi_rrt.executeControl2(*pi_rrt.computeVariation2())
                print 'pirrt iteration completed in ' + str(time.time()-startTime) + ' s'
                pirrtTimes.append(time.time()-startTime)
                print 'average pirrt iteration time is ' + str(sum(pirrtTimes)/len(pirrtTimes)) + ' s'
                i += 1
                print pi_rrt.path[-1].getState()
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


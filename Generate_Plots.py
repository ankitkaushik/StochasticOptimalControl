from copy import deepcopy
import cPickle, dill
import os, errno
from subprocess import call
import sys
import time, datetime
import yaml

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

from PI_RRT import PI_RRT
from plotStore import plotStore
from RRT import RRT
from RRTStar import RRTStar
from Vertex import Vertex

# Load variables
with open('variables.yaml') as f:
    variables = yaml.load(f)

# Create directory to save files
saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/'+datetime.datetime.now().strftime("%m_%d_%y_%H_%M_%S")+'_'+\
            variables['obstacleType']+'_obstacle_alpha_'+str(variables['alpha'])+'/'

try:
    os.makedirs(saveDir)
except OSError as e:
    if e.errno != errno.EEXIST:
        raise
# call('rm '+saveDir+'*',shell=True)
variables['saveDir'] = saveDir

plotStore = plotStore(Vertex(*variables['vInit']),Vertex(*variables['vGoal']),saveDir)

for i in range(10):
    if variables['runType'] == 'rrt':
        if variables['useRRTStar']: 
            rrt = RRTStar(variables,plotStore)
        else:
            rrt = RRT(variables,plotStore)
        rrt.extractPath()
        # dill.dump(rrt,open(saveDir+'RRT_'+str(i)+'.p','wb'))

if variables['runType'] == 'rrtloop':
    if variables['useRRTStar']:
        for o in variables['obstacleTypes']:
            for a in variables['alphas']:
                rrt = RRTStar(variables,plotStore)
                rrt.extractPath()
    else:
        for o in variables['obstacleTypes']:
            for a in variables['alphas']:
                for i in range(100):                    
                    rrt = RRT(vInit,vGoal,alpha=a,plotStore=pS,plottingInterval=plottingInterval,obstacleType=o)
                    rrt.extractPath()

if variables['runType'] == 'pirrt':
    pi_rrt = PI_RRT(variables,plotStore)
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

            #Turning on plotting at regular after intervals after the first RRT completes
            # pi_rrt.plottingInterval = 'notend'
            # pi_rrt.variables['plottingInterval'] = 'notend'
            
            if pi_rrt.generateTrajectories():
                pi_rrt.executeControl(*pi_rrt.computeVariation2())
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

# Testing summary
print 'max(plotStore.RRTcompletionIterations): ' + str(max(plotStore.RRTcompletionIterations))
print 'min(plotStore.RRTcompletionIterations): ' + str(min(plotStore.RRTcompletionIterations))
print 'mean(plotStore.RRTcompletionIterations): ' + str(np.mean(plotStore.RRTcompletionIterations))
print 'max(plotStore.RRTcompletionTimes): ' + str(max(plotStore.RRTcompletionTimes))
print 'min(plotStore.RRTcompletionTimes): ' + str(min(plotStore.RRTcompletionTimes))
print 'mean(plotStore.RRTcompletionTimes): ' + str(np.mean(plotStore.RRTcompletionTimes))


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
import os, errno
import time
from copy import deepcopy
import cPickle
from subprocess import call

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.pyplot import plot, scatter, savefig
import matplotlib.pylab as pylab
params = {'legend.fontsize': 'xx-large',
          'figure.figsize': (15, 5),
         'axes.labelsize': 'xx-large',
         'axes.titlesize':'xx-large',
         'xtick.labelsize':'xx-large',
         'ytick.labelsize':'xx-large'}
pylab.rcParams.update(params)

from Vertex import Vertex
from RRT import RRT
from RRTStar import RRTStar
from PI_RRT import PI_RRT
from plotStore import plotStore

def plotPath(path):
    plt.plot([v.x for v in path], [v.y for v in path], '-b')


# In[3]:


def scatterPath(path):
    scatter([v.x for v in path], [v.y for v in path], '-b')


# In[4]:


def plotStates(states):
    for s in states:
        plt.plot(s[:,0],s[:,1])


# In[5]:


def plotRRTVertices(RRT):
    scatter([v.x for v in RRT.vertices], [v.y for v in RRT.vertices],color='blue')
    scatter(RRT.vGoal.x, RRT.vGoal.y, c = 'g')
    plotObstacles(RRT)


# In[6]:


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


# In[7]:


def saveRRTPathPlot(RRT,saveDir):
    plotObstacles(RRT)
    scatter(RRT.vInit.x, RRT.vInit.y, c = 'r')     
    scatter(RRT.vGoal.x, RRT.vGoal.y, c = 'g')
    for i,v in enumerate(RRT.pathReversed):
        scatter(v.x, v.y ,color='blue')        
        savefig(saveDir+str(i)+'.png')


# In[8]:


def plotTrajectoryVertices(trajectory):
    scatter([v.x for v in trajectory], [v.y for v in trajectory],color='blue')
    scatter(trajectory[-1].x,trajectory[-1].y,color='red')
    scatter(trajectory[0].x,trajectory[0].y,color='green')


# In[9]:


def plotAllVertices(PI_RRT):
    for trajectory in PI_RRT.trajectories:
        plt.plot([v.x for v in trajectory], [v.y for v in trajectory])
        scatter(trajectory[-1].x,trajectory[-1].y)
        scatter(trajectory[0].x,trajectory[0].y)
    plt.plot([v.x for v in PI_RRT.RRT.path], [v.y for v in PI_RRT.RRT.path],color='r')
#     plt.plot(states[:,0],states[:,1],color='g')


# In[10]:


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


# In[11]:


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


# # In a loop, instantiate PI_RRT object which does the following:
# - Performs RRT with no obstacles
# - Generates trajectories around nominal RRT trajectory
# - Computes variation for control input from noise profiles of generated trajectories

# In[12]:


saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/debug5/'
print saveDir

try:
    os.makedirs(saveDir)
except OSError as e:
    if e.errno != errno.EEXIST:
        raise


# vInit = Vertex(-9.,0.,0.,0.,0.,0)
vInit = Vertex(-3.274496147453313, -0.4380171488751353, -0.072003545741959094, 0.0, 0.0, -1)
# vInit = Vertex(-2.8, -1.5, -0.072003545741959094, 0.0, 0.0, -1)
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
dt = 0.1
velocity = 5.0
wheelBase = 2.0
steeringRatio = 1
alpha = 0.25
r = 4.0

plotStore = plotStore(vInit,vGoal,saveDir)
# rrt = RRTStar(vInit,vGoal,dt,velocity,wheelBase,steeringRatio,alpha,r,plotStore,plottingInterval='end')
rrt = RRT(vInit,vGoal,dt,velocity,wheelBase,steeringRatio,alpha,r,plotStore,plottingInterval='notend')
rrt.createObstacles(obstacleType='single')
rrt.extractPath()

# fig = figure(figsize=(20,20))
# pi_rrt = PI_RRT(vInit,vGoal,saveDir,useRRTStar=False)
# pirrtTimes = []
# i = 0
# for i in range(20):
# while pi_rrt.reachedGoal(pi_rrt.path[-1]) is False:
#     startTime = time.time()    
#     print 'COUNT '+ str(i)
#     pi_rrt.runRRT()
#     pi_rrt.generateTrajectories2()
#     try:
#         pi_rrt.executeControl2(*pi_rrt.computeVariation2())
#         print 'pirrt iteration completed in ' + str(time.time()-startTime) + ' s'
#         pirrtTimes.append(time.time()-startTime)
#         print 'average pirrt iteration time is ' + str(sum(pirrtTimes)/len(pirrtTimes)) + ' s'
#         i += 1
#     except:
#         for v in pi_rrt.RRT.pathReversed:
#             pi_rrt.path.append(v)

    # fig = plt.figure(figsize=(20,20))
    # plt.title('Sampling-based path planning using stochastic optimal control',fontsize=20)    
    # for obstacle in pi_rrt.RRT.obstacles:    
    #     x = []
    #     y = []
    # #     print obstacle.center[0] - obstacle.size[0]/2
    #     x.extend([obstacle.center[0] - obstacle.size[0]/2])
    #     x.extend([obstacle.center[0] - obstacle.size[0]/2])
    #     x.extend([obstacle.center[0] + obstacle.size[0]/2])
    #     x.extend([obstacle.center[0] + obstacle.size[0]/2])
    #     x.extend([obstacle.center[0] - obstacle.size[0]/2])
    #     y.extend([obstacle.center[1] - obstacle.size[1]/2])
    #     y.extend([obstacle.center[1] + obstacle.size[1]/2])
    #     y.extend([obstacle.center[1] + obstacle.size[1]/2])
    #     y.extend([obstacle.center[1] - obstacle.size[1]/2])
    #     y.extend([obstacle.center[1] - obstacle.size[1]/2])
    #     obstaclePlot = plot(x,y,'r')
    # pirrtPathPlot = plotPath(pi_rrt.path)
    # rrtVerticesPlot = scatter([v.x for v in pi_rrt.allRRTVertices],[v.y for v in pi_rrt.allRRTVertices],c='cyan')
    # rrtSampledPointsPlot = scatter([v.x for v in pi_rrt.sampledPoints],[v.y for v in pi_rrt.sampledPoints],c='orange')
    # plt.legend([pirrtPathPlot,rrtVerticesPlot,rrtSampledPointsPlot], ['PIRRT Path','All RRT Vertices','All RRT Sampled Points'])
    # plt.grid()
    # savefig(saveDir+str(i)+'.png')

# rc = call(".//home/ankit/Documents/Thesis/createVideo.sh",shell=True)
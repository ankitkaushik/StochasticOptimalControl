import os
import cPickle
from Vertex import Vertex
from RRT import RRT
from RRTStar import RRTStar
from PI_RRT import PI_RRT
from plotStore import plotStore
import matplotlib.pyplot as plt

saveDir = '/'+'/'.join(os.getcwd().split('/')[1:-1])+'/debug/'
# print saveDir

# vInit = Vertex(-9.,0.,0.,0.,0.,0)
vInit = Vertex(-2.22621023679345, -1.0965525803342375, 0.095902618444505039, 0.0, 0.0, -1)
# vInit = Vertex(-3.274496147453313, -0.4380171488751353, -0.072003545741959094, 0.0, 0.0, -1)
# vInit = Vertex(-5.553923114739766, -0.1469038304917034, -0.07016107956316503, 0.0, 0.0, -1)
# vInit = Vertex(-5.553923114739766, -0.1469038304917034,-0.,0.0,0.0,-1)
vGoal = Vertex(9.,0.,0.,10.,0.,0)
dt = 0.1
velocity = 2.3
wheelBase = 2.0
steeringRatio = 1
alpha = 0.25
r = 4.0

plotStore = plotStore(vInit,vGoal,saveDir)

rrt = RRT(vInit,vGoal,dt,velocity,wheelBase,steeringRatio,alpha,r,plotStore)
# rrt.extractPath()
# print rrt.onObstacle(vInit)
# print rrt.onObstacle(Vertex(-1.9972671172882999, -1.074528774322324, 0.82037755769850584, 0.10000000000000001, 0.82037755769850584, 0))


# Testing if new obstacle free method works

# v1 = Vertex(-3.,1.,0.0,0.0,0.0,-1)
# v2 = Vertex(-1,-1,0.0,0.0,0.0,-1)
# v1 = Vertex(-2.25,3.,0.0,0.0,0.0,-1)
# v2 = Vertex(0,0.,0.0,0.0,0.0,-1)
# rrt = RRTStar(vInit,vGoal,dt,velocity,wheelBase,steeringRatio,alpha,r,plotStore)
print rrt.obstacleFree(vInit)
for obstacle in rrt.obstacles:    
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
    plt.plot(x,y,'r')
    plt.scatter(vInit.x,vInit.y)
    # plt.plot([v1.x,v2.x],[v1.y,v2.y],'b')
plt.show()
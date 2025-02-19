import time
import tabulate
import cPickle
import matplotlib.pyplot as plt

from Vertex import Vertex
from RRT import RRT
from PI_RRT import PI_RRT

def plotPath(path):
    plt.plot([v.x for v in path], [v.y for v in path], '-b')

vInit = Vertex(-9.,0.,0.,0.,0.,0)
vGoal = Vertex(9.,0.,0.,10.,0.,0)
dt = 0.1
velocity = 2.3
wheelBase = 2.0
steeringRatio = 1
alpha = 0.25
r = 4.0

# rrt = RRT(vInit, vGoal, dt, velocity, wheelBase, steeringRatio, alpha, r)
# rrt.extractPath()

pi_rrt = PI_RRT(vInit,vGoal)
numIterations = 10
print 'numIterations: ' + str(numIterations)
for i in range(numIterations):
    print 'COUNT '+ str(i)
    pi_rrt.runRRT()
    pi_rrt.generateTrajectories2()
    # try:
    pi_rrt.executeControl2(*pi_rrt.computeVariation2())
    # except:
    #     U,t = pi_rrt.computeVariation2()
    #     print U
    #     print t
    #     break

# cPickle.dump([v.getState() for v in pi_rrt.path],open('pi_rrt_path.p','wb'), protocol=cPickle.HIGHEST_PROTOCOL)
# for obstacle in pi_rrt.RRT.obstacles:
#     x = []
#     y = []
#     x.extend([o[0] for o in obstacle])
#     x.extend([obstacle[0][0]])
#     y.extend([o[1] for o in obstacle])
#     y.extend([obstacle[0][1]])
#     plt.plot(x,y)
#     plotPath(pi_rrt.path)
# plt.show()

cPickle.dump(pi_rrt.allRRTVertices,open('pi_rrt_allRRTVertices.p','wb'), protocol=cPickle.HIGHEST_PROTOCOL)
cPickle.dump(pi_rrt.sampledPoints,open('pi_rrt_sampledPoints.p','wb'), protocol=cPickle.HIGHEST_PROTOCOL)

print 'x = ' + str([v.getState()[0] for v in pi_rrt.path])
print 'y = ' + str([v.getState()[1] for v in pi_rrt.path])
print tabulate.tabulate([v.getState() for v in pi_rrt.path])
from Vertex import Vertex
from RRT import RRT
from PI_RRT import PI_RRT
import cPickle

# vInit = Vertex(-9.,0.,0.,0.,0.,0)
vInit = Vertex(-3.274496147453313, -0.4380171488751353, -0.072003545741959094, 0.0, 0.0, -1)
# vInit = Vertex(-5.553923114739766, -0.1469038304917034, -0.07016107956316503, 0.0, 0.0, -1)
# vInit = Vertex(-5.553923114739766, -0.1469038304917034,-0.,0.0,0.0,-1)
vGoal = Vertex(9.,0.,0.,10.,0.,0)
dt = 0.1
velocity = 2.3
wheelBase = 2.0
steeringRatio = 1
alpha = 0.25
r = 4.0

rrt = RRT(vInit,vGoal,dt,velocity,wheelBase,steeringRatio,alpha,r)
rrt.extractPath()

# for i in range(10):
#     rrt = RRT(vInit,vGoal,dt,velocity,wheelBase,steeringRatio,alpha,r)
#     rrt.extractPath()
#     print i

# pi_rrt = PI_RRT(vInit,vGoal)

# for i in range(10):
#     print 'COUNT '+ str(i)
#     pi_rrt.runRRT()
#     pi_rrt.generateTrajectories2()
#     try:
#         pi_rrt.executeControl2(*pi_rrt.computeVariation2())
#     except:
#         U,t = pi_rrt.computeVariation2()
#         print U
#         print t
#         break

# cPickle.dump(pi_rrt,open('pi_rrt.p', 'wb'),protocol=cPickle.HIGHEST_PROTOCOL)
class Obstacle(object):
	def __init__(self, center=[0,0], size=[0,0]):
	    self.center = center
	    self.size = size

	def getCorners():
		x1 = obstacle.center[0]-obstacle.size[0]/2
		x2 = obstacle.center[0]+obstacle.size[0]/2
		y1 = obstacle.center[1]-obstacle.size[1]/2
		y2 = obstacle.center[1]+obstacle.size[1]/2
		return x1,x2,y1,y2
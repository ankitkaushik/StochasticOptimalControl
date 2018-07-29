class Obstacle(object):
	def __init__(self, center=[0,0], size=[0,0]):
	    self.center = center
	    self.size = size

	def getCorners(self):
		x1 = self.center[0]-self.size[0]/2
		x2 = self.center[0]+self.size[0]/2
		y1 = self.center[1]-self.size[1]/2
		y2 = self.center[1]+self.size[1]/2
		return x1,x2,y1,y2

	def getArea(self):
		return self.size[0]*self.size[1]
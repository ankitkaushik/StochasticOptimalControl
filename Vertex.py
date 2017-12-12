class Vertex(object):
	def __init__(self, x = 0. , y = 0., theta = 0., time = 0., controlInput = 0., parent = -1):
	    self.x = x
	    self.y = y
	    self.theta = theta
	    self.time = time
	    self.controlInput = controlInput
	    self.parent = int(parent)

	def show(self):
		return self.x, self.y, self.theta, self.parent

	def getState(self):
		return self.x, self.y, self.theta, self.time, self.controlInput, self.parent
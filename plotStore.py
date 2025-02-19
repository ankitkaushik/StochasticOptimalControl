class plotStore(object):

	def __init__(self,vInit,vGoal,plotSaveDir):
		self.vInit = vInit
		self.vGoal = vGoal
		self.obstacles = []
		self.allRRTVertices = []
		self.sampledPoints = []
		self.path = []
		self.plotIndex = 0
		self.plotSaveDir = plotSaveDir
		self.RRTcompletionTimes = []
		self.RRTcompletionIterations = []
		self.RRTpaths=[]
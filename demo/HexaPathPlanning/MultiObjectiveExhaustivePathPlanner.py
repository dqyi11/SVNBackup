from Agent import *

class MultiObjectiveExhaustivePathPlanner(object):

    def __init__(self, hexamap, agent):
        self.map = hexamap
        self.runOnlyOnce = False
        self.agent = agent
        self.iterationCount = 0
        self.solutions = []
        self.solutionScores = []
        
        self.allPaths = []
        self.allScores = [] 
        
    def planPath(self, planGraph, start, planningLen, rewardDistributions, dimension):
        self.solutions = []
        self.solutionScores = []
        
        self.allPaths = []
        self.allScores = []
        # enumerate all the paths
        subpath = []
        subpath.append(start)
        self.findPath(planGraph, subpath, rewardDistributions)       
        
        # find non-dominated paths
        for i in range(len(self.allPaths)):
            nonDominated = True
            for j in range(len(self.allPaths)):
                if i!=j:
                    if self.dominated(self.allScores[j], self.allScores[i]):
                        nonDominated = False
            if nonDominated==True:
                self.solutions.append(self.allPaths[i])
                self.solutionScores.append(self.allScores[i])
        
        return self.solutions, self.solutionScores                       
                
        
    def dominated(self, vec1Val, vec2Val):
        #if vec1 dominate vec2
        dominated = True
        vecLen = len(vec1Val)
        for i in range(vecLen):
            if vec1Val[i] < vec2Val[i]:
                dominated = False
        return dominated    
        
    def findPath(self, planGraph, subpath, rewardDistributions):
        currentStep = len(subpath)
        if currentStep >= planGraph.T:
            tempPath = copy.deepcopy(subpath)
            self.allPaths.append(tempPath)
            self.allScores.append(self.agent.getPathRewardVec(tempPath, self.map, rewardDistributions))
            return
        
        
        edges = planGraph.partitions[currentStep].findEdges(tempPath[currentStep-1])
        for edge in edges:
            tempPath = copy.deepcopy(subpath)
            tempPath.append(edge[1])        
            self.findPath(planGraph, tempPath, rewardDistributions)
        
        
            
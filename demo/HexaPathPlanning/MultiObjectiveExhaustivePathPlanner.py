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
                    if True==self.isDominated(self.allScores[i], self.allScores[j]):
                        nonDominated = False
            if nonDominated==True:
                self.solutions.append(self.allPaths[i])
                self.solutionScores.append(self.allScores[i])
        
        return self.solutions, self.solutionScores                       
                
        
    def isDominated(self, vec1Val, vec2Val):
        #if vec1 dominated by vec2
        dominated = True
        vecLen = len(vec1Val)
        same = True
        for i in range(vecLen):
            if vec1Val[i] > vec2Val[i]:
                dominated = False
            if vec1Val[i] != vec2Val[i]:
                same = False
        if same==True:
            dominated = False
        return dominated    
        
    def findPath(self, planGraph, subpath, rewardDistributions):
        currentStep = len(subpath)
        #print "current step " + str(currentStep)
        if currentStep >= planGraph.T:
            tempPath = copy.deepcopy(subpath)
            self.allPaths.append(tempPath)
            self.allScores.append(self.agent.getPathRewardVec(tempPath, self.map, rewardDistributions))
            return
    
        edges = planGraph.partitions[currentStep-1].findEdges(subpath[currentStep-1])
        for edge in edges:
            #print "working on " +str(edge[0]) + " - " + str(edge[1])
            tempPath = copy.deepcopy(subpath)
            tempPath.append(edge[1])        
            self.findPath(planGraph, tempPath, rewardDistributions)
        
    def printToFile(self, filename):
        f = open(filename, 'w')
        for k in range(len(self.allPaths)):
            path = self.allPaths[k]
            for i in range(len(path)):
                p = path[i]
                if i==len(path)-1:
                    f.write(str(p) + " ")
                else:
                    f.write(str(p) + " - ")
            f.write(" : " + str(self.allScores[k]) + "\n")
        f.close()
        
        
            
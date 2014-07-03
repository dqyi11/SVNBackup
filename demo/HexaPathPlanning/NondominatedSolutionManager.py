
class NondominatedSolutionManager(object):
    
    def __init__(self, solutionLen):
        self.solutionLength = solutionLen
        self.nondominatedSolutions = []
        self.nondominatedSolutionScoreVecs = []
        
    def has(self, solution):
        if self.solutionLength != len(solution):
            return False
        for nds in self.nondominatedSolutions:
            if True==self.isSolutionSame(nds, solution):
                return True
        return False        
        
    def isSolutionSame(self, solution1, solution2):
        for i in range(self.solutionLength):
            if solution1[i][0]!=solution2[i][0] or solution1[i][1]!=solution2[i][1]:
                return False
        return True
        
    def printToFile(self, filename):
        f = open(filename, 'w')
        for k in range(len(self.nondominatedSolutions)):
            path = self.nondominatedSolutions[k]
            for i in range(len(path)):
                p = path[i]
                if i==len(path)-1:
                    f.write(str(p) + " ")
                else:
                    f.write(str(p) + " - ")
            f.write(" : " + str(self.nondominatedSolutionScoreVecs[k]) + "\n")
        f.close()
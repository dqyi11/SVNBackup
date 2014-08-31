import numpy as np
import copy

class Chromosome(object):
    
    def __init__(self, chromoLen, geneRange, mutateVar):
        self.chromoLen = chromoLen
        self.geneRange = geneRange
        self.mutateVar = mutateVar
        self.genes = np.random.random(chromoLen) * (geneRange[1] - geneRange[0]) + geneRange[0]
        self.fitness = 0.0
        self.normalizedFitness = 0.0
        self.normalizedWeight = 0.0
        
    def mutate(self):
        newChromo = copy.deepcopy(self)
        mutIdx = np.random.randint(0, self.chromoLen, 1)
        newChromo.genes[mutIdx] += np.random.normal(0, self.mutateVar, 1)
        return newChromo
                
    def crossover(self, other):
        newChromo = Chromosome(self.chromoLen, self.geneRange, self.mutateVar)
        crossIdx = np.random.randint(0, self.chromoLen, 1)
        for i in range(self.chromoLen):
            if i < crossIdx:
                newChromo.genes[i] = self.genes[i]
            else:
                newChromo.genes[i] = other.genes[i]
                
        return newChromo           


class GeneticAlgorithm(object):

    def __init__(self, population_num, geneRange, chromoLen, fitnessFunc, mutateVar=1.0, mutateProb = 0.2):
        self.population_num = population_num
        self.chromoLen = chromoLen
        self.geneRange = geneRange
        self.population = []
        self.fitnessFunc = fitnessFunc
        self.itrCnt = 0
        self.mutateVar = mutateVar
        self.mutateProb = mutateProb
        
        self.topNum = int(0.3*self.population_num)
        self.crossNum = 2 * int(0.2*self.population_num)
        self.mutateNum = self.population_num - self.topNum - self.crossNum
        
        for i in range(self.population_num):
            chromo = Chromosome(self.chromoLen, self.geneRange, self.mutateVar)
            chromo.fitness = self.fitnessFunc(chromo.genes)
            self.population.append(chromo)
            
    def normalizeWeights(self):
        weights = np.zeros(self.population_num)
        for i in range(self.population_num):
            weights[i] = self.population[i].fitness
            
        minVal = weights.min()
        maxVal = weights.max()
        self.weightSum = 0.0
        for i in range(len(weights)):
            self.population[i].normalizedFitness = ( weights[i] - minVal ) / (maxVal - minVal)
            self.population[i].normalizedWeight = 1.0 - self.population[i].normalizedFitness
            self.weightSum += self.population[i].normalizedWeight
            
    def sample(self):
        
        val = np.random.random() * self.weightSum
        cumVal = 0.0
        for p in self.population:
            if val >= cumVal and val < cumVal + p.normalizedWeight:
                return p
            cumVal += p.normalizedWeight
        return None
         

    def next(self):
        
        self.itrCnt += 1
        
        #for i in range(self.population_num):
            #print self.population[i].fitness
            
        
        newPopulation = []
        
        self.normalizeWeights()
        
        
        for i in range(self.population_num):
            
            chromoA = self.sample()
            chromoB = self.sample()
            
            newChromo = chromoA.crossover(chromoB)
            
            if np.random.random() < self.mutateProb:
                newChromo = newChromo.mutate()
                
            newPopulation.append(newChromo)

        
        '''
        for i in range(self.topNum):
            newPopulation.append(copy.deepcopy(self.population[i]))
            
        # MUTATE
        for i in range(self.mutateNum):
            mutateIdx = int(np.random.random()*self.topNum)
            newPopulation.append(self.population[i].mutate())
        
        # CROSSOVER 
        for i in range(int(self.crossNum/2)):
            cAIdx = int(np.random.random()*self.topNum)
            cBIdx = int(np.random.random()*self.topNum)
            cA = self.population[cAIdx]
            cB = self.population[cBIdx]
            newCA, newCB = cA.crossover(cB)
            newPopulation.append(newCA)
            newPopulation.append(newCB) 
        '''
        
        self.population = newPopulation
        
        for i in range(self.population_num):
            chromo = self.population[i]
            chromo.fitness = self.fitnessFunc(chromo.genes)
            
        self.population.sort(key=lambda Chromosome: Chromosome.fitness, reverse=False)
            
        
            
            
        
        
        
    
        

        
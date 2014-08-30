import numpy as np
import copy


class Chromosome(object):
    
    def __init__(self, chromoLen, geneRange):
        self.chromoLen = chromoLen
        self.geneRange = geneRange
        self.genes = np.random.random(chromoLen) * (geneRange[1] - geneRange[0]) + geneRange[0]
        self.fitness = 0.0
        
    def mutate(self):
        newChromo = copy.deepcopy(self)
        for i in range(newChromo.chromoLen):
            if np.random.random() > 0.5:
                newChromo.genes[i] = np.random.random() * (newChromo.geneRange[1] - newChromo.geneRange[0]) + newChromo.geneRange[0]
        return newChromo
                
    def crossover(self, other):
        chromoA = Chromosome(self.chromoLen, self.geneRange)
        chromoB = Chromosome(self.chromoLen, self.geneRange)
        crossIdx = int(np.random.random() * (self.chromoLen-1))
        for i in range(self.chromoLen):
            if i < crossIdx:
                chromoA.genes[i] = self.genes[i]
                chromoB.genes[i] = other.genes[i]
            else:
                chromoB.genes[i] = self.genes[i]
                chromoA.genes[i] = other.genes[i]
                
        return chromoA, chromoB            


class GeneticAlgorithm(object):


    def __init__(self, population_num, geneRange, chromoLen, fitnessFunc):
        self.population_num = population_num
        self.chromoLen = chromoLen
        self.geneRange = geneRange
        self.population = []
        self.fitnessFunc = fitnessFunc
        self.itrCnt = 0
        
        self.topNum = int(0.3*self.population_num)
        self.crossNum = 2 * int(0.2*self.population_num)
        self.mutateNum = self.population_num - self.topNum - self.crossNum
        
        for i in range(self.population_num):
            chromo = Chromosome(self.chromoLen, self.geneRange)
            chromo.fitness = self.fitnessFunc(chromo.genes)
            self.population.append(chromo)
            
    def next(self):
        
        self.itrCnt += 1
   
        self.population.sort(key=lambda Chromosome: Chromosome.fitness, reverse=False)
        
        #for i in range(self.population_num):
            #print self.population[i].fitness
            
        
        newPopulation = []
        
        
        
        
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
        
        
        self.population = newPopulation
        
        for i in range(self.population_num):
            chromo = self.population[i]
            chromo.fitness = self.fitnessFunc(chromo.genes)
            
        
            
            
        
        
        
    
        

        
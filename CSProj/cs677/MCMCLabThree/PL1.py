'''
Created on 2013-6-7

@author: Walter
'''
from BayesNet import *
from BayesNode import *

if __name__ == '__main__':
    
    facultyData = [float(line) for line in open('faculty.dat')]
    
    Net = BayesNet()
    HMM = NormalNode("HyperMeanOfMean", mean = 5, variance=1.0/9.0)
    HMV = InvGammaNode("HyperVarianceOfMean", alpha = 10, beta = 5)
    HVA = InvGammaNode("HyperAlphaOfVariance", alpha = 10, beta = 5)
    HVB = InvGammaNode("HyperBetaOfVariance", alpha = 10, beta =5 )
    Mean = NormalNode("Mean", mean="HyperMeanOfMean", variance="HyperVarianceOfMean")
    Variance = InvGammaNode("Variance", alpha="HyperAlphaOfVariance", beta="HyperBetaOfVariance")
    
    faculty = []
    facultyObserve = {}
    
    Net.addNode(HMM)
    Net.addNode(HMV)
    Net.addNode(HVA)
    Net.addNode(HVB)
    Net.addNode(Mean)
    Net.addNode(Variance)
    
    for i in range(len(facultyData)):
        name = "F{}".format(i)
        faculty.append(NormalNode(name, mean="Mean", variance="Variance"))
        Net.addNode(faculty[i])
        
        facultyObserve[name] = facultyData[i]
        
    Net.plot("faculty")
    
    sampleNum  = 40000
    sampleFrom = 10000
    q1 = Query(["Mean", "Variance","HyperMeanOfMean","HyperVarianceOfMean","HyperAlphaOfVariance","HyperBetaOfVariance"], facultyObserve)
    #print q1.printQuery()
    
    initial = {"Mean":5,"Variance":0.3,"HyperMeanOfMean":5,"HyperVarianceOfMean":0.5,"HyperAlphaOfVariance":3.0,"HyperBetaOfVariance":1.0}
    candsd = {"Mean":0.2,"Variance":0.15,"HyperMeanOfMean":0.2,"HyperVarianceOfMean":0.2,"HyperAlphaOfVariance":0.01,"HyperBetaOfVariance":0.01}
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)
    
    q1.plotSamples(samples)
    
    fileWriter = open('facultyEval.txt', 'w')
    for s in samples:
        fileWriter.write(str(s)+"\n")
    fileWriter.close()
    
    
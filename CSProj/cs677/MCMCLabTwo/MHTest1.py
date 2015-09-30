'''
Created on May 31, 2013

@author: walter
'''

from BayesNet import *
from BayesNode import *
import scipy.stats

if __name__ == '__main__':
    
    facultyData = [float(line) for line in open('faculty.dat')]
    
    Net = BayesNet()
    Mean = NormalNode("Mean", mean=5, variance=1.0/9.0)
    Variance = InvGammaNode("Variance", alpha=11, beta=2.5)
    
    faculty = []
    facultyObserve = {}
    
    Net.addNode(Mean)
    Net.addNode(Variance)
    
    for i in range(len(facultyData)):
        name = "F{}".format(i)
        faculty.append(NormalNode(name, mean="Mean", variance="Variance"))
        Net.addNode(faculty[i])
        
        facultyObserve[name] = facultyData[i]
        
    #Net.plot("faculty")
    
    sampleNum  = 40000
    sampleFrom = 10000
    q1 = Query(["Mean", "Variance"], facultyObserve)
    #print q1.printQuery()
    
    initial = {"Mean":5,"Variance":0.3}
    candsd = {"Mean":0.2,"Variance":0.15}
    samples = q1.doGibbsSampleWithHM(Net, initial, candsd, sampleNum, sampleFrom)
    
    def meanRef(x):
        prior_mean = 5
        prior_var = 1.0/9.0
        denom = len(facultyData) * prior_var + initial["Variance"]
        post_mean = (sum(facultyData) * prior_var + prior_mean * initial["Variance"]) / denom
        post_var = (prior_var * initial["Variance"]) / denom
        
        post_mean = 5
        post_var = 1.0/9.0
        num = len(x)
        y = np.zeros(num)
        for i in range(num):
            y[i] = (1 / (2 * math.pi * post_var)**0.5) * math.exp(- (x[i] - post_mean)**2 / (2 * post_var) )
        
        return y
    
    def varianceRef(x):
        prior_alpha = 11
        prior_beta = 2.5
        post_alpha = prior_alpha + len(facultyData) / 2
        sum_sq_err = sum((o - initial["Mean"])**2 for o in facultyData)
        post_beta = (prior_beta + sum_sq_err / 2)
        
        post_alpha = 11
        post_beta = 2.5
        rv = scipy.stats.invgamma(post_alpha, scale=post_beta)
        num = len(x)
        y = np.zeros(num)
        for i in range(num):
            if x[i] > 0 :
                #y[i] = rv.pdf(x[i])
                y[i] = post_beta**post_alpha / math.gamma(post_alpha) * x[i]**(-post_alpha - 1) * math.exp(-post_beta / x[i])  
            else:
                y[i] = 0 
        return y
    
    q1.addRefFunc("Mean",meanRef)
    q1.addRefFunc("Variance",varianceRef)
    
    q1.plotSamples(samples, True)

'''
Created on May 25, 2013

@author: walter
'''

from BayesNet import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    B = BernoulliNode("Burglary")          
    E = BernoulliNode("Earthquake")
    A = BernoulliNode("Alarm", 'Burglary Earthquake')
    J = BernoulliNode("JohnCalls", 'Alarm')
    M = BernoulliNode("MaryCalls", 'Alarm')
    
    B['T'] = 0.001
    E['T'] = 0.002
    A['T','T','T'] = 0.95
    A['T','T','F'] = 0.94
    A['T','F','T'] = 0.29
    A['T','F','F'] = 0.001
    J['T','T'] = 0.90
    J['T','F'] = 0.05
    M['T','T'] = 0.70
    M['T','F'] = 0.01
    
    Net.addNode(B)
    Net.addNode(E)
    Net.addNode(A)
    Net.addNode(J)
    Net.addNode(M)

    Net.completeFrom(True)
    
    Net.plot('test')
    
    #print Net.getMarkovBlanket("Burglary")
    
    sampleSize = 800000
    sampleFrom = 200000
    
    q1 = Query(["Burglary"],{"JohnCalls":"T","MaryCalls":"T"})
    q1.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q1.printEstProb()
    
    
    q2 = Query(["Alarm"],{"JohnCalls":"T","MaryCalls":"T"})
    q2.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q2.printEstProb()
    
    q3 = Query(["Earthquake"],{"JohnCalls":"T","MaryCalls":"T"})
    q3.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q3.printEstProb()
    
    q4 = Query(["Burglary"],{"JohnCalls":"F","MaryCalls":"F"})
    q4.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q4.printEstProb()
    
    q5 = Query(["Burglary"],{"JohnCalls":"T","MaryCalls":"F"})
    q5.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q5.printEstProb()
    
    q6 = Query(["Burglary"],{"JohnCalls":"T"})
    q6.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q6.printEstProb()
    
    q7 = Query(["Burglary"],{"MaryCalls":"T"})
    q7.estProbByGibbsSample(Net, sampleSize, sampleFrom)
    print q7.printEstProb() 
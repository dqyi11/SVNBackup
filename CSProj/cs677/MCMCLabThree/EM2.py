'''
Created on 2013-6-14

@author: Walter
'''

from BayesNet import *
from BayesNode import *

if __name__ == '__main__':
    
    Net = BayesNet()        
    B = DiscreteVarNode("Burglary")          
    E = DiscreteVarNode("Earthquake")
    A = DiscreteVarNode("Alarm", 'Burglary Earthquake')
    J = DiscreteVarNode("JohnCalls", 'Alarm')
    M = DiscreteVarNode("MaryCalls", 'Alarm')
    
    B[True] = 0.001
    E[True] = 0.002
    A[True,True,True] = 0.95
    A[True,True,False] = 0.94
    A[True,False,True] = 0.29
    A[True,False,False] = 0.001
    J[True,True] = 0.90
    J[True,False] = 0.05
    M[True,True] = 0.70
    M[True,False] = 0.01
    
    Net.addNode(B)
    Net.addNode(E)
    Net.addNode(A)
    Net.addNode(J)
    Net.addNode(M)
    
    Net.init()
    
    estRun = 10
    evsi = np.zeros(estRun)
    for er in range(estRun):
        
        sampleNum = 80000
        sampleFrom = 40000
        
        q1 = Query(["Burglary"],{"MaryCalls":True})
        samples = q1.doGibbsSample(Net, sampleNum, sampleFrom)
        
        def calcUtitlity(instance, call):
            if instance["Burglary"] == True and call == True:
                return 0.0
            if instance["Burglary"] == True and call == False:
                return -1000.0
            if instance["Burglary"] == False and call == True:
                return -25.0
            if instance["Burglary"] == False and call == False:
                return 0.0
            
        def estProbabilty(varname, samples):
            total = len(samples)
            t_num = 0
            f_num = 0
            for s in samples:
                if s[varname] == True:
                    t_num += 1
                if s[varname] == False:
                    f_num += 1
            return float(t_num)/float(total), float(f_num)/float(total)
        
        def estMaxEU(samples):
            u_call_t = []
            u_call_f = []
            for s in samples:
                u_call_t.append(calcUtitlity(s, True))
                u_call_f.append(calcUtitlity(s, False))
                
            eu_call_t = np.mean(u_call_t)
            eu_call_f = np.mean(u_call_f)
            
            #print "MaxEU {}:{}".format(eu_call_t,eu_call_f)
            if eu_call_t > eu_call_f:
                return eu_call_t
            else:
                return eu_call_f
            
        maxEU_Mt = estMaxEU(samples)
        PAt, PAf = estProbabilty("Alarm", samples)
        
        #print estProbabilty("Burglary", samples)
        
        q2 = Query(["Burglary"],{"MaryCalls":True,"Alarm":True})
        samples2 = q2.doGibbsSample(Net, sampleNum, sampleFrom)
        
        #print estProbabilty("Burglary", samples2)
        
        maxEU_MtAt = estMaxEU(samples2)
        
        q3 = Query(["Burglary"],{"MaryCalls":True,"Alarm":False})
        samples3 = q3.doGibbsSample(Net, sampleNum, sampleFrom)
        
        #print estProbabilty("Burglary", samples3)
        
        maxEU_MtAf = estMaxEU(samples3)
        
        #print "{}:{} + {}:{}".format(maxEU_MtAt, PAt, maxEU_MtAf, PAf)
        
        evsi[er] = maxEU_MtAt * PAt + maxEU_MtAf * PAf - maxEU_Mt
        
    #print evsi
    fig = plt.figure()
    ax = fig.add_subplot(111)
    It = np.arange(estRun)
    ax.plot(It, evsi, "r.-")
    ax.set_xlabel(r"run time")
    ax.set_ylabel(r"EVSI")
    ax.set_title("EVSI in Alarm when Mary Calls")
    
    plt.show()
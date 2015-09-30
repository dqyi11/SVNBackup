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
        print "{} and {}".format(eu_call_t, eu_call_f)
        if eu_call_t > eu_call_f:
            return eu_call_t, True, eu_call_t, eu_call_f
        else:
            return eu_call_f, False, eu_call_t, eu_call_f
        
    sampleNum = 80000
    sampleFrom = 40000
    
    etRun = 10
    #maxEU_Mt = np.zeros(etRun)
    EUt_Mt = np.zeros(etRun)
    EUf_Mt = np.zeros(etRun)
    EUt_Jt = np.zeros(etRun)
    EUf_Jt = np.zeros(etRun)
    EUt_MtJt = np.zeros(etRun)
    EUf_MtJt = np.zeros(etRun)
    
    for er in range(etRun):
        q1 = Query(["Burglary"],{"MaryCalls":True})
        samples = q1.doGibbsSample(Net, sampleNum, sampleFrom)
            
        maxEU_Mt, choice1, EUt_Mt[er], EUf_Mt[er] = estMaxEU(samples)
        print "{}:{}".format(choice1, maxEU_Mt)
        #PAt, PAf = estProbabilty("Alarm", samples)
        
        q2 = Query(["Burglary"],{"JohnCalls":True})
        samples = q2.doGibbsSample(Net, sampleNum, sampleFrom)
        
        maxEU_Jt, choice2, EUt_Jt[er], EUf_Jt[er] = estMaxEU(samples)
        print "{}:{}".format(choice2, maxEU_Jt)
        
        q3 = Query(["Burglary"],{"MaryCalls":True,"JohnCalls":False})
        samples = q3.doGibbsSample(Net, sampleNum, sampleFrom)
        
        maxEU_MtJt, choice3, EUt_MtJt[er], EUf_MtJt[er] = estMaxEU(samples)
        print "{}:{}".format(choice3, maxEU_MtJt)
            
    
    it = np.arange(etRun)
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)    
    ax1.plot(it, EUt_Mt, label="Call Police")
    ax1.plot(it, EUf_Mt, label="Not Call Police")
    ax1.set_xlabel("run time")
    ax1.set_ylabel("utility")
    ax1.legend()
    ax1.set_title("E[U(a,Burglary)|MaryCalls=True]")
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)    
    ax2.plot(it, EUt_Jt, label="Call Police")
    ax2.plot(it, EUf_Jt, label="Not Call Police")
    ax2.set_xlabel("run time")
    ax2.set_ylabel("utility")
    ax2.legend()
    ax2.set_title("E[U(a,Burglary)|JohnCalls=True]")
    
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)    
    ax3.plot(it, EUt_MtJt, label="Call Police")
    ax3.plot(it, EUf_MtJt, label="Not Call Police")
    ax3.set_xlabel("run time")
    ax3.set_ylabel("utility")
    ax3.legend()
    ax3.set_title("E[U(a,Burglary)|MaryCalls=True,JohnCalls=True]")
    
    plt.show()
    
    
        
        
    
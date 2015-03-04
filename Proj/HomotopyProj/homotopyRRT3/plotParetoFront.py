'''
Created on Feb 27, 2015

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt

def findNondominance(solutions):
    doms = []
    nondoms = []
    for s1 in solutions:
        s1_nondomed = True
        for s2 in solutions:
            if s1[0] == s2[0] and s1[1] == s2[1]:
                continue
            s2_dom_s1 = True
            for i in range(len(s1)):
                if s1[i] < s2[i]:
                    s2_dom_s1 = False
            if s2_dom_s1 == True:
                s1_nondomed = False
        if s1_nondomed == True:
            nondoms.append(s1)
        else:
            doms.append(s1)
                
        
    return doms, nondoms

if __name__ == '__main__':
    
    # load txt and plot
    #FILENAME = "paths01.txt"
    FILENAME = "bestpath02s.txt"
    data = np.loadtxt(FILENAME, dtype=float)
    #
    print data
    
    doms, nondoms = findNondominance(data)
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    
    domx = []
    domy = []
    for d in doms:
        #print d
        domx.append(d[0])
        domy.append(d[1])
    ax1.plot(domx, domy, color='red', marker='o' , markersize=10, linewidth=0)
    ndomx = []
    ndomy = []
    for nd in nondoms:
        #print nd
        ndomx.append(nd[0])
        ndomy.append(nd[1])
    ax1.plot(ndomx, ndomy, color='blue', marker='s', markersize=10, linewidth=0)
        
    params = {'legend.fontsize': 20}
    plt.rcParams.update(params)

    ax1.set_xlabel('homotopy unsatisfaction', fontsize=20)
    ax1.set_ylabel('shortest distance', fontsize=20)
    ax1.legend(['Dominated', 'Non-dominated'])
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    datalen = len(data)
    xvars = []
    for d in data:
        xvars.append(d[1])
    ax2.bar(np.arange(datalen), xvars, 0.4)
    ax2.set_xlabel('homotopy class', fontsize=20)
    ax2.set_ylabel('shortest distance', fontsize=20)
    
        
    plt.show()
    
    
import numpy as np
import matplotlib.pyplot as plt

def Rastrigin(X):
    val = 0.0
    for d in range(2):
        val += 10 + X[d]**2 - 10 * np.cos(2 * np.pi * X[d])
    return val


if __name__ == '__main__':
     
    runs = 100
    
    phi1 = 2
    phi2 = 2
    #inertia = 0.8
    #phi1 = 2.05
    #phi2 = 2.05
    inertia = 0.72984
    #inertia = 0.9
    
    dim = 2
    
    pb_init = [4.0, 4.2]
    gb_init = [4.6, 4.6]
    
    pb = pb_init
    gb = gb_init
    
    pos = [0.0, 0.0]
    pos[0] = (np.random.random() - 0.5) * (gb[0] - pb[0]) + pb[0]
    pos[1] = (np.random.random() - 0.5) * (gb[1] - pb[1]) + pb[1]
    vel = [0.0, 0.0]
    
    
    gb_fitness = Rastrigin(gb)
    pb_fitness = Rastrigin(pb)
    
    pbHist = []
    gbHist = []
    posHist = []
    
    fitHist = []
    gbFitHist = []
    pbFitHist = []
    
    pbHist1 = []
    pbHist2 = []
    gbHist1 = []
    gbHist2 = []
    posHist1 = []
    posHist2 = []
    
    for i in range(runs):
        
        pbHist.append(pb)
        gbHist.append(gb)
        posHist.append(pos)
        
        pbHist1.append(pb[0])
        pbHist2.append(pb[1])
        gbHist1.append(gb[0])
        gbHist2.append(gb[1])
        posHist1.append(pos[0])
        posHist2.append(pos[1])
        
        for j in range(dim):
            u1 = np.random.random()
            u2 = np.random.random()
            localForce = phi1 * u1 * (pb[j] - pos[j])
            globalForce = phi2 * u2 * (gb[j] - pos[j])
        
            vel[j] = inertia * (vel[j] + localForce + globalForce)
            pos[j] = pos[j] + vel[j]
        
        fitness = Rastrigin(pos)
        if fitness < gb_fitness:
            gb = pos
            gb_fitness = fitness 
        if fitness < pb_fitness:
            pb = pos
            pb_fitness = fitness
            
        fitHist.append(fitness)
        gbFitHist.append(gb_fitness)
        pbFitHist.append(pb_fitness)
        
        
    xd1, xd2 = np.meshgrid(np.arange(-5.12,5.12,0.02), np.arange(-5.12,5.12,0.02)) 
    dataLen = len(xd1)
    xs = []
    ys = []
    for i in range(dataLen):
        xs.append([xd1[i], xd2[i]])
    for x in xs:
        ys.append(Rastrigin(x))
        
    fig0 = plt.figure()
    ax0 = fig0.add_subplot(111)
    ax0.set_title('distribution')
    ax0.contour(xd1,xd2,ys)
    ax0.plot(pb_init[0], pb_init[1], marker='8')
    ax0.plot(gb_init[0], gb_init[1], marker='o')
    ax0.plot(0, 0, marker='s')
    ax0.legend(["personal best","global best", "optimal"],loc=2)
    
    
    maxPos = 0 * np.ones(runs)
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.set_title('position 1D')
    ax1.plot(np.arange(runs), posHist1, np.arange(runs), pbHist1, np.arange(runs), gbHist1, np.arange(runs), maxPos)
    ax1.legend(["particle", "personal best", "global best", "optimal"], loc=4)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('position 2D')
    ax2.plot(np.arange(runs), posHist2, np.arange(runs), pbHist2, np.arange(runs), gbHist2, np.arange(runs), maxPos)
    ax2.legend(["particle", "personal best", "global best", "optimal"], loc=4)
    
    maxFitness = 0 * np.ones(runs)
    
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_title('fitness')
    ax3.plot(np.arange(runs), fitHist, np.arange(runs), pbFitHist, np.arange(runs), gbFitHist, np.arange(runs), maxFitness)
    ax3.legend(["particle", "personal best", "global best", "optimal"])
    
    plt.show()

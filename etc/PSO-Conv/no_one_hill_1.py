import numpy as np
import matplotlib.pyplot as plt

def fitness_func(val):
    
    if val < 0.0:
        return 10 - ((val+60.0)/2.0)**2
    else: 
        return -790 - ((val-20.0)/2.0)**2


if __name__ == '__main__':
     
    runs = 100
    
    phi1 = 4.05
    phi2 = 4.05
    #inertia = 0.8
    #phi1 = 2.05
    #phi2 = 2.05
    inertia = 0.72984
    #inertia = 0.9
    
    pb_init = 80.0
    gb_init = 70.0
    
    pb = pb_init
    gb = gb_init
    
    pos = (np.random.random() - 0.5) * (gb - pb) + pb
    vel = 0.0
    
    
    gb_fitness = fitness_func(gb)
    pb_fitness = fitness_func(pb)
    
    pbHist = []
    gbHist = []
    posHist = []
    
    fitHist = []
    gbFitHist = []
    pbFitHist = []
    
    for i in range(runs):
        
        pbHist.append(pb)
        gbHist.append(gb)
        posHist.append(pos)
        
        u1 = np.random.random()
        u2 = np.random.random()
        localForce = phi1 * u1 * (pb - pos)
        globalForce = phi2 * u2 * (gb - pos)
        
        vel = inertia * (vel + localForce + globalForce)
        pos = pos + vel
        
        fitness = fitness_func(pos)
        if fitness > gb_fitness:
            gb = pos
            gb_fitness = fitness 
        if fitness > pb_fitness:
            pb = pos
            pb_fitness = fitness
            
        fitHist.append(fitness)
        gbFitHist.append(gb_fitness)
        pbFitHist.append(pb_fitness)
        
    xs = np.arange(-100, 100, 1)
    ys = []
    for x in xs:
        ys.append(fitness_func(x))
    fig0 = plt.figure()
    ax0 = fig0.add_subplot(111)
    ax0.set_title('distribution')
    ax0.plot(xs,ys)
    ax0.plot(pb_init, fitness_func(pb_init), marker='8')
    ax0.plot(gb_init, fitness_func(gb_init), marker='o')
    ax0.plot(-60, 10, marker='s')
    ax0.legend(["fitness", "personal best","global best", "optimal"])
    
    maxPos = -60 * np.ones(runs)
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.set_title('position')
    ax1.plot(np.arange(runs), posHist, np.arange(runs), pbHist, np.arange(runs), gbHist, np.arange(runs), maxPos)
    ax1.legend(["particle", "personal best", "global best", "optimal"])
    
    maxFitness = 10 * np.ones(runs)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('fitness')
    ax2.plot(np.arange(runs), fitHist, np.arange(runs), pbFitHist, np.arange(runs), gbFitHist, np.arange(runs), maxFitness)
    ax2.legend(["particle", "personal best", "global best", "optimal"])
    
    
    plt.show()

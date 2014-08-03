import numpy as np
import matplotlib.pyplot as plt

def fitness_func(val):
    
    return 10-(val-4)**2

if __name__ == '__main__':
     
    runs = 400
    
    phi1 = 2
    phi2 = 2
    #inertia = 0.8
    #phi1 = 2.05
    #phi2 = 2.05
    inertia = 0.72984
    #inertia = 0.9
    
    pb = 400.0
    gb = 300.0
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
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('position')
    ax.plot(np.arange(runs), posHist, np.arange(runs), pbHist, np.arange(runs), gbHist)
    ax.legend(["particle", "personal best", "global best"])
    
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('fitness')
    ax2.plot(np.arange(runs), fitHist, np.arange(runs), pbFitHist, np.arange(runs), gbFitHist)
    ax2.legend(["particle", "personal best", "global best"])
    
    
    plt.show()

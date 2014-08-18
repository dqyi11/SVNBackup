import numpy as np
import matplotlib.pyplot as plt

def fitness_func(val):
    
    return 10-((val-4.0)/10.0)**2

if __name__ == '__main__':
     
    runs = 4000
    
    phi1 = 0.2
    phi2 = 0.2
    #inertia = 0.8
    #phi1 = 2.05
    #phi2 = 2.05
    inertia = 0.3
    #inertia = 0.9
    
    phi1_new = 2.05
    phi2_new = 2.05
    inertia_new = 0.72984
    
    pb_init = 800.0
    gb_init = 600.0
    
    pb = pb_init
    gb = gb_init
    pos = (np.random.random() - 0.5) * (gb - pb) + pb
    vel = 0.0
    
    pb2 = pb_init
    gb2 = gb_init
    pos2 = pos
    vel2 = 0.0
    
    gb_fitness = fitness_func(gb)
    pb_fitness = fitness_func(pb)
    
    pbHist = []
    gbHist = []
    posHist = []
    velHist = []
    
    fitHist = []
    gbFitHist = []
    pbFitHist = []
    
    gb2_fitness = fitness_func(gb2)
    pb2_fitness = fitness_func(pb2)
    
    pbHist2 = []
    gbHist2 = []
    posHist2 = []
    velHist2 = []
    
    fitHist2 = []
    gbFitHist2 = []
    pbFitHist2 = []
    
    u1s = np.random.random(runs)
    u2s = np.random.random(runs)
    
    stop = False
    for i in range(runs):
        
        pbHist.append(pb)
        gbHist.append(gb)
        posHist.append(pos)
        
        if stop==False:
            localForce = phi1 * u1s[i] * (pb - pos)
            globalForce = phi2 * u2s[i] * (gb - pos)
            
            vel = inertia * (vel + localForce + globalForce)
            pos = pos + vel
        else:
            vel = 0.0
        
        velHist.append(vel)
        
        fitness = fitness_func(pos)
        if fitness > gb_fitness:
            gb = pos
            gb_fitness = fitness
            stop = True
        if fitness > pb_fitness:
            pb = pos
            pb_fitness = fitness
            
        fitHist.append(fitness)
        gbFitHist.append(gb_fitness)
        pbFitHist.append(pb_fitness)
      
    stop = False 
    for i in range(runs):
        
        pbHist2.append(pb2)
        gbHist2.append(gb2)
        posHist2.append(pos2)
        
        if stop == False:
            localForce = phi1_new * u1s[i] * (pb2 - pos2)
            globalForce = phi2_new * u2s[i] * (gb2 - pos2)
            
            vel2 = inertia_new * (vel2 + localForce + globalForce)
            pos2 = pos2 + vel2
        else:
            vel2 = 0.0
        
        velHist2.append(vel2)
        
        
        fitness2 = fitness_func(pos2)
        if fitness2 > gb2_fitness:
            gb2 = pos2
            gb2_fitness = fitness2 
        if fitness2 > pb2_fitness:
            pb2 = pos2
            pb2_fitness = fitness2
            
        fitHist2.append(fitness2)
        gbFitHist2.append(gb2_fitness)
        pbFitHist2.append(pb2_fitness)
        
        
    xs = np.arange(-100, 1000, 1)
    ys = fitness_func(xs)
    fig0 = plt.figure()
    ax0 = fig0.add_subplot(111)
    ax0.set_title('distribution')
    ax0.plot(xs,ys)
    ax0.plot(pb_init, fitness_func(pb_init), marker='8')
    ax0.plot(gb_init, fitness_func(gb_init), marker='o')
    ax0.plot(4, 10, marker='s')
    ax0.legend(["fitness", "optimal",'personal best','global best'])
    
    maxPos = 4 * np.ones(runs)
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.set_title('position')
    ax1.plot(np.arange(runs), posHist, np.arange(runs), posHist2, np.arange(runs), maxPos)
    ax1.legend(["parameter 1", "parameter 2", "optimal"])
    
    '''
    maxFitness = 10 * np.ones(runs)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('fitness')
    ax2.plot(np.arange(runs), fitHist, np.arange(runs), pbFitHist, np.arange(runs), gbFitHist, np.arange(runs), maxFitness)
    ax2.legend(["particle", "personal best", "global best", "optimal"])
    
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_title('position')
    ax3.plot(np.arange(runs), posHist2, np.arange(runs), pbHist2, np.arange(runs), gbHist2, np.arange(runs), maxPos)
    ax3.legend(["particle", "personal best", "global best", "optimal"])
    
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.set_title('fitness')
    ax4.plot(np.arange(runs), fitHist2, np.arange(runs), pbFitHist2, np.arange(runs), gbFitHist2, np.arange(runs), maxFitness)
    ax4.legend(["particle", "personal best", "global best", "optimal"])
    '''
    plt.show()

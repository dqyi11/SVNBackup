'''
Created on Nov 14, 2013

@author: daqing_yi
'''

import numpy as np;
import matplotlib.pyplot as plt;

def func1(x):
    return x**2;

def func2(x):
    return (x-4)**2;

def maximin(i, buf):
    
    w, h = np.shape(buf);
    min = []
    '''
    for j in range(w):
        if i != j:
            min.append(np.amin(buf[j,:]));
    maxIdx = np.amax(min);
    if maxIdx >= i:
        return maxIdx + 1;
    '''
    for j in range(w):
        min.append(np.amin(buf[j,:]));
    maxIdx = np.amax(min);
    
    
    return maxIdx;
    

if __name__ == '__main__':

    class Particle:
        pass;
    
    # range (-10, 10);
    
    # swarm size
    pop_size = 100;
    iter_max = 4000;
    c1 = 2;
    c2 = 2;
    mo_num = 2;
    mo_func = [func1, func2];
    world_size = 5.0;
    w = 0.8;
    
    # init population
    particles = [];
    for i in range(pop_size):
        p = Particle()
        p.pos = (np.random.random() - 0.5 ) * 2 *world_size;
        p.fitness = 0.0;
        p.pb_fitness = None;
        p.buf = np.zeros((pop_size, mo_num));
        p.v = 0.0;
        particles.append(p);
        
    posArray = []
    for p in particles:
        posArray.append(p.pos);     
        
    fig = plt.figure();
    ax = fig.add_subplot(111);
    ind = np.arange(-world_size, world_size, 0.05);
    F1 = [];
    F2 = [];
    for val in ind:
        F1.append(func1(val));
        F2.append(func2(val));
        
    pF1 = [];
    pF2 = [];
    for p in particles:
        pF1.append(func1(p.pos));
        pF2.append(func2(p.pos));
    
    ax.plot(F1,F2,'.');
    ax.plot(pF1,pF2,'x', color='red');
    ax.plot(np.mean(pF1), np.mean(pF2), 'D', color='orange');
    ax.plot(func1(np.mean(posArray)), func2(np.mean(posArray)), 's', color='brown');
    ax.set_xlabel("Func 1");
    ax.set_ylabel("Func 2");
    
    plt.show();        
        
    
    # run and calc fitness function
    gbest = particles[0]
    i = 0;
    while i < iter_max :
        # update fitness
        paIdx = 0;
        for p in particles:
            for p1 in range(pop_size):
                for p2 in range(mo_num):
                    p.buf[p1, p2] = mo_func[p2](p.pos) - mo_func[p2](particles[p1].pos);
                    
            p.fitness = maximin(paIdx, p.buf);
            
            if i == 0:
                p.pb_fitness = p.fitness;
                p.pb_best = p.pos
                
            if p.fitness < p.pb_fitness:
                p.pb_fitness = p.fitness
                p.pb_best = p.pos
     
            if p.fitness < gbest.fitness:
                gbest = p
                
            paIdx = paIdx + 1;
            
        # move      
        for p in particles:
            v = w * p.v + c1 * np.random.random() * (p.pb_best - p.pos) \
                    + c2 * np.random.random() * (gbest.pos - p.pos);
            p.pos = p.pos + v;
            if p.pos > world_size:
                p.pos = world_size;
            elif p.pos < -world_size:
                p.pos = -world_size;
              
        i  += 1;
        
    posArray = []
    for p in particles:
        posArray.append(p.pos); 
            
    
    fig = plt.figure();
    ax = fig.add_subplot(111);
    ind = np.arange(-world_size, world_size, 0.01);
    F1 = [];
    F2 = [];
    for val in ind:
        F1.append(func1(val));
        F2.append(func2(val));
        
    pF1 = [];
    pF2 = [];
    for p in particles:
        pF1.append(func1(p.pos));
        pF2.append(func2(p.pos));
    
    ax.plot(F1,F2,'.');
    ax.plot(pF1,pF2,'x', color='red');
    ax.plot(np.mean(pF1), np.mean(pF2), 'D', color='orange');
    ax.plot(func1(np.mean(posArray)), func2(np.mean(posArray)), 's', color='brown');
    ax.set_xlabel("Func 1");
    ax.set_ylabel("Func 2");
    
    yMin, yMax = plt.ylim();
    yBuff = 0.1*(yMax - yMin);
    plt.ylim(yMin - yBuff, yMax + yBuff);
    
    xMin, xMax = plt.xlim();
    xBuff = 0.1*(xMax - xMin);
    plt.xlim(xMin - xBuff, xMax + xBuff);
    #print pF1;
    #print pF2;
    
    plt.show();
    
    print "({}, {})".format(np.mean(pF1), np.mean(pF2));
          
    print "[{}] - ({}, {})".format(np.mean(posArray), func1(np.mean(posArray)), func2(np.mean(posArray)));
    
        
    
    
    
    
    
    
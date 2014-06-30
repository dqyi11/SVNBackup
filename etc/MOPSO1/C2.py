'''
Created on 2013-11-18

@author: Walter
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
    particles1 = [];
    for i in range(pop_size):
        p1 = Particle()
        p1.pos = (np.random.random() - 0.5 ) * 2 *world_size;
        p1.fitness = 0.0;
        p1.pb_fitness = None;
        p1.buf = np.zeros((pop_size, mo_num));
        p1.v = 0.0;
        particles1.append(p1);
        
    particles2 = [];
    for i in range(pop_size):
        p2 = Particle()
        p2.pos = (np.random.random() - 0.5 ) * world_size/2;
        p2.fitness = 0.0;
        p2.pb_fitness = None;
        p2.buf = np.zeros((pop_size, mo_num));
        p2.v = 0.0;
        particles2.append(p2);
    
    # run and calc fitness function
    gbest = particles1[0]
    i = 0;
    while i < iter_max :
        # update fitness
        for p1 in particles1:
            p1.fitness = func1(p1.pos);
            
            if i == 0:
                p1.pb_fitness = p1.fitness;
                p1.pb_best = p1.pos
                
            if p1.fitness < p1.pb_fitness:
                p1.pb_fitness = p1.fitness
                p1.pb_best = p1.pos
     
            if p1.fitness < gbest.fitness:
                gbest = p1;
            
        # move      
        for p1 in particles1:
            v = w * p1.v + c1 * np.random.random() * (p1.pb_best - p1.pos) \
                    + c2 * np.random.random() * (gbest.pos - p1.pos);
            p1.pos = p1.pos + v;
            if p1.pos > world_size:
                p1.pos = world_size;
            elif p1.pos < -world_size:
                p1.pos = -world_size;
              
        i  += 1;
        
    posArray1 = []
    for p1 in particles1:
        posArray1.append(p1.pos);
        
    # run and calc fitness function
    gbest = particles2[0]
    i = 0;
    while i < iter_max :
        # update fitness
        for p2 in particles2:
            p2.fitness = func2(p2.pos);
            
            if i == 0:
                p2.pb_fitness = p2.fitness;
                p2.pb_best = p2.pos
                
            if p2.fitness < p2.pb_fitness:
                p2.pb_fitness = p2.fitness
                p2.pb_best = p2.pos
     
            if p2.fitness < gbest.fitness:
                gbest = p2;
            
        # move      
        for p2 in particles2:
            v = w * p2.v + c1 * np.random.random() * (p2.pb_best - p2.pos) \
                    + c2 * np.random.random() * (gbest.pos - p2.pos);
            p2.pos = p2.pos + v;
            if p2.pos > world_size:
                p2.pos = world_size;
            elif p2.pos < -world_size:
                p2.pos = -world_size;
              
        i  += 1;    
        
    posArray2 = []
    for p2 in particles2:
        posArray2.append(p2.pos);          
    
    fig = plt.figure();
    ax = fig.add_subplot(111);
    ind = np.arange(-world_size, world_size, 0.05);
    F1 = [];
    F2 = [];
    for val in ind:
        F1.append(func1(val));
        F2.append(func2(val));
        
    p1F1 = [];
    p1F2 = [];
    p2F1 = [];
    p2F2 = []
    for p1 in particles1:
        p1F1.append(func1(p1.pos));
        p1F2.append(func2(p1.pos));
    for p2 in particles2:
        p2F1.append(func1(p2.pos));
        p2F2.append(func2(p2.pos));
    
    ax.plot(F1,F2,'.');
    ax.plot(p1F1,p1F2,'x', color='red');
    ax.plot(p2F1,p2F2, 'x', color='green');
    ax.plot(np.mean(p1F1), np.mean(p1F2), 'D', color='orange');
    ax.plot(np.mean(p2F1), np.mean(p2F2), 'D', color='brown');
    ax.plot(func1(np.mean(p1.pos)), func2(np.mean(p1.pos)), 's', color='orange');
    ax.plot(func1(np.mean(p2.pos)), func2(np.mean(p2.pos)), 's', color='brown')
    ax.set_xlabel("Func 1");
    ax.set_ylabel("Func 2");
    #print pF1;
    #print pF2;
    
    yMin, yMax = plt.ylim();
    yBuff = 0.1*(yMax - yMin);
    plt.ylim(yMin - yBuff, yMax + yBuff);
    
    xMin, xMax = plt.xlim();
    xBuff = 0.1*(xMax - xMin);
    plt.xlim(xMin - xBuff, xMax + xBuff);
    
    plt.show();
    
    #print "({}, {})".format(np.mean(pF1), np.mean(pF2));
          
    #print "[{}] - ({}, {})".format(np.mean(posArray), func1(np.mean(posArray)), func2(np.mean(posArray)));
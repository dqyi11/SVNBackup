'''
Created on Dec 17, 2014

@author: daqing_yi
'''

from Cannon import *
import pylab

if __name__ == '__main__':
    
    #=============================REAL PROGRAM START================================
    # Let's go over the physics behind the cannon shot, just to make sure it's
    # correct:
    # sin(45)*100 = 70.710 and cos(45)*100 = 70.710
    # vf = vo + at
    # 0 = 70.710 + (-9.81)t
    # t = 70.710/9.81 = 7.208 seconds for half
    # 14.416 seconds for full journey
    # distance = 70.710 m/s * 14.416 sec = 1019.36796 m
    
    timeslice = 0.05 # How many seconds should elapse per iteration?
    iterations = 288 # How many iterations should the simulation run for?
    # (notice that the full journey takes 14.416 seconds, so 145 iterations will
    # cover the whole thing when timeslice = 0.10)
    noiselevel = 30  # How much noise should we add to the noisy measurements?

    # These are arrays to store the data points we want to plot at the end.
    x = []
    y = []
    nx = []
    ny = []
    vx = []
    vy = []
    
    # Let's make a cannon simulation.
    c = Cannon(timeslice,noiselevel)
    
    for i in range(iterations):
        x.append(c.GetX())
        y.append(c.GetY())
        newestX = c.GetXWithNoise()
        newestY = c.GetYWithNoise()
        nx.append(newestX)
        ny.append(newestY)
        # Iterate the cannon simulation to the next timeslice.
        c.Step()
        vx.append(c.GetXVelocity())
        vy.append(c.GetYVelocity())
        
    # Plot all the results we got.
    pylab.plot(x,y,'-',nx,ny,':')
    pylab.xlabel('X position')
    pylab.ylabel('Y position')
    pylab.title('Measurement of a Cannonball in Flight')
    pylab.legend(('true','measured'))
    pylab.show()
    
    with open('position.txt', 'w') as f1:
        for i in range(iterations):
            f1.write(str(x[i])+" "+str(y[i])+"\n")
        
    with open('noised_position2.txt', 'w') as f2:
        for i in range(iterations):
            f2.write(str(nx[i])+" "+str(ny[i])+"\n")
            
    with open('velocity.txt', 'w') as f3:
        for i in range(iterations):
            f3.write(str(vx[i])+" "+str(vy[i])+"\n")
        
    

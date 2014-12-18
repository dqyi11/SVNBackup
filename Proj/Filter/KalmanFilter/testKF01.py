'''
Created on Dec 17, 2014

@author: daqing_yi
'''

import pylab, numpy, math
from KalmanFilter import *

if __name__ == '__main__':
    
    timeslice = 0.05 
    muzzle_velocity = 100 # How fast should the cannonball come out?
    angle = 45 # Angle from the ground.

    x = []
    y = []
    with open('position.txt', 'r') as f1:
        for line in f1:
            pdata = line.split(" ")
            x.append(float(pdata[0]))
            y.append(float(pdata[1]))
        
    nx1 = []
    ny1 = []
    with open('noised_position1.txt', 'r') as f2:
        for line in f2:
            pdata = line.split(" ")
            nx1.append(float(pdata[0]))
            ny1.append(float(pdata[1]))
            
    nx2 = []
    ny2 = []
    with open('noised_position2.txt', 'r') as f4:
        for line in f4:
            pdata = line.split(" ")
            nx2.append(float(pdata[0]))
            ny2.append(float(pdata[1]))
            
    vx = []
    vy = []
    with open('velocity.txt', 'r') as f3:
        for line in f3:
            pdata = line.split(" ")
            vx.append(float(pdata[0]))
            vy.append(float(pdata[1]))
            
    # This is the state transition vector, which represents part of the kinematics.
    # 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
    # 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
    # 0,  1, 0,  0  => vx(n+1) =        vx(n)
    # 0,  0, 0,  1  => vy(n+1) =                     vy(n)
    # Remember, acceleration gets added to these at the control vector.
    A = numpy.matrix([[1,0, timeslice,0],[0,1,0,timeslice],[0,0,1,0],[0,0,0,1]])
                
    B = numpy.matrix([[0,0,0,0],[0,1,0,0], [0,0,0,0], [0,0,0,1]])
    # The control vector, which adds acceleration to the kinematic equations.
    # 0          =>  x(n+1) =  x(n+1)
    # -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
    # 0          => vx(n+1) = vx(n+1)
    # -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
    U = numpy.matrix([[0],[0.5*-9.81*timeslice*timeslice],[0],[-9.81*timeslice]])
    
    # After state transition and control, here are the equations:
    #  x(n+1) = x(n) + vx(n)
    #  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
    # vx(n+1) = vx(n)
    # vy(n+1) = vy(n) + -9.81*ts
    # Which, if you recall, are the equations of motion for a parabola.  Perfect.
    
    # Observation matrix is the identity matrix, since we can get direct
    # measurements of all values in our example.
    # H = numpy.eye(4)
    H = numpy.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    
    speedX = muzzle_velocity*math.cos(angle*math.pi/180)
    speedY = muzzle_velocity*math.sin(angle*math.pi/180)
    # This is our guess of the initial state.  I intentionally set the Y value
    # wrong to illustrate how fast the Kalman filter will pick up on that.
    X0 = numpy.matrix([[0],[500],[speedX],[speedY]])
    
    #P0 = numpy.eye(4)
    P0 = numpy.matrix([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    Q = numpy.zeros(4)
    #R = numpy.eye(4)*0.2
    #R = numpy.matrix([[0.2, 0, 0, 0], [0, 0, 0.2, 0], [0, 0.2, 0, 0], [0, 0, 0, 0.2]])
    R = 0.2 * numpy.eye(6)
    
    iterations = len(x)
    
    kf = KalmanFilter(A, B, H, X0, P0, Q, R)
    
    kx = []
    ky = []
    # Iterate through the simulation.
    for i in range(iterations):
        kx.append(kf.GetCurrentState()[0,0])
        ky.append(kf.GetCurrentState()[1,0])
        kf.Step(U,numpy.matrix([[nx1[i]],[ny1[i]],[nx2[i]],[ny2[i]],[vx[i]],[vy[i]]]))
    
                
    # Plot all the results we got.
    pylab.plot(x,y,'r-',nx1,ny1,'y:',nx2, ny2,'g--',kx,ky,'b.-')
    pylab.xlabel('X position')
    pylab.ylabel('Y position')
    pylab.title('Fusion of two measurements')
    pylab.legend(('true','measured 1', 'measured 2', 'kalman'))
    pylab.show()
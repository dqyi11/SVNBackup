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
        
    nx = []
    ny = []
    with open('noised_position.txt', 'r') as f2:
        for line in f2:
            pdata = line.split(" ")
            nx.append(float(pdata[0]))
            ny.append(float(pdata[1]))
            
    vx = []
    vy = []
    with open('velocity.txt', 'r') as f3:
        for line in f3:
            pdata = line.split(" ")
            vx.append(float(pdata[0]))
            vy.append(float(pdata[1]))
            
    # This is the state transition vector, which represents part of the kinematics.
    # 1, ts, 0,  0  =>  x(n+1) = x(n) + vx(n)
    # 0,  1, 0,  0  => vx(n+1) =        vx(n)
    # 0,  0, 1, ts  =>  y(n+1) =              y(n) + vy(n)
    # 0,  0, 0,  1  => vy(n+1) =                     vy(n)
    # Remember, acceleration gets added to these at the control vector.
    state_transition = numpy.matrix([[1,timeslice,0,0],[0,1,0,0],[0,0,1,timeslice],[0,0,0,1]])
                
    control_matrix = numpy.matrix([[0,0,0,0],[0,0,0,0],[0,0,1,0],[0,0,0,1]])
    # The control vector, which adds acceleration to the kinematic equations.
    # 0          =>  x(n+1) =  x(n+1)
    # 0          => vx(n+1) = vx(n+1)
    # -9.81*ts^2 =>  y(n+1) =  y(n+1) + 0.5*-9.81*ts^2
    # -9.81*ts   => vy(n+1) = vy(n+1) + -9.81*ts
    control_vector = numpy.matrix([[0],[0],[0.5*-9.81*timeslice*timeslice],[-9.81*timeslice]])
    
    # After state transition and control, here are the equations:
    #  x(n+1) = x(n) + vx(n)
    # vx(n+1) = vx(n)
    #  y(n+1) = y(n) + vy(n) - 0.5*9.81*ts^2
    # vy(n+1) = vy(n) + -9.81*ts
    # Which, if you recall, are the equations of motion for a parabola.  Perfect.
    
    # Observation matrix is the identity matrix, since we can get direct
    # measurements of all values in our example.
    observation_matrix = numpy.eye(4)
    
    speedX = muzzle_velocity*math.cos(angle*math.pi/180)
    speedY = muzzle_velocity*math.sin(angle*math.pi/180)
    # This is our guess of the initial state.  I intentionally set the Y value
    # wrong to illustrate how fast the Kalman filter will pick up on that.
    initial_state = numpy.matrix([[0],[speedX],[500],[speedY]])
    
    initial_probability = numpy.eye(4)

    process_covariance = numpy.zeros(4)
    measurement_covariance = numpy.eye(4)*0.2
    
    iterations = len(x)
    
    kf = KalmanFilterLinear(state_transition, control_matrix, observation_matrix, initial_state, initial_probability, process_covariance, measurement_covariance)
    
    kx = []
    ky = []
    # Iterate through the simulation.
    for i in range(iterations):
        kx.append(kf.GetCurrentState()[0,0])
        ky.append(kf.GetCurrentState()[2,0])
        kf.Step(control_vector,numpy.matrix([[nx[i]],[vx[i]],[ny[i]],[vy[i]]]))
    
                
    # Plot all the results we got.
    pylab.plot(x,y,'-',nx,ny,':',kx,ky,'--')
    pylab.xlabel('X position')
    pylab.ylabel('Y position')
    pylab.title('Measurement of a Cannonball in Flight')
    pylab.legend(('true','measured','kalman'))
    pylab.show()
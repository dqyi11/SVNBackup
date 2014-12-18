'''
Created on Dec 17, 2014

@author: daqing_yi
'''
import random, math

# Simulates the classic physics problem of a cannon shooting a ball in a
# parabolic arc.  In addition to giving "true" values back, you can also ask
# for noisy values back to test Kalman filters.
class Cannon:
    #--------------------------------VARIABLES----------------------------------
    angle = 45 # The angle from the ground to point the cannon.
    muzzle_velocity = 100 # Muzzle velocity of the cannon.
    gravity = [0,-9.81] # A vector containing gravitational acceleration.
    # The initial velocity of the cannonball
    velocity = [muzzle_velocity*math.cos(angle*math.pi/180), muzzle_velocity*math.sin(angle*math.pi/180)]
    loc = [0,0] # The initial location of the cannonball.
    acceleration = [0,0] # The initial acceleration of the cannonball.
    #---------------------------------METHODS-----------------------------------
    def __init__(self,_timeslice,_noiselevel):
        self.timeslice = _timeslice
        self.noiselevel = _noiselevel
    def add(self,x,y):
        return x + y
    def mult(self,x,y):
        return x * y
    def GetX(self):
        return self.loc[0]
    def GetY(self):
        return self.loc[1]
    def GetXWithNoise(self):
        return random.gauss(self.GetX(),self.noiselevel)
    def GetYWithNoise(self):
        return random.gauss(self.GetY(),self.noiselevel)
    def GetXVelocity(self):
        return self.velocity[0]
    def GetYVelocity(self):
        return self.velocity[1]
    # Increment through the next timeslice of the simulation.
    def Step(self):
        # We're gonna use this vector to timeslice everything.
        timeslicevec = [self.timeslice,self.timeslice]
        # Break gravitational force into a smaller time slice.
        sliced_gravity = map(self.mult,self.gravity,timeslicevec)
        # The only force on the cannonball is gravity.
        sliced_acceleration = sliced_gravity
        # Apply the acceleration to velocity.
        self.velocity = map(self.add, self.velocity, sliced_acceleration)
        sliced_velocity = map(self.mult, self.velocity, timeslicevec )
        # Apply the velocity to location.
        self.loc = map(self.add, self.loc, sliced_velocity)
        # Cannonballs shouldn't go into the ground.
        if self.loc[1] < 0:
            self.loc[1] = 0
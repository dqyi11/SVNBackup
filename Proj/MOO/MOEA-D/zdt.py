'''
Created on Oct 21, 2014

@author: daqing_yi
'''

import numpy as np

def zdt1_func(x):
    y = np.zeros(2, np.float)
    y[0] = x[0]
    sum = 0.0
    for i in range(1, 30):
        sum += x[i];
    g = 1 + 9 * (sum / 29.0);
    h = 1 - np.sqrt(x[0]/g);           
    y[1] = g * h;
    return y

def getParetoFrontZDT1():
    
    paretoX = np.arange(0.0,1.0,0.01);
    paretoY = np.zeros(len(paretoX));
    for i in range(len(paretoX)):
        paretoY[i] = 1 - np.sqrt(paretoX[i]);
    paretoFront = np.vstack((paretoX, paretoY))
    
    return paretoFront

def zdt2_func(x):
    y = np.zeros(2, np.float)
    y[0] = x[0]
    sum = 0.0
    for i in range(1, 30):
        sum += x[i];
    g = 1 + 9 * (sum / 29.0)
    h = 1 - (x[0]/g)**2;           
    y[1] = g * h;
    return y

def getParetoFrontZDT2():
    
    paretoX = np.arange(0.0,1.0,0.01);
    paretoY = np.zeros(len(paretoX));
    paretoPos = [];
    for i in range(len(paretoX)):
        paretoY[i] = 1 - paretoX[i]**2;
    paretoFront = np.vstack((paretoX, paretoY))
    
    return paretoFront


def zdt3_func(x):
    y = np.zeros(2, np.float)
    y[0] = x[0]
    sum = 0.0
    for i in range(1, 30):
        sum += x[i];
    g = 1 + 9 * (sum / 29.0);
    h = 1 - np.sqrt(x[0]/g) - x[0]/(g*np.sin(10*np.pi*x[0]))          
    y[1] = g * h;
    return y

def getParetoFrontZDT3():
    
    paretoX = np.arange(0.0,0.0830015349,0.0001)
    paretoX = np.union1d(paretoX, np.arange(0.1822287280, 0.2577623634,0.0001))
    paretoX = np.union1d(paretoX, np.arange(0.4093136748, 0.4538821041,0.0001))
    paretoX = np.union1d(paretoX, np.arange(0.6183967944, 0.6525117038,0.0001))
    paretoX = np.union1d(paretoX, np.arange(0.8233317983, 0.8518328654,0.0001))
    paretoY = np.zeros(len(paretoX))
    for i in range(len(paretoX)):
        paretoY[i] = 1 - np.sqrt(paretoX[i]) - paretoX[i]*np.sin(10*np.pi*paretoX[i]);
    paretoFront = np.vstack((paretoX, paretoY))
    
    return paretoFront
    

def zdt4_func(x):
    y = np.zeros(2, np.float)
    y[0] = x[0]
    sum = 0.0
    for i in range(1, 30):
        sum += x[i]**2 - 10*np.cos(4*np.pi*x[i])
    g = 1 + 10 * 29 + sum
    h = 1 - (x[0]/g)**2     
    y[1] = g * h;
    return y

def getParetoFrontZDT4():
    
    paretoX = np.arange(0.0,1.0,0.0001)
    paretoY = np.zeros(len(paretoX))
    for i in range(len(paretoX)):
        paretoY[i] = 1 - np.sqrt(paretoX[i])
    paretoFront = np.vstack((paretoX, paretoY))
    
    return paretoFront

def zdt6_func(x):
    y = np.zeros(2, np.float)
    y[0] = 1 - np.exp(-4 * x[0]) * ((np.sin(6*np.pi*x[0]))**6)
    sum = 0.0
    for i in range(1, 30):
        sum += x[i]
    g = 1 + 9 * ((sum/29)**0.25)    
    y[1] = 1 - (y[0]/g)**2
    return y

def getParetoFrontZDT6():
    
    paretoX = np.arange(0.2807753191,1.0,0.0001)
    paretoY = np.zeros(len(paretoX))
    for i in range(len(paretoX)):
        paretoY[i] = 1 - paretoX[i]**2
    paretoFront = np.vstack((paretoX, paretoY))
    
    return paretoFront
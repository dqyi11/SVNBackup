'''
Created on 2013-11-11

@author: Walter
'''


import numpy as np;
import matplotlib.pyplot as plt;

if __name__ == '__main__':
    
    A1 = []
    cnt = 0;
    for line in open('C1-posX.txt'):
        A1.append(map(float,line.split(',')))
        cnt = cnt + 1;
        
    A2 = []
    cnt = 0;
    for line in open('C2-posX.txt'):
        A2.append(map(float,line.split(',')))
        cnt = cnt + 1;
        
    A3 = []
    cnt = 0;
    for line in open('C3-posX.txt'):
        A3.append(map(float,line.split(',')))
        cnt = cnt + 1;        
        
    A4 = []
    cnt = 0;
    for line in open('C4-posX.txt'):
        A4.append(map(float,line.split(',')))
        cnt = cnt + 1; 
        
    A5 = []
    cnt = 0;
    for line in open('C5-posX.txt'):
        A5.append(map(float,line.split(',')))
        cnt = cnt + 1;  
        
    figX = plt.figure();
    axX = figX.add_subplot(111);
    
    ind = np.arange(len(A1[0])) * 0.005;
    x1 = axX.plot(ind, A1[0][:],'r', ind, A1[1][:], 'r', label="case 1");
    x2 = axX.plot(ind, A2[0][:], 'g', ind, A2[1][:], 'g', label = "case 2");
    x3 = axX.plot(ind, A3[0][:], 'b', ind, A3[1][:], 'b', label = "case 3");
    x4 = axX.plot(ind, A4[0][:], 'y', ind, A4[1][:], 'y', label = "case 4");
    x5 = axX.plot(ind, A5[0][:len(A1[0])], 'k', ind, A5[1][:len(A1[0])], 'k', label = "case4");
        
    idx = [];
    for i in range(5):
        idx.append("Case " + str(i+1));
    axX.legend([x1, x2, x3, x4, x5], ['red line', 'green line', 'blue line', 'yellow line', 'black line']);
    
    axX.set_xlabel("Time");
    axX.set_ylabel("Position");
    axX.set_title("X");   
    
    plt.show();    
'''
Created on Sep 26, 2013

@author: daqing_yi
'''

import numpy as np
import matplotlib
#matplotlib.use("WXAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    
    'Shapley s game'
    '    || L      || M       || R       '
    ' T  || (0,0)  || (1,0)   || (0,1)  '
    ' M  || (0,1)  || (0,0)   || (1,0)  '
    ' D  || (1,0)  || (0,1)   || (0,0)  '
    
    'Player 1  '
    'Player 2  '
    
    #Belief1 = np.array([1.0,0.0,1.0]);
    #Belief2 = np.array([1.0,0.0,1.0]);
    
    Belief1 = np.array([1.0,0.0,0.0]);
    Belief2 = np.array([1.0,0.0,0.0]);
    
    profile1 = np.zeros(3, np.double);
    profile2 = np.zeros(3, np.double);
    profile1[0] = Belief1[0] / (Belief1[0] + Belief1[1] + Belief1[2]);
    profile1[1] = Belief1[1] / (Belief1[0] + Belief1[1] + Belief1[2]);
    profile1[2] = 1.0 - profile1[0] - profile1[1];  # Belief1[2] / (Belief1[0] + Belief1[1] + Belief1[2]);
    profile2[0] = Belief2[0] / (Belief2[0] + Belief2[1] + Belief2[2]);
    profile2[1] = Belief2[1] / (Belief2[0] + Belief2[1] + Belief2[2]);
    profile2[2] = 1.0 - profile2[0] - profile2[1]; # Belief2[2] / (Belief2[0] + Belief2[1] + Belief2[2]);
    
    def BR1(q1,q2):
        '''
        p1, p2 = 0.0, 0.0;
        ' q1:L, q2:M, 1-q1-q2:R '
        # D > L, T> M, M > R 
        'T'
        p1 = q2;
        'M'
        p2 = 1- q1 - q2;
        return p1, p2;
        '''
        Act1 = "T";
        if q2 >= q1 and q2 >= (1.0 - q1 - q2):
            Act1 = "T";
        elif (1.0 - q1 - q2) >= q1 and (1.0 - q1 - q2) >= q2:
            Act1 = "M";
        elif q1 >= q2 and q1 >= (1.0 - q1 - q2):
            Act1 = "D";
        return Act1;
            
    
    def BR2(p1, p2):
        '''
        q1, q2 = 0.0, 0.0;   
        ' p1:T, p2:R, 1-p1-p2:L '
        # R > T, M > L, D > M
        'L'
        q1 = p2;
        'M'
        q2 = 1- p1 -p2;
        return q1, q2;   
        '''
        Act2 = "L";
        if p1 >= p2 and p1 >= (1.0 - p1 - p2):
            Act2 = "R";
        elif p2 >= p1 and p2 >= (1.0 - p1 - p2):
            Act2 = "L";
        elif (1.0 - p1 - p2) >= p1 and (1.0 - p1 - p2) >= p2:
            Act2 = "M";
        return Act2;
    
    T = 20000;
    
    profileHist1 = np.zeros((3,T), np.double);
    profileHist2 = np.zeros((3,T), np.double);
    
    for i in range(0,T):

        profileHist1[0,i] = profile1[0];
        profileHist1[1,i] = profile1[1];
        profileHist1[2,i] = profile1[2];
        
        profileHist2[0,i] = profile2[0];
        profileHist2[1,i] = profile2[1];
        profileHist2[2,i] = profile2[2];
        
        # player 1 best response
        # what is belief of player2's strategy
        '''
        BR1_p1, BR1_p2 = BR1(profile2[0], profile2[1]);
        rand1 = np.random.random();
        if rand1 <= BR1_p1:
            Belief1[0] = Belief1[0] + 1;
        elif rand1 > BR1_p1 and rand1 <= BR1_p1 + BR1_p2:
            Belief1[1] = Belief1[1] + 1;    
        elif rand1 > BR1_p2 + BR1_p1:
            Belief1[2] = Belief1[2] + 1; 
        '''
        Act1 = BR1(profile2[0], profile2[1]);
        if Act1 == "T":
            Belief1[0] = Belief1[0] + 1.0;
        elif Act1 == "M":
            Belief1[1] = Belief1[1] + 1.0;
        elif Act1 == "D":
            Belief1[2] = Belief1[2] + 1.0;   

        
        # player 2 best response
        # what is belief of player1's strategy
        '''
        BR2_q1, BR2_q2 = BR2(profile1[0], profile1[1]);
        rand2 = np.random.random();
        if rand2 <= BR2_q1:
            Belief2[0] = Belief2[0] + 1;
        elif rand2 > BR2_q1 and rand2 <= BR2_q1 + BR2_q2:
            Belief2[1] = Belief2[1] + 1;
        elif rand2 > BR2_q1 + BR2_q2:
            Belief2[2] = Belief2[2] + 1;
        '''
        Act2 = BR2(profile1[0], profile1[1]);
        if Act2 == "L":
            Belief2[0] = Belief2[0] + 1.0;
        elif Act2 == "M":
            Belief2[1] = Belief2[1] + 1.0;
        elif Act2 == "R":
            Belief2[2] = Belief2[2] + 1.0;

        profile1[0] = Belief1[0] / (Belief1[0] + Belief1[1] + Belief1[2]);
        profile1[1] = Belief1[1] / (Belief1[0] + Belief1[1] + Belief1[2]);
        profile1[2] = 1.0 - profile1[0] - profile1[1];  # Belief1[2] / (Belief1[0] + Belief1[1] + Belief1[2]);
        profile2[0] = Belief2[0] / (Belief2[0] + Belief2[1] + Belief2[2]);
        profile2[1] = Belief2[1] / (Belief2[0] + Belief2[1] + Belief2[2]);
        profile2[2] = 1.0 - profile2[0] - profile2[1]; # Belief2[2] / (Belief2[0] + Belief2[1] + Belief2[2]);
        
    print "Player 1";
    print str(profileHist1[0,T-1]) + " " + str(profileHist1[1,T-1]) + " " + str(profileHist1[2,T-1]);    
    print "Player 2";
    print str(profileHist1[0,T-1]) + " " + str(profileHist1[1,T-1]) + " " + str(profileHist1[2,T-1]);   
    
    fig1 = plt.figure();
    ax1 = fig1.add_subplot(111, projection='3d');
    ax1.plot(profileHist1[0,:], profileHist1[1,:], profileHist1[2,:], "r-");
    x = profileHist1[0,:];
    y = profileHist1[1,:];
    z = profileHist1[2,:];
    #ax1.set_linewidth(0.02);
    #ax1.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1, color='r');
    #ax1.quiver(profileHist1[0,:-1], profileHist1[1,:-1], profileHist1[0,:1] - profileHist1[0,:-1], profileHist1[1,:1] - profileHist1[1,:-1], scale_units='xy', angles='xy', scale=1)
    ax1.set_xlim([-0.1, 1.1]);
    ax1.set_ylim([-0.1, 1.1]);
    #ax1.set_xlabel(r"p");
    #ax1.set_ylabel(r"1-p");
    ax1.set_title("Player 1");
    
    fig2 = plt.figure();
    ax2 = fig2.add_subplot(111, projection='3d');
    ax2.plot(profileHist2[0,:], profileHist2[1,:], profileHist2[2,:], "b-");
    x = profileHist2[0,:];
    y = profileHist2[1,:];
    z = profileHist2[2,:];
    #ax2.set_linewidth(0.02);
    #ax2.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1, color='b');
    ax2.set_xlim([-0.1, 1.1]);
    ax2.set_ylim([-0.1, 1.1]);    
    #ax2.set_xlabel(r"q");
    #ax2.set_ylabel(r"1-q");
    ax2.set_title("Player 2");
    
    it = np.arange(0,T);
    fig3 = plt.figure();
    ax3 = fig3.add_subplot(111);
    ax3.plot(it, profileHist1[0,:], "r-", label="T");
    ax3.plot(it, profileHist1[1,:], "g--", label="M");
    ax3.plot(it, profileHist1[2,:], "b:", label="D");
    #ax3.set_ylim([-0.1,1,1]);
    #ax3.legend("T", "M", "D");
    ax3.legend(loc=1, ncol=3, shadow=True)
    ax3.set_ylabel(r"profile");
    ax3.set_xlabel(r"iteration");
    ax3.set_title("Player 1");
    
    fig4 = plt.figure();
    ax4 = fig4.add_subplot(111);
    ax4.plot(it, profileHist2[0,:], "r-", label="L");
    ax4.plot(it, profileHist2[1,:], "g--", label="M");
    ax4.plot(it, profileHist2[2,:], "b:", label="R");
    #ax4.set_ylim([-0.1,1,1]);
    ax4.legend(loc=1, ncol=3, shadow=True)
    ax4.set_ylabel(r"profile");
    ax4.set_xlabel(r"iteration");
    ax4.set_title("Player 2");
    
    plt.show(); 
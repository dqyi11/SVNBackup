'''
Created on 2013-9-25

@author: Walter
'''

import numpy as np
import matplotlib
#matplotlib.use("WXAgg")
import matplotlib.pyplot as plt

if __name__ == '__main__':

    'coordinate game'
    '    || C       || D '
    ' C  || (2,2)   || (0,0)    '
    ' D  || (0,0)   || (1,1)    '
    
    'Player 1  '
    'Player 2  '
    
    U1 = np.array([[2,0],[0,1]]);
    U2 = np.array([[2,0],[0,1]]);
    
    #Belief1 = np.array([1.0,1.0]);
    #Belief2 = np.array([1.0,1.0]);
    Belief1 = np.array([0.0,1.0]);
    Belief2 = np.array([1.0,0.0]);
    
    profile1 = np.zeros(2, np.double);
    profile2 = np.zeros(2, np.double);
    profile1[0] = Belief1[0] / (Belief1[0] + Belief1[1]);
    profile1[1] = 1.0 - profile1[0]; #Belief1[1] / (Belief1[0] + Belief1[1]);
    profile2[0] = Belief2[0] / (Belief2[0] + Belief2[1]);
    profile2[1] = 1.0 - profile2[0]; #Belief2[1] / (Belief2[0] + Belief2[1]);
    
    def BR1(q):
        p = 0.0;
        Act1 = "C";
        # p(q-1) + (1-q)
        if q <= 1.0/3.0:
            p = 0.0;
            Act1 = "D";
        elif q > 1.0/3.0:
            p = 1.0;
            Act1 = "C"
        #return p;
        return Act1;
    
    def BR2(p):
        q = 0.0;
        Act2 = "C";
        # q(p-1) + (1-p)
        if p <= 1.0/3.0:
            q = 0.0;
            Act2 = "D";
        elif p > 1.0/3.0:
            q = 1.0;
            Act2 = "C";
        #return q; 
        return Act2;   
    
    T = 200
    
    profileHist1 = np.zeros((2,T), np.double);
    profileHist2 = np.zeros((2,T), np.double);
    
    for i in range(0,T):
        #print profile1;
        #print profile2;
        
        #profileHist1.append(profile1);
        profileHist1[0,i] = profile1[0];
        profileHist1[1,i] = profile1[1];
        #profileHist2.append(profile2);
        profileHist2[0,i] = profile2[0];
        profileHist2[1,i] = profile2[1];
        
        # player 1 best response
        # what is belief of player2's strategy
        '''
        BR1_p = BR1(profile2[0]);
        if np.random.random() <= BR1_p:
            Belief1[0] = Belief1[0] + 1;
        else:
            Belief1[1] = Belief1[1] + 1;  
        '''
        P1_act = BR1(profile2[0]);
        if P1_act == "C":
            Belief1[0] = Belief1[0] + 1.0;
        elif P1_act == "D":
            Belief1[1] = Belief1[1] + 1.0;  
            
        #print Belief1;
        
        # player 2 best response
        # what is belief of player1's strategy
        '''
        BR2_q = BR2(profile1[0]);
        if np.random.random() <= BR2_q:
            Belief2[0] = Belief2[0] + 1;
        else:
            Belief2[1] = Belief2[1] + 1;
        '''
        P2_act = BR2(profile1[0]);
        if P2_act == "C":
            Belief2[0] = Belief2[0] + 1.0;
        elif P2_act == "D":
            Belief2[1] = Belief2[1] + 1.0;
             
        
        profile1[0] = Belief1[0] / (Belief1[0] + Belief1[1]);
        profile1[1] = 1.0 - profile1[0]; #Belief1[1] / (Belief1[0] + Belief1[1]);
        profile2[0] = Belief2[0] / (Belief2[0] + Belief2[1]);
        profile2[1] = 1.0 - profile2[0]; #Belief2[1] / (Belief2[0] + Belief2[1]);
        
    
    print "Player 1";
    print str(profileHist1[0,T-1]) + " " + str(profileHist1[1,T-1]);
    print "Player 2";
    print str(profileHist2[0,T-1]) + " " + str(profileHist2[1,T-1]);
        
    
    fig1 = plt.figure();
    ax1 = fig1.add_subplot(111);
    #ax1.plot(profileHist1[0,:], profileHist1[1,:], "r-s");
    x = profileHist1[0,:];
    y = profileHist1[1,:];
    #ax1.set_linewidth(0.02);
    ax1.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1, color='r');
    #ax1.quiver(profileHist1[0,:-1], profileHist1[1,:-1], profileHist1[0,:1] - profileHist1[0,:-1], profileHist1[1,:1] - profileHist1[1,:-1], scale_units='xy', angles='xy', scale=1)
    ax1.set_xlim([-0.1, 1.1]);
    ax1.set_ylim([-0.1, 1.1]);
    ax1.grid();
    ax1.set_xlabel(r"p");
    ax1.set_ylabel(r"1-p");
    ax1.set_title("Player 1");
    
    fig2 = plt.figure();
    ax2 = fig2.add_subplot(111);
    #ax2.plot(profileHist2[0,:], profileHist2[1,:], "b-s");
    x = profileHist2[0,:];
    y = profileHist2[1,:];
    #ax2.set_linewidth(0.02);
    ax2.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1, color='b');
    ax2.set_xlim([-0.1, 1.1]);
    ax2.set_ylim([-0.1, 1.1]);
    ax2.grid();    
    ax2.set_xlabel(r"q");
    ax2.set_ylabel(r"1-q");
    ax2.set_title("Player 2");
    
    it = np.arange(0,T);
    fig3 = plt.figure();
    ax3 = fig3.add_subplot(111);
    ax3.plot(it, profileHist1[0,:], "r-", label="C");
    #ax3.set_ylim([-0.1,1,1]);
    #ax3.legend("T", "M", "D");
    #ax3.legend(loc=1, ncol=3, shadow=True)
    ax3.set_ylabel(r"profile");
    ax3.set_xlabel(r"iteration");
    ax3.set_title("Player 1");
    
    fig4 = plt.figure();
    ax4 = fig4.add_subplot(111);
    ax4.plot(it, profileHist2[0,:], "r-", label="C");
    #ax4.set_ylim([-0.1,1,1]);
    #ax4.legend(loc=1, ncol=1, shadow=True)
    ax4.set_ylabel(r"profile");
    ax4.set_xlabel(r"iteration");
    ax4.set_title("Player 2");
    
    plt.show();    
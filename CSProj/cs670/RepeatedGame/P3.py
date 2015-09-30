'''
Created on 2013-10-26

@author: Walter
'''
if __name__ == '__main__':
    
    import numpy as np;
    import matplotlib.pyplot as plt;
    
    
    ac = np.zeros((999,9), float);
    ad = np.zeros((999,9), float);
    weasel = np.zeros((999,9), float); 

    cnt = 0;
    for line in open('AC_RND.txt'):
        data = line.split(",")  

        ac[cnt, 0] = float(data[0]);
        ac[cnt, 1] = float(data[1]);
        ac[cnt, 2] = float(data[2]);
        ac[cnt, 3] = float(data[3]);
        ac[cnt, 4] = float(data[4]);
        ac[cnt, 5] = float(data[5]);
        ac[cnt, 6] = float(data[6]);
        ac[cnt, 7] = float(data[7]);
        ac[cnt, 8] = float(data[8]);        
        cnt = cnt + 1;
        
    cnt = 0;
    for line in open('AD_RND.txt'):
        data = line.split(",")  

        ad[cnt, 0] = float(data[0]);
        ad[cnt, 1] = float(data[1]);
        ad[cnt, 2] = float(data[2]);
        ad[cnt, 3] = float(data[3]);
        ad[cnt, 4] = float(data[4]);
        ad[cnt, 5] = float(data[5]);
        ad[cnt, 6] = float(data[6]);
        ad[cnt, 7] = float(data[7]);
        ad[cnt, 8] = float(data[8]);
        cnt = cnt + 1;
        
    cnt = 0;
    for line in open('WEASEL_RND.txt'):
        data = line.split(",")  

        weasel[cnt, 0] = float(data[0]);
        weasel[cnt, 1] = float(data[1]);
        weasel[cnt, 2] = float(data[2]);
        weasel[cnt, 3] = float(data[3]);
        weasel[cnt, 4] = float(data[4]);
        weasel[cnt, 5] = float(data[5]);
        weasel[cnt, 6] = float(data[6]);
        weasel[cnt, 7] = float(data[7]);
        weasel[cnt, 8] = float(data[8]);
        cnt = cnt + 1;
        
    meanScore = np.zeros((3,9),float);
    varScore = np.zeros((3,9),float);
    
    for i in np.arange(0,9,1):
        meanScore[0,i] = np.mean(ac[:,i]);
        varScore[0,i] = np.var(ac[:,i]);
        meanScore[1,i] = np.mean(ad[:,i]);
        varScore[1,i] = np.var(ad[:,i]);
        meanScore[2,i] = np.mean(weasel[:,i]);
        varScore[2,i] = np.var(weasel[:,i]);     
        
    print meanScore
    print varScore   
    
    fig = plt.figure();
    ax = fig.add_subplot(111);
    ind = np.arange(9)
    width = 0.25
    rects1 = ax.bar(ind, meanScore[0,:], width,
                color='black',
                yerr=varScore[0,:],
                error_kw=dict(elinewidth=2,ecolor='red'));
                
    rects2 = ax.bar(ind+width, meanScore[1,:], width,
            color='blue',
            yerr=varScore[1,:],
            error_kw=dict(elinewidth=2,ecolor='red'));
            
    rects3 = ax.bar(ind+2*width, meanScore[2,:], width,
            color='yellow',
            yerr=varScore[2,:],
            error_kw=dict(elinewidth=2,ecolor='red'));
            
    # axes and labels
    ax.set_xlim(-width, len(ind) +width)
    ax.set_ylim(0,6.0)
    ax.set_ylabel('average scores')
    #ax.set_title('AD, AC and Weasel againt Random agent with different probabilities of cooperate')
    xTickMarks = [str(s) for s in np.arange(0.1,0.9,0.1)]
    ax.set_xticks(ind+width)
    xtickNames = ax.set_xticklabels(xTickMarks)
    plt.setp(xtickNames, rotation=45, fontsize=10)
    
    ax.legend( (rects1[0], rects2[0], rects3[0]), ('Always Cooperate', 'Always Defect', 'Weasel') )
    
    plt.show();
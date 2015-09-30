'''
Created on 2013-10-25

@author: Walter
'''

if __name__ == '__main__':
    
    import numpy as np;
    import matplotlib.pyplot as plt;
    
    agentName = ["Weasel","AD","Random","AC","TFT","TF2T","NF","Pavlov","WSLS"];
    
    run5 = np.zeros((999,9), float);
    run100 = np.zeros((999,9), float);
    run200 = np.zeros((999,9), float); 

    cnt = 0;
    for line in open('5-run.txt'):
        data = line.split(",")  

        run5[cnt, 0] = float(data[0]);
        run5[cnt, 1] = float(data[1]);
        run5[cnt, 2] = float(data[2]);
        run5[cnt, 3] = float(data[3]);
        run5[cnt, 4] = float(data[4]);
        run5[cnt, 5] = float(data[5]);
        run5[cnt, 6] = float(data[6]);
        run5[cnt, 7] = float(data[7]);
        run5[cnt, 8] = float(data[8]);        
        cnt = cnt + 1;
        
    cnt = 0;
    for line in open('100-run.txt'):
        data = line.split(",")  

        run100[cnt, 0] = float(data[0]);
        run100[cnt, 1] = float(data[1]);
        run100[cnt, 2] = float(data[2]);
        run100[cnt, 3] = float(data[3]);
        run100[cnt, 4] = float(data[4]);
        run100[cnt, 5] = float(data[5]);
        run100[cnt, 6] = float(data[6]);
        run100[cnt, 7] = float(data[7]);
        run100[cnt, 8] = float(data[8]);
        cnt = cnt + 1;
        
    cnt = 0;
    for line in open('200-run.txt'):
        data = line.split(",")  

        run200[cnt, 0] = float(data[0]);
        run200[cnt, 1] = float(data[1]);
        run200[cnt, 2] = float(data[2]);
        run200[cnt, 3] = float(data[3]);
        run200[cnt, 4] = float(data[4]);
        run200[cnt, 5] = float(data[5]);
        run200[cnt, 6] = float(data[6]);
        run200[cnt, 7] = float(data[7]);
        run200[cnt, 8] = float(data[8]);
        cnt = cnt + 1;
        
    meanScore = np.zeros((3,9),float);
    varScore = np.zeros((3,9),float);
    
    for i in np.arange(0,9,1):
        meanScore[0,i] = np.mean(run5[:,i]);
        varScore[0,i] = np.var(run5[:,i]);
        meanScore[1,i] = np.mean(run100[:,i]);
        varScore[1,i] = np.var(run100[:,i]);
        meanScore[2,i] = np.mean(run200[:,i]);
        varScore[2,i] = np.var(run200[:,i]);     
        
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
    #ax.set_title('Weasel vs all agents: scores by 5 runs, 100 runs and 200 runs')
    xTickMarks = [str(s) for s in agentName]
    ax.set_xticks(ind+width)
    xtickNames = ax.set_xticklabels(xTickMarks)
    plt.setp(xtickNames, rotation=45, fontsize=10)
    
    ax.legend( (rects1[0], rects2[0], rects3[0]), ('5 stages', '100 stages', '200 stages') )
    
    plt.show();
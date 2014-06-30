'''
Created on Jan 22, 2014

@author: daqing_yi
'''

from Swarm import *;
from PerformanceCalc import *;

class PerformanceLogger(object):
    '''
    classdocs
    '''

    def __init__(self, swarm):
        '''
        Constructor
        '''
        self.swarm  = swarm;
        self.histHausdorffDistance = [];
        self.histDiversity = [];
        self.histSpread = [];
        self.histNondomPercentage = [];
        
    def log(self):
        
        diversityVal = calcDiversity(self.swarm.nondominatedParticles);
        hd = calcHausdorffDist(self.swarm.nondominatedParticles, self.swarm.nondominatedSet, self.swarm.refere_precise);
        spread = calcSpread(self.swarm.nondominatedParticles, self.swarm);
        nondomPercentage = float(len(self.swarm.nondominatedParticles))/float(len(self.swarm.particles));
        
        self.histDiversity.append(diversityVal);
        self.histHausdorffDistance.append(hd);
        self.histSpread.append(spread);
        self.histNondomPercentage.append(nondomPercentage);
        
        
        
    def plot(self, path=None):
        
        fig1 = plt.figure();
        ax1 = fig1.add_subplot(111);
        datalen = len(self.histDiversity);
        x_data = np.arange(datalen);
        
        ax1.plot(x_data, self.histDiversity)
        
        title1 = "Diversity @ " + str(datalen);

        ax1.set_title(title1);
        
        filename1 = title1 + ".png";
        if path != None:
            filename1 = path + "\\" + filename1;
        plt.savefig(filename1);
        
        fig2 = plt.figure();
        ax2 = fig2.add_subplot(111);
        datalen = len(self.histHausdorffDistance);
        x_data = np.arange(datalen);
        ax2.plot(x_data, self.histHausdorffDistance);
        
        title2 = "Hausdorff distance @ " + str(datalen);
    
        ax2.set_title(title2);
        
        filename2 = title2 + ".png";
        if path != None:
            filename2 = path + "\\" + filename2;
        plt.savefig(filename2);
        
        fig3 = plt.figure();
        ax3 = fig3.add_subplot(111);
        datalen = len(self.histSpread);
        x_data = np.arange(datalen);
        ax3.plot(x_data, self.histSpread);
        
        title3 = "Spread @ " + str(datalen);
    
        ax3.set_title(title3);
        
        filename3 = title3 + ".png";
        if path != None:
            filename3 = path + "\\" + filename3;
        plt.savefig(filename3);
        
        fig4 = plt.figure();
        ax4 = fig4.add_subplot(111);
        datalen = len(self.histNondomPercentage);
        x_data = np.arange(datalen);
        ax4.plot(x_data, self.histNondomPercentage);
        
        title4 = "Nondom Percentage @ " + str(datalen);
    
        ax4.set_title(title4);
        
        filename4 = title4 + ".png";
        if path != None:
            filename4 = path + "\\" + filename4;
        plt.savefig(filename4);
        
    def dump(self, filename, path):
        
        f1 = open(path+"\\"+filename+"-Div.txt", "w");
        for div in self.histDiversity:
            f1.write(str(div)+" ");
        f1.close();
        
        f2 = open(path+"\\"+filename+"-HD.txt", "w");
        for dist in self.histHausdorffDistance:
            f2.write(str(dist)+" ");
        f2.close();
        
        f3 = open(path+"\\"+filename+"-Spread.txt", "w");
        for spread in self.histSpread:
            f3.write(str(spread)+" ");
        f3.close();
        
        f4 = open(path+"\\"+filename+"-Efficiency.txt", "w");
        for eff in self.histNondomPercentage:
            f4.write(str(eff)+" ");
        f4.close();
        
        
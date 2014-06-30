'''
Created on 2013-12-5

@author: Walter
'''

from Swarm import *;

class Swarm1D(Swarm):
    
    def initReferenceSet(self):        
        self.referenceSet = [];
        idxCnt = 0;
        for x in np.arange(-self.worldrange[0]/2, self.worldrange[0]/2, 0.05):    
            ref = Reference();
            ref.pos = [x];
            ref.index = idxCnt;
            self.referenceSet.append(ref);
            idxCnt += 1;
            
        self.categorizeRefSet();
        self.initDomFit();
        self.initNondomFit();
            
    def initDomFit(self):
        for domPos in self.dominatedSet:
            domPos.fit = [];
            domPos.fit.append(self.objfuncs[0](domPos.pos));
            domPos.fit.append(self.objfuncs[1](domPos.pos));
            
    def initNondomFit(self):
        for nondomPos in self.nondominatedSet:
            nondomPos.fit = [];
            nondomPos.fit.append(self.objfuncs[0](nondomPos.pos));
            nondomPos.fit.append(self.objfuncs[1](nondomPos.pos));
            
    def getDomFit(self):
        fit1 = [];
        fit2 = [];
        for domPos in self.dominatedSet:
            fit1.append(domPos.fit[0]);
            fit2.append(domPos.fit[1]);
        return fit1, fit2;
        
    def getNondomFit(self):
        fit1 = [];
        fit2 = [];
        for nondomPos in self.nondominatedSet:
            fit1.append(nondomPos.fit[0]);
            fit2.append(nondomPos.fit[1]);
        return fit1, fit2;
            
    def getXDominate(self):        
        xPos = [];
        for a in self.dominatedSet:
            xPos.append(a.pos[0]);
        return xPos;
        
    def getXNondominate(self):
        xPos = [];
        for a in self.nondominatedSet:
            xPos.append(a.pos[0]);
        return xPos;        
    
    def getDominatedParticlePos(self):        
        xDomParPos = [];
        xDomParIdx = [];
        for p in self.dominatedParticles:
            assert p.nondominated == False;
            xDomParPos.append(p.pos[0,0]);
            xDomParIdx.append(p.index);
        return xDomParPos, xDomParIdx;
    
    def getNondominatedParticlePos(self):
        xNondomParPos = [];
        xNondomParIdx = [];
        for p in self.nondominatedParticles:
            assert p.nondominated == True;
            xNondomParPos.append(p.pos[0,0]);
            xNondomParIdx.append(p.index);
        return xNondomParPos, xNondomParIdx;
                    
        

    def plot(self,count):
        
        fig1 = plt.figure();        
        ax1 = fig1.add_subplot(111);
        
        midValIdx = self.particleNum/2;
        midLine = [];
        for nondom in self.nondominatedSet:
            midLine.append(midValIdx);
        nondomPosX = self.getXNondominate();
        ax1.plot(nondomPosX, midLine, 's', color='#7a7a7a');
        
        '''
        posX = [];
        posY = [];
        for p in self.particles:
            posX.append(p.pos[0,0]);
            posY.append(p.index);
        ax1.plot(posX, posY, 'or');
        '''
        domPX, domPIdx = self.getDominatedParticlePos();
        nondomPX, nondomPIdx = self.getNondominatedParticlePos();
        ax1.plot(domPX, domPIdx, 'o', color='#0000ff');
        ax1.plot(nondomPX, nondomPIdx, 'o', color='#ff0000');
        
        x_fit = [];
        y_fit = [];
        for p in self.particles:
            x_line = [p.pos[0,0], p.pos[0,0] + p.vel[0,0] * self.interval];
            y_line = [p.index, p.index];
            pos = [p.pos[0,0]];
            x_fit.append(self.objfuncs[0](pos));
            y_fit.append(self.objfuncs[1](pos));
            ax1.plot(x_line, y_line, '-b');      
        
        globalbest = self.particles[self.globalbestAgentIdx];
        ax1.plot(globalbest.localbestPos[0,0], self.globalbestAgentIdx, 'ob');
        ax1.plot(self.swarm_centroid[0,0], int(self.particleNum/2), 's', color='orange');
        ax1.set_xlabel("particle position");
        ax1.set_ylabel("particle index");
        title1 = "solution space @ " + str(count);
        ax1.set_title(title1);
        plt.savefig(title1 + ".png");
        
        fig2 = plt.figure();
        ax2 = fig2.add_subplot(111);
        '''
        x_range = [];
        y_range = [];
        for x in np.arange(-self.worldrange[0]/2, self.worldrange[0]/2, 0.05):    
            pos = [x];
            x_range.append(self.objfuncs[0](pos));
            y_range.append(self.objfuncs[1](pos));
        ax2.plot(x_range, y_range, '.r');      
        '''
        domfit1, domfit2 = self.getDomFit();
        nondomfit1, nondomfit2 = self.getNondomFit();
        
        #print str(len(self.dominatedSet)) + " " + str(len(self.nondominatedSet));
        ax2.plot(domfit1, domfit2, '.', color='#aaaaaa');
        ax2.plot(nondomfit1, nondomfit2, '.', color='#7a7a7a');
        ax2.legend(["dominant", "nondominant"])
        
        ax2.plot(x_fit, y_fit, 'ob');
        
        ax2.plot(self.swarm_centroid_fitness[0,0], self.swarm_centroid_fitness[0,1], 's', color='orange');
        ax2.plot(self.average_fitness[0,0], self.average_fitness[0,1], 'x', color='brown');
        ax2.set_xlabel("Fitness 1");
        ax2.set_ylabel("Fitness 2");
        title2 = "fitness space @ " + str(count);
        ax2.set_title(title2);
        plt.savefig(title2 + ".png");
        
        if len(self.histCentroid) > 0 and self.showCentroidHist==True:
            fig3 = plt.figure();
            ax3 = fig3.add_subplot(111);
            idx3 = [];
            ctX = [];
                    
            for i in range(len(self.histCentroid)):
                idx3.append(i);
                ctX.append(self.histCentroid[i][0,0]);
                
            ax3.plot(idx3, ctX);
            ax3.set_xlabel("Iteration");
            ax3.set_ylabel("Position");
            ax3.legend(["Position X"]);
            title3 = "Centroid of " + str(count) + " run";
            ax3.set_title(title3);
            plt.savefig(title3 + ".png");
            
            
        if len(self.histAvgFitness) > 0 and self.showAverageFitness==True: 
            fig4 = plt.figure();
            ax4 = fig4.add_subplot(111);      
            idx4 = []
            avX = [];
            avY = [];

            for i in range(len(self.histAvgFitness)):
                idx4.append(i);
                avX.append(self.histAvgFitness[i][0,0]);
                avY.append(self.histAvgFitness[i][0,1]);   

            ax4.plot(idx4, avX);
            ax4.plot(idx4, avY);
            ax4.set_xlabel("Iteration");
            ax4.set_ylabel("Value");
            ax4.legend(["Function 1","Function 2"]);
            title4 = "Average Fitness of " + str(count) + " run";
            ax4.set_title(title4);
            plt.savefig(title4 + ".png");

        if len(self.histCentroidMaximin) > 0 and self.showMaximinOfCentroid==True:  
            fig5 = plt.figure();
            ax5 = fig5.add_subplot(111);      
            idx5 = np.arange(len(self.histCentroidMaximin));

            ax5.plot(idx4, self.histCentroidMaximin);
            ax5.set_xlabel("Iteration");
            ax5.set_ylabel("Value");            
            title5 = "Maximin Value of Centroid in " + str(count) + " run";
            ax5.set_title(title5);
            plt.savefig(title5 + ".png");
            
        if len(self.histGlobalbestPos) > 0 and self.showGlobalBestPosition==True:
            fig6 = plt.figure();
            ax6 = fig6.add_subplot(111);
            idx6 = [];
            gbX = [];
                    
            for i in range(len(self.histCentroid)):
                idx6.append(i);
                gbX.append(self.histGlobalbestPos[i][0]);
                
            ax6.plot(idx6, gbX);
            ax6.set_xlabel("Iteration");
            ax6.set_ylabel("Position");
            ax6.legend(["Position X"]);
            title6 = "Global Best Position of " + str(count) + " run";
            ax6.set_title(title6);
            plt.savefig(title6 + ".png");
        
        #plt.show();
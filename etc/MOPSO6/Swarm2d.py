'''
Created on 2013-12-5

@author: Walter
'''

from Swarm import *;
from mpl_toolkits.mplot3d import Axes3D;
          
class Swarm2D(Swarm):
    
    def initReferenceSet(self, loadFromFile=False, nondomSetFile=None, domSetFile=None):        
        self.referenceSet = [];
        idxCnt = 0;
        for x in np.arange(-self.worldrange[0]/2, self.worldrange[0]/2, 0.05):
            for y in np.arange(-self.worldrange[1]/2, self.worldrange[1]/2, 0.05):  
                ref = Reference(self.particleDimension, idxCnt);
                ref.pos[0,0] = x;
                ref.pos[0,1] = y;
                self.referenceSet.append(ref);
                idxCnt += 1;
            
        self.categorizeRefSet(loadFromFile, nondomSetFile, domSetFile);
        self.initDomFit();
        self.initNondomFit();
        
        self.refere_precise = 0.05;
        
            
    def initDomFit(self):
        for domPos in self.dominatedSet:
            domPos.fit = [];
            domPos.fit.append(self.calcObjFunc(domPos.pos, 0));
            domPos.fit.append(self.calcObjFunc(domPos.pos, 1));
            
    def initNondomFit(self):
        for nondomPos in self.nondominatedSet:
            nondomPos.fit = [];
            nondomPos.fit.append(self.calcObjFunc(nondomPos.pos, 0));
            nondomPos.fit.append(self.calcObjFunc(nondomPos.pos, 1));
            
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
            
    def getXYDominate(self):        
        xPos = [];
        yPos = []
        for a in self.dominatedSet:
            xPos.append(a.pos[0,0]);
            yPos.append(a.pos[0,1]);
        return xPos, yPos;
        
    def getXYNondominate(self):
        xPos = [];
        yPos = []
        for a in self.nondominatedSet:
            xPos.append(a.pos[0,0]);
            yPos.append(a.pos[0,1]);
        return xPos, yPos;   
    
    def getDominatedParticlePos(self):        
        xDomParPos = [];
        yDomParPos = [];
        for p in self.dominatedParticles:
            assert p.nondominated == False;
            xDomParPos.append(p.pos[0,0]);
            yDomParPos.append(p.pos[0,1]);
        return xDomParPos, yDomParPos;
    
    def getNondominatedParticlePos(self):
        xNondomParPos = [];
        yNondomParPos = [];
        for p in self.nondominatedParticles:
            assert p.nondominated == True;
            xNondomParPos.append(p.pos[0,0]);
            yNondomParPos.append(p.pos[0,1]);
        return xNondomParPos, yNondomParPos; 
        
    def plot(self, count, path=None):
            
        self.logger.plot(path);
            
        fig1 = plt.figure();
        ax1 = fig1.add_subplot(111);

        
        domPosX, domPosY = self.getXYDominate();
        ax1.plot(domPosX, domPosY, 's', color = '#aaaaaa');
        nondomPosX, nondomPosY = self.getXYNondominate();
        ax1.plot(nondomPosX, nondomPosY, 's', color='#7a7a7a');
        ax1.legend(['dom', 'nondom'])
        
        '''
        posX = [];
        posY = [];
        for p in self.particles:
            posX.append(p.pos[0,0]);
            posY.append(p.pos[0,1]);

        ax1.plot(posX, posY, 'or');
        '''
        domPX, domPY = self.getDominatedParticlePos();
        nondomPX, nondomPY = self.getNondominatedParticlePos();
        ax1.plot(domPX, domPY, 'o', color='#0000ff');
        ax1.plot(nondomPX, nondomPY, 'o', color='#ff0000');
    
        
        #globalbest = self.particles[self.globalbestAgentIdx];
        #ax1.plot(globalbest.localbestPos[0,0], globalbest.localbestPos[0,1], 'ob');
        ax1.plot(self.globalbestPos[0,0], self.globalbestPos[0,1], 'o', color='yellow');
        ax1.set_xlabel("particle x position");
        ax1.set_ylabel("particle y position");
        title1 = "2D solution space @ " + str(count);
        ax1.set_title(title1);
        filename1 = title1 + ".png";
        if path != None:
            filename1 = path + "\\" + filename1;
        plt.savefig(filename1); 
        
        fig2 = plt.figure();
        ax2 = fig2.add_subplot(111);
        
        domx_fit = [];
        domy_fit = [];
        for p in self.dominatedParticles:
            domx_line = [p.pos[0,0], p.pos[0,0] + p.vel[0,0] * self.interval];
            domy_line = [p.pos[0,1], p.pos[0,1] + p.vel[0,1] * self.interval];
            pos = [p.pos[0,0], p.pos[0,1]];
            domx_fit.append(self.objfuncs[0](pos));
            domy_fit.append(self.objfuncs[1](pos));
            ax1.plot(domx_line, domy_line, '-b');
        
        nondomx_fit = [];
        nondomy_fit = [];
        for p in self.nondominatedParticles:
            nondomx_line = [p.pos[0,0], p.pos[0,0] + p.vel[0,0] * self.interval];
            nondomy_line = [p.pos[0,1], p.pos[0,1] + p.vel[0,1] * self.interval];
            pos = [p.pos[0,0], p.pos[0,1]];
            nondomx_fit.append(self.objfuncs[0](pos));
            nondomy_fit.append(self.objfuncs[1](pos));
            ax1.plot(nondomx_line, nondomy_line, '-b');
    
        domfit1, domfit2 = self.getDomFit();
        nondomfit1, nondomfit2 = self.getNondomFit();
        
        #print str(len(self.dominatedSet)) + " " + str(len(self.nondominatedSet));
        ax2.plot(domfit1, domfit2, '.', color='#aaaaaa');
        ax2.plot(nondomfit1, nondomfit2, '.', color='#7a7a7a');
        ax2.legend(['dom', 'nondom']);
        
        '''
        x_range = [];
        y_range = [];
        for x in np.arange(-self.worldrange[0]/2, self.worldrange[0]/2, 0.05):
            for y in np.arange(-self.worldrange[1]/2, self.worldrange[1]/2, 0.05):
                pos = [x, y];
                x_range.append(self.objfuncs[0](pos));
                y_range.append(self.objfuncs[1](pos));
        ax2.plot(x_range, y_range, '.r');
        '''     
        ax2.plot(domx_fit, domy_fit, 'ob');
        ax2.plot(nondomx_fit, nondomy_fit, 'og');
        
        ax2.set_xlabel("Fitness 1");
        ax2.set_ylabel("Fitness 2");
        title2 = "2D fitness space @ " + str(count);
        ax2.set_title(title2);
        filename2 = title2 + ".png";
        if path != None:
            filename2 = path + "\\" + filename2;
        plt.savefig(filename2); 
        
        
        if self.gbSet.kernel != None:
            fig11 = plt.figure();
            ax11 = fig11.add_subplot(111, projection='3d');
            valX, valY = np.mgrid[-self.worldrange[0]/2:self.worldrange[0]/2:100j, -self.worldrange[1]/2:self.worldrange[1]/2:100j];
            vals = np.vstack([valX.ravel(), valY.ravel()]).T
            y_pred, sigma2_pred = self.gbSet.kernel.predict(vals, eval_MSE=True);
            ax11.scatter(vals[:,0], vals[:,1], y_pred);            
            title11 = "2D  global estimated distribution " + str(count) + "run";
            title11add = str(len(self.gbSet.nondomObs)) + "/" + str(len(self.gbSet.domObs));
            ax11.set_title(title11 + " " + title11add);
            filename11 = title11 + ".png";
            if path != None:
                filename11 = path + "\\" + filename11;
            plt.savefig(filename11);
        
            
        #plt.show();
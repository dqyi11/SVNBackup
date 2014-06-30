'''
Created on Jan 24, 2014

@author: daqing_yi
'''

from Swarm import *;

class SwarmND(Swarm):
    '''
    classdocs
    '''

    def setSampleSize(self, size):
        self.sampleSize = size;
        
    def regenCandx(self):
        
        self.gbSet.randRefSet(self.sampleSize)
        for p in self.particles:
            p.pbSet.randRefSet(self.sampleSize)
            
    def update(self):
        self.regenCandx();
        Swarm.update(self);
        
    
    def plot(self,count,path=None):
        
        fig1 = plt.figure();        
        ax1 = fig1.add_subplot(111);
        
        domx_fit = [];
        domy_fit = [];
        for p in self.dominatedParticles:
            domx_fit.append(self.calcObjFunc(p.pos, 0));
            domy_fit.append(self.calcObjFunc(p.pos, 1));
            
        nondomx_fit = [];
        nondomy_fit = [];
        for p in self.nondominatedParticles:
            nondomx_fit.append(self.calcObjFunc(p.pos, 0));
            nondomy_fit.append(self.calcObjFunc(p.pos, 1));
        
        ax1.plot(self.paretoX, self.paretoY, '.r');
        if self.localParetoX != None:
            ax1.plot(self.localParetoX, self.localParetoY, '.c');
        ax1.plot(domx_fit, domy_fit, 'ob');
        ax1.plot(nondomx_fit, nondomy_fit, 'og');
        ax1.set_xlabel("fitness 1");
        ax1.set_ylabel("fitness 2");
        title1 = "evaluation space @ " + str(count);
        ax1.set_title(title1);
        filename1 = title1 + ".png";
        if path != None:
            filename1 = path + "\\" + filename1;
        plt.savefig(filename1); 
        
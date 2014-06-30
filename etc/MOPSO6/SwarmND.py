'''
Created on Jan 24, 2014

@author: daqing_yi
'''

from Swarm import *;

class SwarmND(Swarm):
    '''
    classdocs
    '''
    
    def plot(self,count,path=None):
        
        self.logger.plot(path);
        
        
        fig1 = plt.figure();        
        ax1 = fig1.add_subplot(111);
        
        domx_fit = [];
        domy_fit = [];
        for p in self.dominatedParticles:
            domx_line = [p.pos[0,0], p.pos[0,0] + p.vel[0,0] * self.interval];
            domy_line = [p.index, p.index];
            pos = [p.pos[0,0]];
            domx_fit.append(self.objfuncs[0](pos));
            domy_fit.append(self.objfuncs[1](pos));
            ax1.plot(domx_line, domy_line, '-b');
        
        nondomx_fit = [];
        nondomy_fit = [];
        for p in self.nondominatedParticles:
            nondomx_line = [p.pos[0,0], p.pos[0,0] + p.vel[0,0] * self.interval];
            nondomy_line = [p.index, p.index];
            pos = [p.pos[0,0]];
            nondomx_fit.append(self.objfuncs[0](pos));
            nondomy_fit.append(self.objfuncs[1](pos));
            ax1.plot(nondomx_line, nondomy_line, '-b');   
        
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
        
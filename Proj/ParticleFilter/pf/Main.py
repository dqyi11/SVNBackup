import numpy
import scipy
import math
import random
import sys


if __name__ == "__main__":
    firstRound = True
    xprev = 0; yprev = 0
     
    for i in range(1, 22): #main for loop
        if firstRound == True:  #1st round --> calculate the x0 and y0    
            xprev = numpy.random.normal(0, 1)
            yprev = numpy.random.normal(0, 1)    
            firstRound = False  
        else: # 2nd round and on --> keep calculating values x and y
            xlist = []; ylist = [];
            wlist = []
            cum = []
            sum = 0
            rexlist = []; reylist = []
                    
            for j in range(0, 100):  #transition and weight calculation
                d = numpy.random.normal(5, 1)
                theta = numpy.random.uniform(math.pi/5 - math.pi/36, math.pi/5 + math.pi/36)
                xnow = xprev+ d * math.cos(theta)
                ynow = yprev + d * math.sin(theta)
                da = math.sqrt(math.pow((-100-xnow), 2) + math.pow((100-ynow), 2))
                db = math.sqrt(math.pow((150-xnow), 2) + math.pow((90-ynow), 2))
                ra = numpy.random.normal(da, 1)
                rb = numpy.random.normal(db, 1)
                likelihooda = math.exp(-(math.pow(ra - da, 2) / 2))
                likelihoodb = math.exp(-(math.pow(rb - db, 2) / 2))
                weight = likelihooda * likelihoodb
                xlist.append(xnow)
                ylist.append(ynow)
                wlist.append(weight)
                cum.append([sum, sum+weight]) 
                sum+=weight
                
            #re-sampling according to the weight
            for j in range(0, 100):
                num = random.uniform(0, sum)
                for k in range(0, len(cum)):
                    low = cum[k][0]
                    high = cum[k][1]
                    if num < high and num > low:
                        rexlist.append(xlist[k])
                        reylist.append(ylist[k])
                        
            #calculate means for x and y
            meanx = 0; meany = 0;
            for j in range(0, 100):
                meanx += rexlist[j]
                meany += reylist[j]
            meanx /= 100
            meany /= 100
            
            #calculate variances for x and y
            varx = 0; vary = 0;
            for j in range(0, 100):
                varx += math.pow(meanx - rexlist[j], 2)
                vary += math.pow(meany - reylist[j], 2)
            varx /= 100
            vary /= 100
            
            #print out the results
            print meanx, "&", meany, "&", varx, "&", vary
                 
            #prep for the next round
            num = random.randint(0, 100)
            xprev = rexlist[num]; yprev = reylist[num];
            xlist = []; ylist = [];
            wlist = []
            rexlist = []; reylist = [];
            sum = 0
                
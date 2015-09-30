'''
Created on 2013-6-4

@author: Walter
'''
from BayesNet import *
from BayesNode import *
import time

if __name__ == '__main__':
    
    samples = []
    for line in open('professionalGolfer-013024.txt'):
        instance = dict([])
        exec('instance='+line)
        samples.append(instance)
        
    golfData = []
    golfmen = []
    tournaments = []
    for line in open('golfdataR.dat'):
        data = dict([])
        elements = line.split()
        data["Golfmen"] = elements[0]
        data["Score"] = float(elements[1])
        data["Tournament"] = elements[2]
        if (elements[0] in golfmen) == False:
            golfmen.append(elements[0])
        if (elements[2] in tournaments) == False:
            tournaments.append(elements[2])
        golfData.append(data)
    print "GolfData:{}".format(len(golfData))
    print "Golfmen:{}".format(len(golfmen))
    print "Tournamenets:{}".format(len(tournaments))
    
    def addFunc(a, b):
        return a+b
    
    Net = BayesNet()
    HyperGolferVar = InvGammaNode(var="Hypergolfervar", alpha=18, beta=0.015)
    HyperTournMean = NormalNode(var="Hypertournmean", mean=72, variance=2)
    HyperTournVar  = InvGammaNode(var="Hypertournvar", alpha=18, beta=0.015)
    Obsvar         = InvGammaNode(var="Obsvar",alpha=83, beta=0.0014)
    
    Net.addNode(HyperGolferVar)
    Net.addNode(HyperTournMean)
    Net.addNode(HyperTournVar)
    Net.addNode(Obsvar)

    tournamentNode = []
    idx = 0
    for tn in tournaments:
        tournamentNode.append(NormalNode(var=tn, mean="Hypertournmean", variance="Hypertournvar"))
        Net.addNode(tournamentNode[idx])
        idx += 1
    
    golfmenNode = []
    idx = 0
    for gm in golfmen:
        golfmenNode.append(NormalNode(var=gm, mean=0.0, variance="Hypergolfervar"))
        Net.addNode(golfmenNode[idx])
        idx += 1

    golfDataNode = []
    idx = 0
    golfDataObserve = dict([])
    for gd in golfData:
        data = golfData[idx]
        golfermean = data["Golfmen"]
        tournamentmean = data["Tournament"]
        gdvar = golfermean + "+" + tournamentmean
        golfDataNode.append(NormalNode(var=gdvar, mean=[golfermean,tournamentmean], variance="Obsvar", meanFunc=addFunc))
        Net.addNode(golfDataNode[idx])
        golfDataObserve[gdvar] = float(data["Score"])
        idx += 1
        
    golferTnCnt = dict([])
    tnGolferCnt = dict([])
    golferAvg = dict([])
    tnAvg = dict([])
    
    for tn in tournaments:
        tnGolferCnt[tn] = 0
        tnAvg[tn] = 0.0
    for gm in golfmen:
        golferTnCnt[gm] = 0
        golferAvg[gm] = 0.0
    for gd in golfData:
        tnGolferCnt[gd["Tournament"]] += 1
        tnAvg[gd["Tournament"]] += gd["Score"]
    for tn in tournaments:
        tnAvg[tn] = tnAvg[tn]/float(tnGolferCnt[tn])
    for gd in golfData:
        golferTnCnt[gd["Golfmen"]] += 1
        golferAvg[gd["Golfmen"]] += gd["Score"] - tnAvg[gd["Tournament"]]
    for gm in golfmen:
        golferAvg[gm] = golferAvg[gm]/float(golferTnCnt[gm])     
    
    #Net.plot("professionalGolfer", plotformat="svg")
    
    print "start sampling"
    
    sampleNum  = 5000
    sampleFrom = 1000
    initial = dict([])
    candsd = dict([])
    initial["Hypergolfervar"] = 3.5
    candsd["Hypergolfervar"] = 0.5
    initial["Hypertournmean"] = 72.8
    candsd["Hypertournmean"] = 0.5
    initial["Hypertournvar"] = 3
    candsd["Hypertournvar"] = 0.5
    initial["Obsvar"] = 3.1
    candsd["Obsvar"] = 0.5
    for tn in tournaments:
        initial[tn] = tnAvg[tn]
        candsd[tn] = 0.5
    for gm in golfmen:
        initial[gm] = golferAvg[gm]
        candsd[gm] = 0.5
    for gd in golfData:
        initial[gd["Golfmen"] + "+" + gd["Tournament"]] = gd["Score"]
        candsd[gd["Golfmen"] + "+" + gd["Tournament"]] = 0.5
        
    q = Query(golfmen, golfDataObserve)
    q.analyzeSamples(samples)
'''
Created on Jan 26, 2014

@author: daqing_yi
'''

if __name__ == '__main__':
    
    from PerformanceAnalyzer import *;
    import sys;
    
    trial_time = 30;
    figFolder = sys.path[0] + "\\zdt2";
    
    caseName = "ZDT2";
    
    fileList1 = [];
    fileList2 = [];
    fileList3 = [];
    fileList4 = [];
    for tt in range(trial_time):
        filename1 = "ZDT2-"+str(tt)+"--Div.txt";
        filename2 = "ZDT2-"+str(tt)+"--AD.txt";
        filename3 = "ZDT2-"+str(tt)+"--Spread.txt";
        filename4 = "ZDT2-"+str(tt)+"--Efficiency.txt";
        fileList1.append(filename1);
        fileList2.append(filename2);
        fileList3.append(filename3);
        fileList4.append(filename4);
    
    analyzer1 = PerformanceAnalyzer(fileList1, figFolder, "Diversity", 10);
    analyzer1.genData();
    analyzer1.plot(caseName);
    analyzer1.dump(caseName);
    
    analyzer2 = PerformanceAnalyzer(fileList2, figFolder, "Distance", 10);
    analyzer2.genData();
    analyzer2.plot(caseName);
    analyzer2.dump(caseName);
    
    analyzer3 = PerformanceAnalyzer(fileList3, figFolder, "Spread", 10);
    analyzer3.genData();
    analyzer3.plot(caseName);
    analyzer3.dump(caseName);
    
    analyzer4 = PerformanceAnalyzer(fileList4, figFolder, "Efficiency", 10);
    analyzer4.genData();
    analyzer4.plot(caseName);
    analyzer4.dump(caseName);
'''
Created on 2013-6-11

@author: Walter
'''

if __name__ == '__main__':
    
    data = []
    for line in open('alarm10.txt'):
        instance = dict([])
        exec('instance='+line)
        data.append(instance)
        
    B_T = []
    E_T = []
    ABE_TTT = [] 
    ABE_TTF = [] 
    ABE_TFT = [] 
    ABE_TFF = []
    JA_TT = [] 
    JA_TF = [] 
    MA_TT = [] 
    MA_TF = []
    
    for d in data:
        B_T.append(d["Burglary:T"])
        E_T.append(d["Earthquake:T"])
        ABE_TTT.append(d["Alarm:T|Burglary:T,Earthquake:T"])
        ABE_TTF.append(d["Alarm:T|Burglary:T,Earthquake:F"])
        ABE_TFT.append(d["Alarm:T|Burglary:F,Earthquake:T"])
        ABE_TFF.append(d["Alarm:T|Burglary:F,Earthquake:F"])
        JA_TT.append(d["JohnCalls:T|Alarm:T"])
        JA_TF.append(d["JohnCalls:T|Alarm:F"])
        MA_TT.append(d["MaryCalls:T|Alarm:T"])
        MA_TF.append(d["MaryCalls:T|Alarm:F"])
        
    filename = 'alarm10-pars.txt'
    fileWriter = open(filename, 'w')
    fileWriter.write("B_T = "+str(B_T)+"\n")
    fileWriter.write("E_T = "+str(E_T)+"\n")
    fileWriter.write("ABE_TTT = "+str(ABE_TTT)+"\n")
    fileWriter.write("ABE_TTF = "+str(ABE_TTF)+"\n")
    fileWriter.write("ABE_TFT = "+str(ABE_TFT)+"\n")
    fileWriter.write("ABE_TFF = "+str(ABE_TFF)+"\n")
    fileWriter.write("JA_TT = "+str(JA_TT)+"\n")
    fileWriter.write("JA_TF = "+str(JA_TF)+"\n")
    fileWriter.write("MA_TT = "+str(MA_TT)+"\n")
    fileWriter.write("MA_TF = "+str(MA_TF)+"\n")
    fileWriter.close()
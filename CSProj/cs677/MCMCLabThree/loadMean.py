'''
Created on 2013-6-10

@author: Walter
'''

if __name__ == '__main__':
    
    data = []
    for line in open('facultyEval.txt'):
        instance = dict([])
        exec('instance='+line)
        data.append(instance)
        
    mean = []
    variance = []
    for d in data:
        mean.append(d['Mean'])
        variance.append(d['Variance'])
        
    filename = 'f5m.txt'
    fileWriter = open(filename, 'w')
    fileWriter.write(str(mean))
    fileWriter.close()
    
    filename2 = 'f5v.txt'
    fileWriter2 = open(filename2, 'w')
    fileWriter2.write(str(variance))
    fileWriter2.close()
    
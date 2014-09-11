'''
Created on Sep 10, 2014

@author: daqing_yi
'''

import csv

def writeToCsv(filename, data):
    
    with open(filename, 'wb') as fp:
        a = csv.writer(fp, delimiter=',')
        a.writerows(data)
    
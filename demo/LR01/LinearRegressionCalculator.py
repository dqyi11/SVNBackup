import csv
import numpy as np

class LinearRegressionCalculator(object):

    def __init__(self):
        self.inputs = []
        self.outputs = []
        
        self.beta0 = 0.0
        self.beta1 = 0.0
        
        self.mle = 0.0
        
    
    def load(self, filename):        
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.inputs.append(float(row[0]))
                self.outputs.append(float(row[1]))
                
    def calc(self):        
        input_mean = np.mean(self.inputs)
        output_mean = np.mean(self.outputs)
        
        var_x = 0.0
        var_xy = 0.0
        for i in range(len(self.inputs)):
            var_xy += (self.outputs[i] - output_mean)*(self.inputs[i] - input_mean)
            var_x += (self.inputs[i] - input_mean)**2
        self.beta1 = var_xy / var_x
        self.beta0 = output_mean - self.beta1 * input_mean
        
        totalVal = 0.0
        for i in range(len(self.inputs)):
            totalVal += (self.outputs[i] - self.beta1 * self.inputs[i] - self.beta0)**2
        self.mle = totalVal / len(self.inputs)
        
                
    
            
            
import numpy as np

class NeuralNetwork(object):

    def __init__(self, shape):
        
        self.input_num = shape[0]
        self.hidden_num = shape[1]
        self.output_num = shape[2]
        
        self.weight_num = self.input_num * self.hidden_num + self.hidden_num * self.output_num
        self.weights = np.zeros(self.weight_num)
        
        self.bias_num = self.hidden_num + self.output_num
        self.biases = np.zeros(self.bias_num)
        
    
    def calcFunc(self, input):
        
        hiVal = np.zeros(self.hidden_num)
        for h in range(self.hidden_num):
            for i in range(self.input_num):
                hiVal[h] += input[i] * self.weights[i + h*self.input_num]
        hoVal = np.zeros(self.hidden_num)
        for h in range(self.hidden_num):
            hoVal[h] = 1.0/(1.0+np.exp(-hiVal[h]))
            
        oi = 0.0
        for o in range(self.output_num):
            for h in range(self.hidden_num):
                oi += hoVal[h] * self.weights[self.input_num * self.hidden_num]
                
        output = np.zeros(self.output_num)
        for o in range(self.output_num):
            output[o] = 1.0/(1.0+np.exp(-hiVal[h]))
            
        return output
    

            
            
        
        
        
        
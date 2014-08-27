import csv

class LinearRegressionCalculator(object):

    def __init__(self, dimension):
        self.dim = dimension
        self.inputs = []
        for d in range(self.dim):
            self.inputs.append([])
        self.outputs = []
        
    
    def load(self, filename):
        
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                for d in range(self.dim):
                    self.inputs[d].append(float(row[d]))
                self.outputs.append(float(row[self.dim]))
            
            
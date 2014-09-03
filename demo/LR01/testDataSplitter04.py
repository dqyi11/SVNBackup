from DataSplitter import *

if __name__ == '__main__':
    
    ds = DataSplitter("nn_data.csv")
    ds.dump("nn_data", 0.1)
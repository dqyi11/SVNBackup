from DataSplitter import *

if __name__ == '__main__':
    
    ds = DataSplitter("data1.csv")
    ds.dump("data1", 0.1)
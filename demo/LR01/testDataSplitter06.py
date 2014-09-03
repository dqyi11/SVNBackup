from DataSplitter import *

if __name__ == '__main__':
    
    ds = DataSplitter("data2.csv")
    ds.dump("data2", 0.1)
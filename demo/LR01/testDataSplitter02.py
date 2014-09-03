from DataSplitter import *

if __name__ == '__main__':
    
    ds = DataSplitter("testData-30.csv")
    ds.dump("testData-30", 0.2)
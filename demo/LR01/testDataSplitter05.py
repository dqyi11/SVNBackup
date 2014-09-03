from DataSplitter import *

if __name__ == '__main__':
    
    ds = DataSplitter("auto_mpg.csv")
    ds.dump("auto_mpg", 0.1)
from DataSplitter import *

if __name__ == '__main__':
    
    ds = DataSplitter("auto_mpg-norm.csv")
    ds.dump("auto_mpg-norm", 0.1)
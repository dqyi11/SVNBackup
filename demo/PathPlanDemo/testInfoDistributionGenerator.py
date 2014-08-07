from InfoDistributionGenerator import *
from LabelManager import *

if __name__ == '__main__':
    
    mgr = LabelManager()
    gnr = InfoDistributionGenerator(mgr, 500)
    gnr.runCFD('two_houses.dat', 'two_house_source.dat', 'testData')
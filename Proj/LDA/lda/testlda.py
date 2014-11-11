'''
Created on 2014-11-10

@author: Walter
'''

from documentGenerator import *
from LDASampler import *

if __name__ == '__main__':
    
    TOPIC_NUM = 10
    DOCUMENT_LENGTH = 100
    FOLDER = "data"
    
    width = TOPIC_NUM / 2
    vocab_size = width ** 2    
    docGen = documentGenerator(TOPIC_NUM, vocab_size)
    matrix = docGen.generateDocuments()
    
    sampler = LDASampler(TOPIC_NUM)
    for it, phi in enumerate(sampler.run(matrix)):
        print "Iteration", it
        print "Likelihood", sampler.loglikelihood()
 
        if it % 5 == 0:
            for z in range(TOPIC_NUM):
                sampler.save_document_image("data/topic%d-%d.png" % (it,z),
                                    phi[z,:].reshape(width,-1))
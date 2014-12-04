'''
Created on 2014-11-10

@author: Walter
'''

from documentGenerator import *
from LDASampler import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    TOPIC_NUM = 10
    DOCUMENT_LENGTH = 200
    DOCUMENT_NUM = 500
    ITERATION_NUM = 30
    FOLDER = "data"
    
    docGen = documentGenerator(TOPIC_NUM, DOCUMENT_LENGTH)
    matrix = docGen.generateDocuments(DOCUMENT_NUM)
    
    #np.savetxt('worldList.txt', docGen.word_list, delimiter= ' ' ,fmt='%1.1f' )
    
    for z in range(TOPIC_NUM):
        data = docGen.word_list[z,:].reshape(docGen.width,-1)
        height, width = data.shape
        zoom = np.ones((width*2, width*2))
        # imsave scales pixels between 0 and 255 automatically
        plt.imsave('ref/topic'+str(z)+'.png', np.kron(data,zoom))
        
    
    #np.savetxt("docGen.txt", matrix, delimiter= ', ' ,fmt='%1.f')
    
    sampler = LDASampler(TOPIC_NUM)
    for it, phi in enumerate(sampler.run(matrix, ITERATION_NUM)):
        print "Iteration", it
        print "Likelihood", sampler.loglikelihood()
 
        if it % 5 == 0:
            for z in range(TOPIC_NUM):
                sampler.save_document_image("data/topic%d-%d.png" % (it,z),
                                    phi[z,:].reshape(docGen.width,-1))
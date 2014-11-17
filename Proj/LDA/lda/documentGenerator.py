'''
Created on 2014-11-10

@author: Walter
'''
import numpy as np

class documentGenerator(object):


    def __init__(self, topic_num, document_length):
        self.topic_num = topic_num
        self.document_length = document_length
        self.alpha = 0.1
        
        # generate world distribution for each topic
        self.width = self.topic_num / 2
        self.vocab_size = self.width ** 2
        self.word_list = np.zeros((self.topic_num, self.vocab_size))
 
        for k in range(self.width):
            self.word_list[k,:] = self.vertical_topic(self.width, k, self.document_length)
 
        for k in range(self.width):
            self.word_list[k+self.width,:] = self.horizontal_topic(self.width, k, self.document_length)
            
        # turn counts into probabilities
        self.word_list /= self.word_list.sum(axis=1)[:, np.newaxis] 
        
        
    def generateDocument(self):
        """
        Generate a document:
        """
        # 1) Sample topic proportions from the Dirichlet distribution.
        theta = np.random.mtrand.dirichlet([self.alpha] * self.topic_num)
        v = np.zeros(self.vocab_size)
        for n in range(self.document_length):
            # 2) Sample a topic index from the Multinomial with the topic proportions from 1).
            z = self.sample_index(theta)
            # 3) Sample a word from the Multinomial corresponding to the topic index from 2).
            w = self.sample_index(self.word_list[z,:])
            v[w] += 1
        return v
    
    def generateDocuments(self, num):
        """
        Generate a document-term matrix.
        """
        m = np.zeros((num, self.vocab_size))
        for i in xrange(num):
            m[i, :] = self.generateDocument()
        return m
    
    def vertical_topic(self, width, topic_index, document_length):
        # Generate a topic whose words form a vertical bar.
        m = np.zeros((width, width))
        m[:, topic_index] = int(document_length / width)
        return m.flatten()
 
    def horizontal_topic(self, width, topic_index, document_length):
        # Generate a topic whose words form a horizontal bar.
        m = np.zeros((width, width))
        m[topic_index, :] = int(document_length / width)
        return m.flatten()
    
    def sample_index(self, p):
        # Sample from the Multinomial distribution and return the sample index.
        return np.random.multinomial(1,p).argmax()

        
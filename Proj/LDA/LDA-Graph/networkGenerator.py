'''
Created on 2014-11-10

@author: Walter
'''
import numpy as np

class networkGenerator(object):


    def __init__(self, community_num, neighbor_num):
        self.community_num = community_num
        self.neighbor_num = neighbor_num
        self.alpha = 0.1
        
        # generate world distribution for each topic
        self.width = self.community_num / 2
        self.vocab_size = self.width ** 2
        self.word_distribution = np.zeros((self.community_num, self.vocab_size))
 
        for k in range(self.width):
            self.word_distribution[k,:] = self.vertical_topic(self.width, k, self.neighbor_num)
 
        for k in range(self.width):
            self.word_distribution[k+self.width,:] = self.horizontal_topic(self.width, k, self.neighbor_num)
            
        # turn counts into probabilities
        self.word_distribution /= self.word_distribution.sum(axis=1)[:, np.newaxis] 
        
        
    def generateSocialInteractionProfile(self):
        """
        Generate a social interaction profile:
        """
        # 1) Sample topic proportions from the Dirichlet distribution.
        theta = np.random.mtrand.dirichlet([self.alpha] * self.community_num)
        v = np.zeros(self.vocab_size)
        vc = np.zeros(self.neighbor_num)
        ve = np.zeros(self.neighbor_num)
        for n in range(self.neighbor_num):
            # 2) Sample a topic index from the Multinomial with the topic proportions from 1).
            z = self.sample_index(theta)
            # 3) Sample a word from the Multinomial corresponding to the topic index from 2).
            w = self.sample_index(self.word_distribution[z,:])
            v[w] += 1
            ve[n] = w
            vc[n] = z
        return v, ve, vc
    
    def generateSocialNetwork(self, node_num):
        """
        Generate a social agent - social interaction profile matrix.
        """
        m = np.zeros((node_num, self.vocab_size))
        me = np.zeros((node_num, self.neighbor_num))
        mc = np.zeros((node_num, self.neighbor_num))
        for i in xrange(node_num):
            m[i, :], me[i,:], mc[i, :] = self.generateSocialInteractionProfile()
        return m, me, mc
    
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

        
'''
Created on 2014-11-10

@author: Walter
'''

import numpy as np
from scipy.special import gammaln
import matplotlib.pyplot as plt

class LDASampler(object):

    def __init__(self, topic_num, alpha=0.1, beta=0.1):
        """
        topic_num: desired number of topics
        alpha: a scalar (FIXME: accept vector of size topic_num)
        beta: a scalar (FIME: accept vector of size vocab_size)
        """
        self.n_topics = topic_num
        self.alpha = alpha
        self.beta = beta
 
    def _initialize(self, matrix):
        # number of times document m and topic z co-occur
        self.nmz = np.zeros((self.n_docs, self.n_topics))
        # number of times topic z and word w co-occur
        self.nzw = np.zeros((self.n_topics, self.vocab_size))
        self.nm = np.zeros(self.n_docs)
        self.nz = np.zeros(self.n_topics)
        self.topics = {}
 
        for m in xrange(self.n_docs):
            # i is a number between 0 and doc_length-1
            # w is a number between 0 and vocab_size-1
            for i, w in enumerate(self.word_indices(matrix[m, :])):
                # choose an arbitrary topic as first topic for word i
                z = np.random.randint(self.n_topics)
                self.nmz[m,z] += 1
                self.nm[m] += 1
                self.nzw[z,w] += 1
                self.nz[z] += 1
                self.topics[(m,i)] = z
 
    def run(self, matrix, max_iter):
        # Run the Gibbs sampler.
        self.n_docs = matrix.shape[0]
        self.vocab_size = matrix.shape[1]
        self._initialize(matrix)
 
        for it in xrange(max_iter):
            for m in xrange(self.n_docs):
                for i, w in enumerate(self.word_indices(matrix[m, :])):
                    z = self.topics[(m,i)]
                    self.nmz[m,z] -= 1
                    self.nm[m] -= 1
                    self.nzw[z,w] -= 1
                    self.nz[z] -= 1
 
                    p_z = self._conditional_distribution(m, w)
                    z = self.sample_index(p_z)
 
                    self.nmz[m,z] += 1
                    self.nm[m] += 1
                    self.nzw[z,w] += 1
                    self.nz[z] += 1
                    self.topics[(m,i)] = z
 
            # FIXME: burn-in and lag!
            yield self.phi()
            
    def _conditional_distribution(self, m, w):
        """
        Conditional distribution (vector of size n_topics).
        """
        vocab_size = self.nzw.shape[1]
        left = (self.nzw[:,w] + self.beta) / (self.nz + self.beta * vocab_size)
        right = (self.nmz[m,:] + self.alpha) / (self.nm[m] + self.alpha * self.n_topics)
        p_z = left * right
        # normalize to obtain probabilities
        p_z /= np.sum(p_z)
        return p_z
 
    def loglikelihood(self):
        """
        Compute the likelihood that the model generated the data.
        """
        vocab_size = self.nzw.shape[1]
        n_docs = self.nmz.shape[0]
        lik = 0
 
        for z in xrange(self.n_topics):
            lik += self.log_multi_beta(self.nzw[z,:]+self.beta)
            lik -= self.log_multi_beta(self.beta, vocab_size)
 
        for m in xrange(n_docs):
            lik += self.log_multi_beta(self.nmz[m,:]+self.alpha)
            lik -= self.log_multi_beta(self.alpha, self.n_topics)
 
        return lik

    def phi(self):
        """
        Compute phi = p(w|z).
        """
        V = self.nzw.shape[1]
        num = self.nzw + self.beta
        num /= np.sum(num, axis=1)[:, np.newaxis]
        return num      
    
    def theta(self): 
        '''
        Compute theta = p(z|m)
        '''
        V = self.nmz.shape[1]
        num = self.nmz + self.alpha
        num /= np.sum(num,axis=1)[:, np.newaxis]
        return num
             
    def word_indices(self, vec):
        """
        Turn a document vector of size vocab_size to a sequence
        of word indices. The word indices are between 0 and
        vocab_size-1. The sequence length is equal to the document length.
        """
        for idx in vec.nonzero()[0]:
            for i in xrange(int(vec[idx])):
                yield idx
 
    def log_multi_beta(self, alpha, K=None):
        # Logarithm of the multinomial beta function.
        if K is None:
            # alpha is assumed to be a vector
            return np.sum(gammaln(alpha)) - gammaln(np.sum(alpha))
        else:
            # alpha is assumed to be a scalar
            return K * gammaln(alpha) - gammaln(K*alpha)
        
    def sample_index(self, p):
        # Sample from the Multinomial distribution and return the sample index.
        return np.random.multinomial(1,p).argmax()
    
    def save_document_image(self, filename, doc, zoom=2):
        # Save document as an image. Doc must be a square matrix
        height, width = doc.shape
        zoom = np.ones((width*zoom, width*zoom))
        # imsave scales pixels between 0 and 255 automatically
        plt.imsave(filename, np.kron(doc,zoom))
 
        
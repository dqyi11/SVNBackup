'''
Created on Nov 21, 2014

@author: daqing_yi
'''

import numpy as np
from scipy.special import gammaln
import matplotlib.pyplot as plt

class LDASampler(object):

    def __init__(self, topic_num, alpha=0.1, beta=0.1):
        """
        community_num: desired number of community
        alpha: a scalar (FIXME: accept vector of size community_num)
        beta: a scalar (FIME: accept vector of size node_size)
        """
        self.n_communities = topic_num
        self.alpha = alpha
        self.beta = beta
 
    def _initialize(self, matrix):
        # number of times document m and topic z co-occur
        self.nmz = np.zeros((self.n_actors, self.n_communities))
        # number of times topic z and word w co-occur
        self.nzw = np.zeros((self.n_communities, self.node_size))
        self.nm = np.zeros(self.n_actors)
        self.nz = np.zeros(self.n_communities)
        self.communities = {}
 
        for m in xrange(self.n_actors):
            # i is a number between 0 and doc_length-1
            # w is a number between 0 and node_size-1
            for i, w in enumerate(self.word_indices(matrix[m, :])):
                # choose an arbitrary topic as first topic for word i
                z = np.random.randint(self.n_communities)
                self.nmz[m,z] += 1
                self.nm[m] += 1
                self.nzw[z,w] += 1
                self.nz[z] += 1
                self.communities[(m,i)] = z
 
    def run(self, matrix, max_iter):
        # Run the Gibbs sampler.
        self.n_actors = matrix.shape[0]
        self.node_size = matrix.shape[1]
        self._initialize(matrix)
 
        for it in xrange(max_iter):
            for m in xrange(self.n_actors):
                for i, w in enumerate(self.word_indices(matrix[m, :])):
                    z = self.communities[(m,i)]
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
                    self.communities[(m,i)] = z
 
            # FIXME: burn-in and lag!
            yield self.phi()
            
    def _conditional_distribution(self, m, w):
        """
        Conditional distribution (vector of size n_communities).
        """
        vocab_size = self.nzw.shape[1]
        left = (self.nzw[:,w] + self.beta) / (self.nz + self.beta * vocab_size)
        right = (self.nmz[m,:] + self.alpha) / (self.nm[m] + self.alpha * self.n_communities)
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
 
        for z in xrange(self.n_communities):
            lik += self.log_multi_beta(self.nzw[z,:]+self.beta)
            lik -= self.log_multi_beta(self.beta, vocab_size)
 
        for m in xrange(n_docs):
            lik += self.log_multi_beta(self.nmz[m,:]+self.alpha)
            lik -= self.log_multi_beta(self.alpha, self.n_communities)
 
        return lik

    def phi(self):
        """
        Compute phi = p(w|z).
        """
        V = self.nzw.shape[1]
        num = self.nzw + self.beta
        num /= np.sum(num, axis=1)[:, np.newaxis]
        return num           
            
    def word_indices(self, vec):
        """
        Turn a document vector of size node_size to a sequence
        of word indices. The word indices are between 0 and
        node_size-1. The sequence length is equal to the document length.
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

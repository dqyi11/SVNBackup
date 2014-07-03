from MultiPartiteGraph import *

class ExpandingMultiPartiteGraph(MultiPartiteGraph):

    def __init__(self, T, name, dim):
        super(ExpandingMultiPartiteGraph, self).__init__(T, name)
        self.dim = dim
    
        
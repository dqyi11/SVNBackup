'''
Created on Jun 12, 2013

@author: joseph
'''


from Node import NodeType

Direction = {"Parent":0, "Child":1, "Start":2}

class Path:
    
    def __init__(self, node, direction):
        """ Creates a path of size 1."""
        self.nodePairs = []
        if not (node == None and direction == None):
            self.nodePairs.append((node, direction))
            
    def __repr__(self):
        l = [pair[0] for pair in self.nodePairs]
        return str(l)
        
    def clone(self):
        result = Path(None, None)
        for pair in self.nodePairs:
            result.nodePairs.append(pair)
        return result
    
    def append(self, node, direction):
        """ Add a node to this path. The direction is this node's relationship with the last node in the path.
        So if we have the path (A)->(B)<-(C), and we want to add a node named D to make (A)->(B)<-(C)->(D), then 
        we would call append(D, Direction["CHILD"]).
        """
        if len(self.nodePairs) == 0:
            assert direction == Direction["Start"]
            assert node.type == NodeType["Query"]
        self.nodePairs.append((node, direction))
       
    def __getitem__(self, index):
        return self.nodePairs[index]
    
    def __len__(self):
        return len(self.nodePairs)
    
    def __contains__(self, node):
        """ Returns True if the given node is in this path, False otherwise. This needs to be efficient
        because we will call it many times."""
        for pair in self.nodePairs:
            if pair[0] == node:
                return True
        return False
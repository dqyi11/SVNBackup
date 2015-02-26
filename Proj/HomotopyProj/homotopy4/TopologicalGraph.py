'''
Created on Nov 17, 2014

@author: daqing_yi
'''

#from graphviz import Graph
import networkx as nx
import matplotlib.pyplot as plt

class TopologicalNode(object):
    
    def __init__(self, name):
        self.name = name
        
class TopologicalEdge(object):
    
    def __init__(self, node_a, node_b, name=""):
        self.node_a = node_a
        self.node_b = node_b
        self.name = name

class TopologicalGraph(object):

    def __init__(self):
        self.nodes = []
        self.edges = []
        self.G = nx.Graph()
        self.edge_labels = {}
        self.edge_label_strs = {}
        
    def addNode(self, name):
        n = TopologicalNode(name)
        self.nodes.append(n)
        self.G.add_node(name, text=name)
        return n
        
    def addEdge(self, node_a, node_b, name):
        e = TopologicalEdge(node_a, node_b, name)
        self.edges.append(e)
        self.G.add_edge(node_a.name, node_b.name, text=name)
        self.edge_labels[node_a.name, node_b.name] = name
        self.edge_label_strs[node_a.name+"+"+node_b.name] = name
        self.edge_label_strs[node_b.name+"+"+node_a.name] = name
        return e
    
    def findNode(self, name):
        for n in self.nodes:
            if n.name == name:
                return n
        return None
    
    def getEdgeLabel(self, node_a_name, node_b_name):
        label_name = node_a_name+"+"+node_b_name
        label = self.edge_label_strs[label_name]
        return label
        
    def visualize(self, filename):
        '''
        g = Graph(format='png', engine='neato')
        for n in self.nodes:
            g.node(n.name)
        
        for e in self.edges:
            if e!= None:
                g.edge(e.node_a.name, e.node_b.name, label=e.name)
        
        g.render(filename+'.png', view=True)
        '''
        #graph_pos = nx.graphviz_layout(self.G)
        #graph_pos = nx.spring_layout(self.G, scale=4)
        graph_pos = nx.spring_layout(self.G)
        nx.draw_networkx_edge_labels(self.G, pos=graph_pos, edge_labels=self.edge_labels)
        nx.draw_networkx_nodes(self.G, pos=graph_pos, node_size=1500, node_color=(153./255,178./255,255./255,1.0), wdith=0.0)
        nx.draw_networkx_edges(self.G, pos=graph_pos, width=3, edge_color='orange')
        nx.draw_networkx_labels(self.G, pos=graph_pos)
        plt.show()
        
        
    def findAllPathsByBFS(self, start, end):
        paths = nx.all_simple_paths(self.G, start, end)
        ps = []
        for path in paths:
            p = []
            for i in range(len(path)-1):
                p.append(self.getEdgeLabel(path[i], path[i+1]))
            ps.append(p)
            
        return ps    
                
        '''
        all_paths=[]
        current_path=[]
        def append_path(p):
            all_paths.append( current_path )
      
        for e in nx.dfs_edges(nx.bfs_tree(self.G, start)):
            if e[0]== start: #We start a new path
                if len(current_path):
                    # Do we end in an out node ? 
                    append_path(current_path)
                    current_path=[]
            if e[1] == end: # We found a path
                append_path(current_path) 
      
            current_path.append(e)          
        if len(current_path):
            append_path(current_path)
        return all_paths
        '''
        
        
        
        
        
        
        

        
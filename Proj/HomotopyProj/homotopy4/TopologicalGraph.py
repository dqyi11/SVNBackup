'''
Created on Nov 17, 2014

@author: daqing_yi
'''

#from graphviz import Graph
import networkx as nx
import matplotlib.pyplot as plt
import copy


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
        
        self.nodes_property = {}
        self.positive_list = []
        self.negative_list = []
        
        
    def addNode(self, name):
        n = TopologicalNode(name)
        self.nodes.append(n)
        self.G.add_node(name, text=name)
        return n
        
    def addEdge(self, node_a, node_b, name):
        e = TopologicalEdge(node_a, node_b, name)
        self.edges.append(e)
        self.G.add_edge(node_a.name, node_b.name, key=name)
        #self.G.add_edge(node_b.name, node_a.name, text=name)
        if not( (node_a.name, node_b.name) in self.edge_labels.keys() ):
            self.edge_labels[node_a.name, node_b.name] = []
            
        self.edge_labels[node_a.name, node_b.name].append(name)
        #self.edge_labels[node_b.name, node_a.name] = name
        #self.edge_label_strs[node_a.name+"+"+node_b.name] = name
        #self.edge_label_strs[node_b.name+"+"+node_a.name] = name
        return e
    
    def findNode(self, name):
        for n in self.nodes:
            if n.name == name:
                return n
        return None
    
        
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
        
        for e in self.G.edges():
            print e
        
        graph_pos = nx.spring_layout(self.G)
        nx.draw_networkx_edge_labels(self.G, pos=graph_pos, edge_labels=self.edge_labels)
        nx.draw_networkx_nodes(self.G, pos=graph_pos, node_size=1500, node_color=(153./255,178./255,255./255,1.0), wdith=0.0)
        nx.draw_networkx_edges(self.G, pos=graph_pos, width=3, edge_color='orange')
        nx.draw_networkx_labels(self.G, pos=graph_pos)
        plt.show()
        
        
    def isEligiblePath(self, path):
        for pl in self.positive_list:
            if not (pl in path):
                return False
        for p in path:
            if self.nodes_property[p] == -1:
                return False
        return True
        
    def findAllPathsByBFS(self, start, end, filter=False):
        paths = nx.all_simple_paths(self.G, start, end)
        
        print paths
        ps = []
        for path in paths:
           
            print path
            
            if filter==True:
                if self.isEligiblePath(path)==True:
                    strings = self.getStringsFromPath(path)
                    for s in strings:
                        ps.append(s)
            else:
                strings = self.getStringsFromPath(path)
                for s in strings:
                    ps.append(s)           
        return ps  
    
    def getEdgeLabels(self, name_a, name_b):
        
        if (name_a, name_b) in self.edge_labels.keys():
            return self.edge_labels[name_a, name_b]
        elif (name_b, name_a) in self.edge_labels.keys():
            return self.edge_labels[name_b, name_a]
        
        return []
        
    
    def getStringsFromPath(self, path):
        strings = [[]]
        
        for i in range(len(path)-1):
            name_a = path[i]
            name_b = path[i+1]
            edge_labels = self.getEdgeLabels(name_a, name_b)
            edge_num = len(edge_labels)
            if edge_num == 1:
                for string in strings:
                    string.append(edge_labels[0])
            elif edge_num > 1:
                str_len = len(strings)
                for j in range(edge_num-1):
                    edge_label = edge_labels[j+1]
                    for i in range(str_len):
                        new_string = copy.deepcopy(strings[i])
                        new_string.append(edge_label)
                        strings.append(new_string)
                        
                for i in range(str_len):
                    strings[i].append(edge_labels[0])
            
        return strings  
                

        
        
        
        
        
        
        

        
'''
Created on Aug 6, 2015

@author: daqing_yi
'''

class Path(object):


    def __init__(self):
        self.waypoints = []
        
        
    def loadFromFile(self, filename):
        
        out_file = open(filename, "w")
        str_lines = out_file.readlines()
        
        in_path_mode = False
        for str_line in str_lines:
            if str_line == "\n":
                in_path_mode = True
            else:
                str_poses = str_line.split("\t")
                for str_pos in str_poses:
                    str_ps = str_pos.split(" ")
                    pos = []
                    for str_p in str_ps:
                        pos.append(int(str_p))
                    self.waypoints.append(pos)
                        

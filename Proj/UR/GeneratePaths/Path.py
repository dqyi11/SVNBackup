'''
Created on Aug 6, 2015

@author: daqing_yi
'''

class Path(object):


    def __init__(self):
        self.waypoints = []
        
        
    def loadFromFile(self, filename):
        try:
            out_file = open(filename, "r")
            str_lines = out_file.readlines()
            
            in_path_mode = False
            for str_line in str_lines:
                if str_line == "\n":
                    in_path_mode = True
                else:
                    if in_path_mode == True:
                        str_line = str_line.replace('\n', '')
                        str_poses = str_line.split("\t")
                        for str_pos in str_poses:
                            if str_pos != "":
                                str_ps = str_pos.split(" ")
                                pos = []
                                for str_p in str_ps:
                                    pos.append(int(str_p))
                                self.waypoints.append(pos)
            return True
        except:
            return False
                        

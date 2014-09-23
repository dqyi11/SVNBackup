'''
Created on Sep 23, 2014

@author: daqing_yi
'''
import cv2
import numpy as np

def meanShiftFilter(img, spatial_radius, range_radius, iterate_num=100):
    
    img_width = img.shape[0]
    img_height = img.shape[1]
    
    img_luv = cv2.cvtColor(img, cv2.cv.CV_BGR2Luv)
    
    img_luv_ret = np.zeros(img_luv.shape, np.int)
    
    
    print img_luv.shape
    
    for i in range(img_width):
        for j in range(img_height):
            
            print str(i) + ' - ' + str(j)
            
            shift = 5                
            i_rec, j_rec = i, j
            L_rec, U_rec, V_rec = img_luv[i, j, 0], img_luv[i, j, 1], img_luv[i, j, 2]
            
            for itr in range(iterate_num):                
                if shift <= 3:
                    break
                
                f_i_from, f_i_to = np.max([0, i-spatial_radius]), np.min([img_width, i+spatial_radius+1])
                f_j_from, f_j_to = np.max([0, j-spatial_radius]), np.min([img_height, j+spatial_radius+1])
                
                mi, mj = 0, 0
                mL, mU, mV = 0, 0, 0
                cnt = 0

                for f_i in range(f_i_from, f_i_to):
                    for f_j in range(f_j_from, f_j_to):
                        
                        dL = int(img_luv[f_i, f_j, 0]) - int(img_luv[i, j, 0])
                        dU = int(img_luv[f_i, f_j, 1]) - int(img_luv[i, j, 1])
                        dV = int(img_luv[f_i, f_j, 2]) - int(img_luv[i, j, 2])
                        
                        if np.linalg.norm([dL, dU, dV]) <= range_radius:
                            mi += f_i
                            mj += f_j
                            mL += img_luv[f_i, f_j, 0]
                            mU += img_luv[f_i, f_j, 1]
                            mV += img_luv[f_i, f_j, 2]
                            cnt += 1
                            
                nL = int( mL / float(cnt) )
                nU = int( mU / float(cnt) )
                nV = int( mV / float(cnt) )
                
                ni = int( mi / float(cnt) + 0.5 )
                nj = int( mj / float(cnt) + 0.5 )
                
                shift = np.linalg.norm([ni-i_rec, nj-j_rec, nL-img_luv[i, j, 0], nU-img_luv[i, j, 1], nV-img_luv[i, j, 2]])                
                i_rec, j_rec = ni, nj
                L_rec, U_rec, V_rec = nL, nU, nV
                
            img_luv_ret[i,j,0], img_luv_ret[i,j,1], img_luv_ret[i,j,2] = L_rec, U_rec, V_rec
            
    return cv2.cvtColor(img_luv_ret, cv2.cv.CV_Luv2BGR)


def transitiveClosure(img):
    pass            
            
def prune(img):
    pass                
         
            
                            
                        
                        
                        
                
                
            
            
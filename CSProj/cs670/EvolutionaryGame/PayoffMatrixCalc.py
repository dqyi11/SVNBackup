'''
Created on 2013-10-31

@author: Walter
'''
import numpy as np;

 
def ConvertPayoff(T, R, P, S, gamma):

    pm = np.zeros((4,4), np.double);
    '''
    col : AC, AD, TfT, NotTfT
    row : AC, AD, TfT, NotTfT
    '''
    #AC - AC
    pm[0,0] = R/(1-gamma);
    # AC - AD
    pm[0,1] = S/(1-gamma);
    # AC - TfT
    pm[0,2] = R/(1-gamma);
    # AC - NotTfT
    pm[0,3] = S/(1-gamma);
    
    #AD - AC
    pm[1,0] = T/(1-gamma);
    #AD - AD 
    pm[1,1] = P/(1-gamma);
    #AD - TfT
    pm[1,2] = T + gamma*P/(1-gamma);
    #AD - NotTfT
    pm[1,3] = R + gamma*T/(1-gamma);    

    #TfT - AC
    pm[2,0] = R/(1-gamma);
    #TfT - AD
    pm[2,1] = S + gamma*P/(1-gamma);
    #TfT - TfT
    pm[2,2] = R/(1-gamma);
    #TfT - NotTfT
    pm[2,3] = (S + gamma * P + gamma**2 * T + gamma**3 + R) / ( 1- gamma**4);
    
    #NotTfT - AC
    pm[3,0] = T/(1-gamma);
    #NotTfT - AD
    pm[3,1] = P + gamma*S/(1-gamma);
    #NotTfT - TfT
    pm[3,2] = (T + gamma * P + gamma**2 * S + gamma**3 * R) / ( 1- gamma**4)
    #NotTfT - NotTfT
    pm[3,3] = (P + gamma * R) / (1 - gamma**2);
    
    return pm;

if __name__ == "__main__":
      
    # T, R, P, S, gamma
    # prisoner's dilemma
    payoffMatrix1 = ConvertPayoff(4,3,2,1,0.95);
    print payoffMatrix1;
#    payoffMatrix2 = ConvertPayoff(4,3,2,1,0.99);
#    print payoffMatrix2;
#    # stag hunt
#    payoffMatrix3 = ConvertPayoff(3,4,2,1,0.95);
#    print payoffMatrix3;
#    payoffMatrix4 = ConvertPayoff(3,4,2,1,0.99);
#    print payoffMatrix4;
#    # battle of the sexes
#    payoffMatrix5 = ConvertPayoff(3,2,1,4,0.95);
#    print payoffMatrix5;
#    payoffMatrix6 = ConvertPayoff(3,2,1,4,0.99);
#    print payoffMatrix6;
    
    
    
     
    
    
    
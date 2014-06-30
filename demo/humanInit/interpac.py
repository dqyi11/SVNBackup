import pymatlab
import numpy as np

def interpac(t, px, py):
    
    newpx = []
    newpy = []
    
    session = pymatlab.session_factory()
    session.putvalue('t', t)
    session.putvalue('x', np.darray(px))
    session.putvalue('y', np.darray(py))
    
    session.run('out=interpac(t,x,y)')
    outval = session.getvalue('out') 
    
    newpx = outval[:,0]
    newpy = outval[:,1]   
    
    return newpx, newpy
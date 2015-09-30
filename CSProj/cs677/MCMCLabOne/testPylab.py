'''
Created on May 28, 2013

@author: walter
'''

if __name__ == '__main__':
    import matplotlib
    matplotlib.use('WXAgg')
    from pylab import *
    
    t = arange(0.0, 2.0, 0.01)
    s = sin(2*pi*t)
    plot(t,s,linewidth=1.0)
    show()
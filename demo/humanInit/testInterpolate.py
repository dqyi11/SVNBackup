import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

xs = np.arange(0, 5, 0.5)
ys = np.random.random(len(xs))

newxs = np.arange(0, 4, 0.1)

f = interp1d(xs,ys,kind='cubic')
#f = interp1d(xs,ys, kind='cubic')

'''
print newxs
print f(newxs)
'''


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(xs, ys, 'o', newxs, f(newxs), '-')
plt.show()

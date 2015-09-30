import matplotlib.pyplot as plt
import numpy as np
import math

fig = plt.figure()
ax = fig.add_subplot(111)

x = np.arange(0,5,0.1)
y = [math.sin(a) for a in x]
ax.plot(x,y,"ro-")
plt.show()

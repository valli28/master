import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

yy, zz = np.meshgrid(range(10), range(10))
xx = yy


print(xx)
print(yy)
print(zz)


ax = plt.subplot(projection='3d')
ax.plot_surface(xx, yy, zz)
plt.show()
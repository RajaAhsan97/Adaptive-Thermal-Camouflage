"""
    This code is made by RMA for the purpose to visualize thermal contour pattern
    on camou-pixel saved from COMSOL Multiphysics in a text file
"""

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import matplotlib.colors
import numpy as np
import math
from scipy.interpolate import griddata
from mpl_toolkits.mplot3d import Axes3D


# Defining coordinates for Hexagon pixel 
xhex = [-88, -88, 0, 88, 88, 0, -88]
yhex = [-50.8, 50.8, 101.6, 50.8, -50.8, -101.6, -50.8]

##f = open('hex_grids_data(20 sec).txt',"r")
##d = f.readline()
##print(d)


# 'np' is used to read the data in the text file into array
data = np.loadtxt('hex_grids_data(20 sec).txt')
x_cord = data[:,0]         #  extract x cordinates
x_size = np.size(x_cord)
y_cord = data[:,1]         #  extract y cordinates 
y_size = np.size(y_cord)
temp_data = data[:,2]      #  extract temperature data
temp_size = np.size(temp_data)

##print(x_cord)
##print(x_size)
##print(y_cord)
##print(y_size)
##print(temp_data)
##print(temp_size)

ii = 0
xg = []
xg = [0.0 for i in range(math.ceil((len(x_cord)/2)))]
q = math.ceil((len(x_cord)/2))
##print(q)
yg = []
yg = [0.0 for i in range(math.ceil((len(x_cord)/2)))]
zg = []
zg = [0.0 for i in range(math.ceil((len(x_cord)/2)))]
for i in range(0,len(x_cord),2):
    xg[ii] = x_cord[i]
    yg[ii] = y_cord[i]
    zg[ii] = temp_data[i]
    ii+=1
    
xg_size = np.size(xg)
yg_size = np.size(yg)
zg_rows = len(zg)

x = np.linspace(-88,88,1761)
y = np.linspace(-101,101,2021)
X,Y = np.meshgrid(x,y)
##print("X=")
##print(X)
X_rows = len(X)
X_cols = len(X[0])
##print(X_rows,X_cols)
##print("Y=")
##print(Y)
Y_rows = len(Y)
Y_cols = len(Y[0])
##print(Y_rows,Y_cols)

temperature = griddata((xg,yg),zg,(X,Y),method='cubic')
##print("temperature")
##print(temperature)
temp_rows = len(temperature)
temp_cols = len(temperature[0])
##print(temp_rows,temp_cols)

"""
    Plotting figures (2D heat map)
"""

fig = plt.figure()
plt.plot(xhex,yhex)

s = plt.pcolormesh(X,Y, temperature , cmap = 'jet' )
fig.colorbar(s)
plt.title( '2-D Heat Map' )
plt.axis('image')
#plt.grid()
plt.show()

"""
    plotting end
"""

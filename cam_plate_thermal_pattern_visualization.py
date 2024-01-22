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

##x = temp_data[26719]
##print(x)

ii = 0
xg = []#[None] * math.ceil((len(x_cord)/2))
xg = [0.0 for i in range(math.ceil((len(x_cord)/2)))]
q = math.ceil((len(x_cord)/2))
##print(q)
yg = []#[None] * math.ceil((len(x_cord)/2))
yg = [0.0 for i in range(math.ceil((len(x_cord)/2)))]
zg = []#[None] * math.ceil((len(x_cord)/2))
zg = [0.0 for i in range(math.ceil((len(x_cord)/2)))]
for i in range(0,len(x_cord),2):
    xg[ii] = x_cord[i]
    yg[ii] = y_cord[i]
    zg[ii] = temp_data[i]
    ii+=1
    
xg_size = np.size(xg)
yg_size = np.size(yg)
zg_rows = len(zg)
#zg_cols = len(zg[0])
##print("size")
##print(xg_size)
##print(yg_size)
##print(zg_rows)
##
##print(zg)
##
##print(zg[13358])

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
    Plotting figures (2D heat map and 3D surface plot)
"""

# Plotting 3D surface Plot
##fig = plt.figure() 
##ax = plt.axes(projection ='3d')
##my_cmap = plt.get_cmap('jet')
##surf = ax.plot_surface(X, Y, temperature,cmap=my_cmap,edgecolor='none')  #plot_surface 
##ax.view_init(90, 0)
##fig.colorbar(surf, ax=ax)
##ax.grid(False)
##plt.show()

# Plot 2D heat map of camou-plate
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

#cnt = 0
##print(X[:,0])

##x1 = [0.0 for i in range(X_cols)]
##y1 = [0.0 for i in range(X_cols)]
##t1 = [0.0 for i in range(X_cols)]
##XX = [0.0 for i in range(X_cols*X_rows)]
##YY = [0.0 for i in range(X_cols*X_rows)]
##tt = [0.0 for i in range(X_cols*X_rows)]
##for i in range(0,X_cols):
##    x1 = X[:,i]
##    y1 = Y[:,i]
##    t1 = temperature[:,i]
##    for ii in range(0,X_rows):
##        XX[cnt] = x1[ii]
##        YY[cnt] = y1[ii]
##        tt[cnt] = t1[ii]
##        cnt += 1
##    x1 = []
##    y1 = []
##    t1 = []
##
##x_t = np.transpose(XX)
##y_t = np.transpose(YY)
##t_t = np.transpose(tt)
##print(len(x_t))
##print(len(y_t))
##print(len(t_t))
####plt.scatter(x_t,y_t,t_t,cmap='jet')
##plt.show()



##fig = plt.figure()
##cs = plt.contourf(X, Y, temperature,cmap='jet')
##norm= matplotlib.colors.Normalize(vmin=cs.cvalues.min(), vmax=cs.cvalues.max())
##sm = plt.cm.ScalarMappable(norm=norm, cmap = cs.cmap)
##sm.set_array([])
##fig.colorbar(sm, ticks=cs.levels)
###plt.colorbar(cp) # Add a colorbar to a plot
##
####ax.set_title('Filled Contours Plot')
#####ax.set_xlabel('x (cm)')
####ax.set_ylabel('y (cm)')
##plt.axis('image')
##plt.show()

##x1_rows = len(x1)
##x1_cols = len(x1[0])
##print("x1")
##print(x1_rows,x1_cols)





##with open('hex_grids_data(20 sec).txt') as inf:
##    reader = csv.reader(inf, delimiter=" ")
##    second_col = list(zip(*reader))[1]
##    print(inf)



##
##plt.show()




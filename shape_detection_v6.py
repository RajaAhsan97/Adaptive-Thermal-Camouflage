"""
     code by RMA
     
     This code uses Canny Edge detection to find the edges, and
     detection of corner points of the hexagon by using Harris
     corner detction algorithm furthermore for precise detection
     of corner points OpenCv function {cornersubpix} is used.

     The center point for the hexagon is successfully detected but
     for some grabbed frames .csv file, some point between the
     corner points is also detected by which the center point of hexagon
     is not determined correctly.
     To correct this problem , my idea is to sort the detected corner
     points in clockwise/anticlockwise order and compute the distance
     between the neigbouring corner points, if distance between them
     is less than the maximum distance then neglect the center point
     between the corner points
"""

from numpy import genfromtxt
import cv2
import matplotlib.pyplot as plt
import numpy as np
#import scipy.ndimage
from scipy import ndimage
import math
import operator
from functools import reduce


def matlab_style_gauss2D(shape=(40,40),sigma=1):
     """
     2D gaussian mask - should give the same result as MATLAB's
     fspecial('gaussian',[shape],[sigma])
     """
     m,n = [(ss-1.)/2. for ss in shape]
     y,x = np.ogrid[-m:m+1,-n:n+1]
     h = np.exp( -(x*x + y*y) / (2.*sigma*sigma) ) #* (2.0*np.pi*sigma**2) )
     h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
     sumh = h.sum()
     if sumh != 0:
         h /= sumh
     return h


def clockwiseangle_and_distance(point):
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.   
    return angle, lenvector

## ___________________________________________________________________________________________________________________
## Sample .csv file (for testing) of the saved thermal frame from GUI (i.e camou-pixel_I GUI.py) used to acquire the 
## contour of camou-pixel I and determining the centrod
# TEMP-2022-08-04T12-25-01-22.csv
# TEMP-2022-08-12T16-59-08-14.csv
# TEMP-2022-08-12T16-59-14-11.csv
# TEMP-2022-08-12T16-45-22-03.csv
# TEMP-2022-08-05T17-11-33-86.csv

## TEMP-2022-08-17T16-43-23-75.csv
## TEMP-2022-08-22T11-24-49-79.csv
## TEMP-2022-08-22T12-14-39-93.csv
## TEMP-2022-08-22T12-14-41-77.csv
## TEMP-2022-08-22T12-14-43-51.csv
## TEMP-2022-08-22T13-02-07-54.csv

## TEMP-2022-08-22T13-02-08-80.csv
## TEMP-2022-08-22T13-02-11-99.csv
## TEMP-2022-08-22T13-18-04-42.csv
## TEMP-2022-08-30T18-42-32-78.csv

## TEMP-2022-08-22T14-24-25-02.csv
## TEMP-2022-08-22T14-24-27-12.csv
## TEMP-2022-08-22T14-24-28-24.csv
## TEMP-2022-08-22T14-24-30-27.csv
## TEMP-2022-08-22T14-24-31-94.csv
## TEMP-2022-08-22T14-24-36-98.csv
## TEMP-2022-08-22T14-46-12-53.csv
## TEMP-2022-09-01T14-56-05-18.csv
## TEMP-2022-09-01T14-56-01-24.csv
## TEMP-2022-09-01T14-55-52-98.csv
## TEMP-2022-08-30T17-56-52-85.csv
## TEMP-2022-08-30T17-56-46-28.csv
## TEMP-2022-08-30T18-33-06-20.csv
## TEMP-2022-09-08T16-30-24-86.csv
##TEMP-2022-09-08T16-30-24-86.csv
## ___________________________________________________________________________________________________________________

img = genfromtxt('TEMP-2022-09-08T16-30-24-86.csv', delimiter = ',')

h = matlab_style_gauss2D()

im = np.zeros((192,256))
#im[60:120,110:190] = img[60:120,110:190]

im = cv2.filter2D(img,-1,h)


IM1 = cv2.rotate(im,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
# show image
plt.title('Thermal Image')
image = ax.imshow(IM1,cmap='jet')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show()
plt.savefig("Thermal image.png", bbox_inches="tight", transparent=True)

print(im)
tp_min = np.min(im)
print(tp_min)
tp_max = np.max(im)
print(tp_max)

# normalization of thermal data to Grey scale values (0-255) - Linear transformation method 
Norm_image = (im - tp_min) * ((255-0) / (tp_max - tp_min)) + 0


# transform float value to integer gray scale values
round_off = np.uint8(np.round_(Norm_image))

print(im)

fig, ax = plt.subplots()
plt.title('Transformed Gray-Scale image')
rf = cv2.rotate(round_off,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(rf,cmap = 'jet')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()
plt.savefig("Identity_linear_Transformed.png", bbox_inches="tight", transparent=True)
print(round_off)

# CONTOUR Detection - EDGE CANNY 
Edgecanny = cv2.Canny(round_off, 200.0, 255.0, L2gradient = False)

fig, ax = plt.subplots()
plt.title('CTGP- Contour detection')
ED = cv2.rotate(Edgecanny,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(ED,cmap = 'jet')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show() 
plt.savefig("contout detected.png", bbox_inches="tight", transparent=True)

################################################3
val_gray = np.where(Edgecanny == 255)
print('gray val')
print(val_gray[0][:])

Edgecanny[val_gray[0][:],val_gray[1][:]] = 1
Edgecanny[:,0:110] = 0


detect_hex = ndimage.binary_fill_holes(Edgecanny).astype(int)
print('DET HEX', detect_hex)
fill_detect_hex = np.where(detect_hex == 1)

##########################################3333

fig, ax = plt.subplots()
plt.title('Transformed Binary image')
DH = cv2.rotate(detect_hex,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(DH,cmap='jet')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show()
plt.savefig("filled detected contour.png", bbox_inches="tight", transparent=True)


# modify the data type
# setting to 32-bit floating point
##print('DATA TYPE1:', np.info(detect_hex))

detect_hex = np.float32(detect_hex)
##print('DATA TYPE2:', np.info(detect_hex))
dest = cv2.cornerHarris(detect_hex, 2, 3, 0.052)  # 2,5,0.07   # 2,3,0.055
print('dest')
print(dest)

fig, ax = plt.subplots()
#plt.title('corner detected')
HC = cv2.rotate(dest,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(HC,cmap = 'tab20b')
plt.xticks([])
plt.yticks([])
#plt.colorbar(image)
fig.show() # show f
plt.savefig("Harris corners.png", bbox_inches="tight", transparent=True)

# Cropping the pixel region of Harris corners for visualization
cropped_HC = HC[85:124,72:112]
fig, ax = plt.subplots()
image = ax.imshow(cropped_HC,cmap = 'tab20b')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show() # show f
plt.savefig("Cropped Harris corners.png", bbox_inches="tight", transparent=True)

# Results are marked through the dilated corners
dest = cv2.dilate(dest, None)

fig, ax = plt.subplots()
#plt.title('corner detected dilated')
HCD = cv2.rotate(dest,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(HCD,cmap = 'tab20b')
plt.xticks([])
plt.yticks([]) 
fig.show() # show f
plt.savefig("Harris corners (dilated).png", bbox_inches="tight", transparent=True)

# Cropping the pixel region of dilated Harris corner for visualization
cropped_HCD = HCD[85:124,72:112]
fig, ax = plt.subplots()
image = ax.imshow(cropped_HCD,cmap = 'tab20b')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show() # show f
plt.savefig("Cropped Harris corners (dilated).png", bbox_inches="tight", transparent=True)

########################################################
 
ret, dst = cv2.threshold(dest,0.001*dest.max(),255,0)
dst = np.uint8(dst)

fig, ax = plt.subplots()
DST = cv2.rotate(dst,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(DST,cmap = 'tab20b')
plt.xticks([])
plt.yticks([]) 
#plt.colorbar(image)
fig.show() # show f
plt.savefig("Harris corners (dilated-thresholded).png", bbox_inches="tight", transparent=True)

# Cropping the pixel region of thresholded dilated Harris corner for visualization
threshold_DST = DST[85:124,72:112]
fig, ax = plt.subplots()
image = ax.imshow(threshold_DST,cmap = 'tab20b')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show() # show f
plt.savefig("Cropped Harris corners (dilate-thresholded).png", bbox_inches="tight", transparent=True)

# find centroids
ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

# define the criteria to stop and refine the corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
corners = cv2.cornerSubPix(detect_hex, np.float32(centroids[1:]), (4,6), (-1,-1), criteria)

# Now draw them
#res = np.hstack((centroids,corners))

##print('CENTROID1:', centroids)
##print(np.info(centroids))
centroids = np.int0(centroids)
##print('CENTROID2:', centroids)
##print(np.info(centroids))

corners = np.int0(corners)
#res = np.int0(res)
#print('res',res)

IM = np.zeros((192,256,3))
IM_CORNERS = np.zeros((192,256,3))
IM[:,:,0] = Edgecanny
IM_CORNERS[:,:,0] = Edgecanny

IM[corners[:,1],corners[:,0]] = [0,0,255]    # refined corners {cv2.cornerSubPix}

IM_CORNERS[corners[:,1],corners[:,0]] = [0,0,255]    # refined corners {cv2.cornerSubPix}

IM[centroids[1:,1],centroids[1:,0]]=[0,255,0]   # Harris corner detction


fig, ax = plt.subplots()
#plt.title('filled detected hex')
CP = cv2.rotate(IM,cv2.ROTATE_90_COUNTERCLOCKWISE)
image = ax.imshow(CP,cmap='gray')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()
plt.savefig("final corners located.png", bbox_inches="tight", transparent=True)

# Cropping the pixel region of thresholded dilated Harris corner for visualization
CROPPED_CP = CP[85:124,72:112]
fig, ax = plt.subplots()
image = ax.imshow(CROPPED_CP,cmap = 'gray')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show() # show f
plt.savefig("Cropped final corners located.png", bbox_inches="tight", transparent=True)

print('centroids',centroids)
print('CENTROIDS:', centroids[1:])
print('corners',corners)

## sorting detected corner coordinates (x,y) in counter-closkwise direction
r,c = len(corners),len(corners[0])
print(r,c)
y = 1
corner_coord = []
while y < r:
     x = 0
     corner_coord.append((corners[y][x], corners[y][x+1]))
     y += 1
corner_coord.append((corners[0][x], corners[0][x+1]))
print('cordinates',corner_coord)

origin = []
origin.append(corners[y-1][x])
origin.append(corners[y-1][x+1])
print('origin',origin)

refvec = [0,1]

##corner_sort = sorted(corner_coord , key = clockwiseangle_and_distance)

center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), corner_coord), [len(corner_coord)] * 2))
ss = sorted(corner_coord, key=lambda coord: (-135 - math.degrees(math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
x_center = int(center[0])
y_center = int(center[1])
y_center = y_center-2
print('central coordinates', x_center, y_center)
print('ss',ss)
print('ss1', corner_coord)
corner_sort = ss


##print('sorted coordinates', corner_sort)
print(corner_sort[1][0], corner_sort[1][1])


## Removing unwanted corner detected coordinates and storing only the six 6
## corner coordinates of heaxagon into array
r,c = len(corner_sort),len(corner_sort[0])
print(r,c)
j = 0
corner_points = []
print("Distance measurement")

print('corner points', corner_sort)

corner_points = corner_sort 
print('corner points', corner_points)

## calculating center points of hexagon from each opposite corner points  
x_left_top, y_left_top = corner_points[0][0], corner_points[0][1]   # 0
x_right_bottom, y_right_bottom = corner_points[4][0], corner_points[4][1]  # 3
x_mid , y_mid = int((x_left_top + x_right_bottom)/2) , int((y_left_top + y_right_bottom)/2) 
print(x_left_top, y_left_top , x_right_bottom, y_right_bottom, x_mid, y_mid )
#IM[y_mid, x_mid] = 1
cv2.line(IM_CORNERS, (x_left_top+1, y_left_top+1), (x_right_bottom-1, y_right_bottom-1), (0, 255, 255), thickness=1)


x_right_top, y_right_top = corner_points[1][0], corner_points[1][1] # 4
x_left_bottom, y_left_bottom = corner_points[5][0], corner_points[5][1] # 1
xx_mid , yy_mid = int((x_right_top + x_left_bottom)/2) , int((y_right_top + y_left_bottom)/2) 
print(x_right_top, y_right_top , x_left_bottom, y_left_bottom, xx_mid , yy_mid)
#IM[yy_mid, xx_mid] = 1
cv2.line(IM_CORNERS, (x_right_top+1, y_right_top-1), (x_left_bottom-1, y_left_bottom+1), (0, 255, 255), thickness=1)

if r == 8:
     dist = abs(corner_points[6][1] - corner_points[7][1])
     if dist <= 2:
          x1 , y1 = corner_points[6][0], corner_points[6][1]
          x2 , y2 = corner_points[7][0], corner_points[7][1]
          x_top, y_top = int((x1 + x2)/2), int((y1 + y2)/2)
     else:
          x_top, y_top = corner_points[7][0], corner_points[7][1] # 5
elif r == 7:
     x_top, y_top = corner_points[6][0], corner_points[6][1]
elif r == 9:
     x1 , y1 = corner_points[7][0], corner_points[7][1]
     x2 , y2 = corner_points[8][0], corner_points[8][1]
     x_top, y_top = int((x1 + x2)/2), int((y1 + y2)/2)
else:
     print('greater corner detected need consideration')
x1 , y1 = corner_points[2][0], corner_points[2][1]
x2 , y2 = corner_points[3][0], corner_points[3][1]
x_bottom, y_bottom = int((x1 + x2)/2), int((y1 + y2)/2)
#x_bottom, y_bottom = corner_points[3][0], corner_points[3][1] #2
##x_bottom1, y_bottom1 = corners[6][0], corners[6][1]
##x_bottom2, y_bottom2 = corners[7][0], corners[7][1]
##x_bottom, y_bottom = int((x_bottom1 + x_bottom2)/2), int((y_bottom1 + y_bottom2)/2) 
xxx_mid , yyy_mid = int((x_top + x_bottom)/2) , int((y_top + y_bottom)/2) 
print(x_top, y_top , x_bottom, y_bottom, xxx_mid , yyy_mid)
#IM[yyy_mid, xxx_mid] = 1
cv2.line(IM_CORNERS, (x_top, y_top), (x_bottom, y_bottom), (0, 255, 255), thickness=1)

# CORNER points are marked with YELLOW color
IM_CORNERS[y_top, x_top,:] = [1,1,0]
IM_CORNERS[y_bottom, x_bottom,:] = [1,1,0]
IM_CORNERS[y_left_top, x_left_top,:] = [1,1,0]
IM_CORNERS[y_right_bottom, x_right_bottom,:] = [1,1,0]
IM_CORNERS[y_right_top, x_right_top,:] = [1,1,0]
IM_CORNERS[y_left_bottom, x_left_bottom,:] = [1,1,0]

## calculating the centeroid of hexagon
print('CENTROID1:', x_mid, ',', y_mid)
print('CENTROID2:', xx_mid, ',', yy_mid)
print('CENTROID3:', xxx_mid, ',', yyy_mid)

x_centroid = int((x_mid + xx_mid + xxx_mid)/3)
y_centroid = int((y_mid + yy_mid + yyy_mid)/3)
IM_CORNERS[y_centroid, x_centroid,:] = [1,1,1]   #IM[y_centroid, x_centroid,:] = [1,1,1]
print('CTGP centroid', y_centroid, x_centroid)


ROTATE_IM_CORNERS = cv2.rotate(IM_CORNERS,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
##plt.title('corner detected marked')
image = ax.imshow(ROTATE_IM_CORNERS,cmap = 'gray')
plt.xticks([])
plt.yticks([]) 
fig.show()
plt.savefig("CTGP centroid.png", bbox_inches="tight", transparent=True)

# Cropping the pixel region of CFGP CENTROID marked for visualization
CROPPED_ROTATE_IM_CORNERS = ROTATE_IM_CORNERS[85:124,72:112]
fig, ax = plt.subplots()
image = ax.imshow(CROPPED_ROTATE_IM_CORNERS,cmap = 'gray')
plt.xticks([])
plt.yticks([]) 
##plt.colorbar(image)
fig.show() # show f
plt.savefig("Cropped CTGP centroid.png", bbox_inches="tight", transparent=True)


# CREATING NEW CTGP SELECTION
coords = []
rotate = 92
sides = 6
theta = math.radians(rotate)
print('Rotation angle:', theta)
n = sides #+ 1
x0 = x_centroid + 1 #x_center   #x_centroid+1
y0 = y_centroid + 1  #y_center   #y_centroid
r = 11
for s in range(sides):
     t = 2.0 * math.pi * s / sides + theta
     coords.append([ int(r * math.cos(t) + x0), int(r * math.sin(t) + y0)])
print(coords)

hex_vertices = np.array(coords)
print(hex_vertices)
hex_vertices = hex_vertices.reshape(-1,1,2)

print(hex_vertices)
cv2.polylines(IM_CORNERS, [hex_vertices], True, (255,0,255), thickness = 1)

ROTATE_IM_CORNERS = cv2.rotate(IM_CORNERS,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
##plt.title('Draw new hexagon')
image = ax.imshow(ROTATE_IM_CORNERS, cmap = 'gray')
plt.xticks([])
plt.yticks([]) 
fig.show() # show f
plt.savefig("NEW CTGP SELECTION.png", bbox_inches="tight", transparent=True)

# Cropping the pixel region of CFGP CENTROID marked for visualization
CROPPED_NEW_CTGP_SELECTIOn = ROTATE_IM_CORNERS[85:124,72:112]
fig, ax = plt.subplots()
image = ax.imshow(CROPPED_NEW_CTGP_SELECTIOn, cmap = 'gray')
plt.xticks([])
plt.yticks([]) 
##plt.colorbar(image)
fig.show() # show f
plt.savefig("Cropped NEW CTGP SELECTION.png", bbox_inches="tight", transparent=True)



r,c = len(IM),len(IM[0])
filled_new_hex = np.zeros((r,c))
cv2.fillPoly(filled_new_hex, pts = [hex_vertices], color = 1)

ROTATE_FILLED_NEW_HEX = cv2.rotate(filled_new_hex,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
##plt.title('filled new hexagon')
image = ax.imshow(ROTATE_FILLED_NEW_HEX,cmap = 'jet')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show() # show f
plt.savefig("NEW FILLEd CTGP SELECTIOn.png", bbox_inches="tight", transparent=True)

filled_hex_coord = np.where(filled_new_hex == 1.0)
print(filled_hex_coord)
r , c = len(filled_hex_coord),len(filled_hex_coord[0])
temps = np.empty(c, dtype=float)

for i in range(0,c):
    temps[i] = img[filled_hex_coord[0][i],filled_hex_coord[1][i]]

tmp_max = max(temps)
tmp_avg = sum(temps)/len(temps)
print('Max temp = ', tmp_max)
print('Avg temp = ', tmp_avg)

#X = ndimage.binary_fill_holes(E).astype(int)

# Distance between each corner point to center of hexagon
dist_top_left_cntr = ((x_left_top - x_centroid)**2 + (y_left_top - y_centroid)**2)**0.5
dist_bottom_right_cntr = ((x_right_bottom - x_centroid)**2 + (y_right_bottom - y_centroid)**2)**0.5
print(dist_top_left_cntr,dist_bottom_right_cntr)

dist_top_right_cntr = ((x_right_top - x_centroid)**2 + (y_right_top - y_centroid)**2)**0.5
dist_bottom_left_cntr = ((x_left_bottom - x_centroid)**2 + (y_left_bottom - y_centroid)**2)**0.5
print(dist_top_right_cntr,dist_bottom_left_cntr)

dist_top_cntr = ((x_top - x_centroid)**2 + (y_top - y_centroid)**2)**0.5
dist_bottom_cntr = ((x_bottom - x_centroid)**2 + (y_bottom - y_centroid)**2)**0.5
print(dist_top_cntr,dist_bottom_cntr)

#--------------------------- DEFINE SURROUNDING AREA ------------------------------#
surr_area = filled_new_hex

# fill surrounding region with value 1
surr_area[60:120,115:187] = 1
# removing the new hexagon coordinate from surrounding coordinates
surr_area[filled_hex_coord[0][:], filled_hex_coord[1][:]] = 0

surr_coord = np.where(surr_area == 1)
rrr , ccc = len(surr_coord),len(surr_coord[0])

surr_temps = [0] * ccc

set_point = 30

for i in range(0,ccc):
     img[int(surr_coord[0][i]), int(surr_coord[1][i])] = set_point
     surr_temps[i] = img[int(surr_coord[0][i]), int(surr_coord[1][i])]

##print('surrounding temperature =', surr_temps)
surr_tmp_max = max(surr_temps)
surr_temp_avg = sum(surr_temps)/len(surr_temps)
print('AVERAGE Surroundng temperature:', surr_temp_avg)

ROTATE_SURR_AREA = cv2.rotate(surr_area,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
##plt.title('Draw new hexagon')
image = ax.imshow(ROTATE_SURR_AREA, cmap = 'jet')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
plt.savefig("CTGP Surrounding Selection.png", bbox_inches="tight", transparent=True)

# --------------------------- Thermal CTGP Frame with surrounding (TESTING CAMOUFLAGE SCENARIO) ----------------------- #

ROTATE_surr_img = cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
##plt.title('Draw new hexagon')
image = ax.imshow(ROTATE_surr_img, cmap = 'jet')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
plt.savefig("GUI THERMAL FRAME SHOW.png", bbox_inches="tight", transparent=True)


# --------------------------- Thermal CTGP Frame with surrounding (REAL BACKGROUND CAMOUFLAGE SCENARIO) ----------------------- #

img2 = genfromtxt('71.csv', delimiter=',')
img2 = cv2.flip(img2, 0)

offset_x = 20
offset_y = 0

# fill surrounding region with value 1
surr_area[30:150,100:205] = 1
# removing the new hexagon coordinate from surrounding coordinates
surr_area[filled_hex_coord[0][:], filled_hex_coord[1][:]] = 0

surr_coord = np.where(surr_area == 1)
rrr , ccc = len(surr_coord),len(surr_coord[0])

surr_temps = [0] * ccc
surr_xx = [0] * ccc
surr_zz = [0] * ccc

# Filling the CTGP surrounding region by real thermal background
for i in range(0,ccc):
     img[int(surr_coord[0][i]), int(surr_coord[1][i])] = img2[int(surr_coord[0][i]) - offset_x, int(surr_coord[1][i]) - offset_y]

CTGP_background_temp = [0]*c
# Acquiring the CTGP background temperature
for i in range(0,c):
     CTGP_background_temp[i] = img2[int(surr_coord[0][i]) - offset_x, int(surr_coord[1][i]) - offset_y]
CTGP_background_AVG_temp = sum(CTGP_background_temp)/len(CTGP_background_temp)

print('CTGP REAL Background average temperature: ', CTGP_background_AVG_temp)

ROTATE_real_surr_img = cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
fig, ax = plt.subplots()
##plt.title('Draw new hexagon')
image = ax.imshow(ROTATE_real_surr_img, cmap = 'jet')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
plt.savefig("GUI THERMAL FRAME SHOW (REAL BACKGROUND).png", bbox_inches="tight", transparent=True)

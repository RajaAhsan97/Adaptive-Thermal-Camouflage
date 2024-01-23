"""
     Code by RMA
     ATC SYSTEM 2

     1. Characteristic Thermo-Graphic Pixel (CTGP) corner points detection for
        drawing the Hexagonal selection shape and dividing the entire selection
        into 6 regions with reference to the placement of TECs on it.
     2. Grabbing the cordinates which lies inside the regions for calculation of temperatures
        
"""


from numpy import genfromtxt
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import imutils


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

# Frames captured for training the model 
# TEMP-2023-01-05T16-56-06-53
# TEMP-2023-01-05T16-33-47-16
# TEMP-2023-01-05T16-33-44-42
# TEMP-2023-01-05T18-00-25-80

# Read frame DATA
img = genfromtxt('TEMP-2023-01-05T16-56-06-53.csv', delimiter = ',')

# Creating Gaussain filter Kernal matrix 
h = matlab_style_gauss2D()

# initialization of frame for storing filtered frame
im = np.zeros((192,256))

# Apply filter to the frame
im = cv2.filter2D(img,-1,h)

# Rotate the frame
IM = cv2.rotate(im,cv2.ROTATE_90_COUNTERCLOCKWISE)

# Display the frame
fig, ax = plt.subplots()
plt.title('Thermal image')
image = ax.imshow(IM,cmap='gray')
plt.xticks([])
plt.yticks([])
fig.show() 

# Acquire the minimum temperature of the frame
tp_min = np.min(im)
# Acquire the miaximum temperature of the frame
tp_max = np.max(im)
print('Min TEMP:', tp_min, 'Max TEMP:', tp_max)
# Normalization of thermal data to GRAY scale values (0-255)
Norm_image = (im - tp_min) * ((255-0) / (tp_max - tp_min)) + 0
# Round off the frame values
round_off = np.round_(Norm_image)

# Rotate the frame
ROUND_OFF = cv2.rotate(round_off,cv2.ROTATE_90_COUNTERCLOCKWISE)

# Display the frame
fig, ax = plt.subplots()
plt.title('Normalized Frame')
image = ax.imshow(ROUND_OFF,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

# Thresholding the frame for acquiring only the corner locations of CTGP
thresh = cv2.threshold(round_off, 150, 255, cv2.THRESH_BINARY)[1]

# Rotate the frame
THRESH = cv2.rotate(thresh,cv2.ROTATE_90_COUNTERCLOCKWISE)

# Display the frame
fig, ax = plt.subplots()
plt.title('Thresholded IMAGE')
image = ax.imshow(THRESH,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

# Enhancing the corner points for acquiring the corner coordinates
thresh_dilate = cv2.dilate(thresh, None, iterations=5)

# Rotate the frame
THRESH_DILATE = cv2.rotate(thresh_dilate,cv2.ROTATE_90_COUNTERCLOCKWISE)

# Display the frame
fig, ax = plt.subplots()
plt.title('Dilated IMAGE')
image = ax.imshow(THRESH_DILATE,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

 
##print(np.info(thresh_dilate))

# Converting FRAME TYPE (from FLOAT64 to UINT8)
thresh_u8 = thresh_dilate.astype(np.uint8)

##print('Image type conversion to uint8')
##print(np.info(thresh_u8))

# Rotate the frame
THRESH_U8 = cv2.rotate(thresh_u8,cv2.ROTATE_90_COUNTERCLOCKWISE)

# Display the frame
fig, ax = plt.subplots()
plt.title('THRESH U8')
image = ax.imshow(THRESH_U8,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

# Determining the no. of contours in the frame
contours = cv2.findContours(thresh_u8.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)
print('No. of contours',contours)
print(np.info(contours))

i = 0
corner_coord = []

rows, cols = len(round_off), len(round_off[0])
print(rows,cols)
IM = np.zeros((rows,cols,3))

# Calculation of the central coordinates of the corners 
for c in contours:
     print('Corner coordinate:', i)
     # calculate moments for each contour
     M = cv2.moments(c)
 
     # calculate x,y coordinate of center
     cX = int(M["m10"] / M["m00"])
     cY = int(M["m01"] / M["m00"])
     print(cX, ',', cY)

     # Mark central coordinates of corners with REd color
     cv2.circle(IM, (cX, cY), 1, (255), cv2.FILLED)

     # Storing the corner corner coordinates
     corner_coord.append((cX, cY))
     i = i+1


## CREATING EDGES OF CTGP by connecting the corner points

# -------------------------- EDGE 1 --------------------------- #
cv2.line(IM, (corner_coord[0][0], corner_coord[0][1]), (corner_coord[1][0], corner_coord[1][1]), (255, 255, 255), thickness=1)

# EDGE 1 distance
dist1 = math.dist(corner_coord[0], corner_coord[1])
dist_mid1 = dist1 // 2
print("DIST 1:", dist1, 'Half dist:', dist_mid1)

# finding the centroid coordinate of EDGE 1
center_x1, center_y1 = ((corner_coord[0][0] + corner_coord[1][0]) // 2), ((corner_coord[0][1] + corner_coord[1][1]) // 2)
print("Center coordinate 1:", center_x1, center_y1)
cv2.circle(IM, (center_x1,center_y1), 2, (0,255,0), thickness=1)

# -------------------------- EDGE 2 --------------------------- #
cv2.line(IM, (corner_coord[1][0], corner_coord[1][1]), (corner_coord[3][0], corner_coord[3][1]), (255, 255, 255), thickness=1)

# EDGE 2 distance
dist2 = math.dist(corner_coord[1], corner_coord[3])
dist_mid2 = dist2 // 2
print("DIST 2:", dist2, 'Half dist:', dist_mid2)

# finding the centroid coordinate of EDGE 2
center_x2, center_y2 = ((corner_coord[1][0] + corner_coord[3][0]) // 2), ((corner_coord[1][1] + corner_coord[3][1]) // 2)
print("Center coordinate 2:", center_x2, center_y2)
cv2.circle(IM, (center_x2,center_y2), 2, (0,255,0), thickness=1)

# -------------------------- EDGE 3 --------------------------- #
cv2.line(IM, (corner_coord[3][0], corner_coord[3][1]), (corner_coord[5][0], corner_coord[5][1]), (255, 255, 255), thickness=1)

# EDGE 3 distance
dist3 = math.dist(corner_coord[3], corner_coord[5])
dist_mid3 = dist3 // 2
print("DIST 3:", dist3, 'Half dist:', dist_mid3)

# finding the centroid coordinate of EDGE 3
center_x3, center_y3 = ((corner_coord[3][0] + corner_coord[5][0]) // 2), ((corner_coord[3][1] + corner_coord[5][1]) // 2)
print("Center coordinate 3:", center_x3, center_y3)
cv2.circle(IM, (center_x3,center_y3), 2, (0,255,0), thickness=1)

# -------------------------- EDGE 4 --------------------------- #
cv2.line(IM, (corner_coord[5][0], corner_coord[5][1]), (corner_coord[4][0], corner_coord[4][1]), (255, 255, 255), thickness=1)

# EDGE 4 distance
dist4 = math.dist(corner_coord[5], corner_coord[4])
dist_mid4 = dist4 // 2
print("DIST 4:", dist4, 'Half dist:', dist_mid4)

# finding the centroid coordinate of EDGE 4
center_x4, center_y4 = ((corner_coord[5][0] + corner_coord[4][0]) // 2), ((corner_coord[5][1] + corner_coord[4][1]) // 2)
print("Center coordinate 4:", center_x4, center_y4)
cv2.circle(IM, (center_x4,center_y4), 2, (0,255,0), thickness=1)

# -------------------------- EDGE 5 --------------------------- #
cv2.line(IM, (corner_coord[4][0], corner_coord[4][1]), (corner_coord[2][0], corner_coord[2][1]), (255, 255, 255), thickness=1)

# EDGE 5 distance
dist5 = math.dist(corner_coord[4], corner_coord[2])
dist_mid5 = dist5 // 2
print("DIST 5:", dist5, 'Half dist:', dist_mid5)

# finding the centroid coordinate of EDGE 5
center_x5, center_y5 = ((corner_coord[4][0] + corner_coord[2][0]) // 2), ((corner_coord[4][1] + corner_coord[2][1]) // 2)
print("Center coordinate 5:", center_x5, center_y5)
cv2.circle(IM, (center_x5,center_y5), 2, (0,255,0), thickness=1)

# -------------------------- EDGE 6 --------------------------- #
cv2.line(IM, (corner_coord[2][0], corner_coord[2][1]), (corner_coord[0][0], corner_coord[0][1]), (255, 255, 255), thickness=1)

# EDGE 6 distance
dist6 = math.dist(corner_coord[2], corner_coord[0])
dist_mid6 = dist6 // 2
print("DIST 6:", dist6, 'Half dist:', dist_mid6)

# finding the centroid coordinate of EDGE 6
center_x6, center_y6 = ((corner_coord[2][0] + corner_coord[0][0]) // 2), ((corner_coord[2][1] + corner_coord[0][1]) // 2)
print("Center coordinate 6:", center_x6, center_y6)
cv2.circle(IM, (center_x6,center_y6), 2, (0,255,0), thickness=1)


# DIVIDING the entire CTGP into 6 regions
cv2.line(IM, (center_x1, center_y1), (center_x4, center_y4),
         (255, 0, 255), thickness=1)

cv2.line(IM, (center_x2, center_y2), (center_x5, center_y5),
         (255, 0, 255), thickness=1)

cv2.line(IM, (center_x3, center_y3), (center_x6, center_y6),
         (255, 0, 255), thickness=1)

# finding CTGP centroid 1 from the two opposite centroid coordinate of Edges
center_x0_1, center_y0_1 = ((center_x1 + center_x4) // 2), ((center_y1 + center_y4) // 2)
center_x0_2, center_y0_2 = ((center_x2 + center_x5) // 2), ((center_y2 + center_y5) // 2)
center_x0_3, center_y0_3 = ((center_x3 + center_x6) // 2), ((center_y3 + center_y6) // 2)

# finding the mean centroid coordinates
center_x0, center_y0 = ((center_x0_1 + center_x0_2 + center_x0_3) // 3), ((center_y0_1 + center_y0_2 + center_y0_3) // 3)

# Mark the centroid coordinate of CTGP
cv2.circle(IM, (center_x0,center_y0), 2, (0,255,0), thickness=1)

# -------------------------------------------------------------------------------------------------------------------------
cv2.line(IM, (corner_coord[0][0], corner_coord[0][1]), (corner_coord[5][0], corner_coord[5][1]),
         (255, 255, 255), thickness=1)
cv2.line(IM, (corner_coord[1][0], corner_coord[1][1]), (corner_coord[4][0], corner_coord[4][1]),
         (255, 255, 255), thickness=1)
cv2.line(IM, (corner_coord[2][0], corner_coord[2][1]), (corner_coord[3][0], corner_coord[3][1]),
         (255, 255, 255), thickness=1)

TEC_region1_mid_X, TEC_region1_mid_Y = ((corner_coord[5][0] + center_x0) // 2), ((corner_coord[5][1] + center_y0) // 2) - 7   # -5

rot_rectangle1 = ((TEC_region1_mid_X, TEC_region1_mid_Y), (10, 10), -135)
box1 = cv2.boxPoints(rot_rectangle1) 
box1 = np.int0(box1) #Convert into integer values
print('TEC1 coordinate: ', box1)
cv2.drawContours(IM,[box1],0,(255,255,255),1)
 
TEC_region2_mid_X, TEC_region2_mid_Y = ((corner_coord[0][0] + center_x0) // 2), ((corner_coord[0][1] + center_y0) // 2) + 7  # +5

rot_rectangle2 = ((TEC_region2_mid_X, TEC_region2_mid_Y), (10, 10), -135)
box2 = cv2.boxPoints(rot_rectangle2) 
box2 = np.int0(box2) #Convert into integer values
print('TEC2 coordinate: ', box2)
cv2.drawContours(IM,[box2],0,(255,255,255),1)

TEC_region3_mid_X, TEC_region3_mid_Y = ((corner_coord[4][0] + center_x0) // 2) + 8, ((corner_coord[4][1] + center_y0) // 2) - 3  # +3,-1

rot_rectangle3 = ((TEC_region3_mid_X, TEC_region3_mid_Y), (10, 10), 14)
box3 = cv2.boxPoints(rot_rectangle3) 
box3 = np.int0(box3) #Convert into integer values
print('TEC3 coordinate: ', box3)
cv2.drawContours(IM,[box3],0,(255,255,255),1)

TEC_region4_mid_X, TEC_region4_mid_Y = ((corner_coord[1][0] + center_x0) // 2) - 6, ((corner_coord[1][1] + center_y0) // 2) + 6   # -3,+3

rot_rectangle4 = ((TEC_region4_mid_X, TEC_region4_mid_Y), (10, 10), 14)
box4 = cv2.boxPoints(rot_rectangle4) 
box4 = np.int0(box4) #Convert into integer values
print('TEC4 coordinate: ', box4)
cv2.drawContours(IM,[box4],0,(255,255,255),1)

TEC_region5_mid_X, TEC_region5_mid_Y = ((corner_coord[3][0] + center_x0) // 2) - 6, ((corner_coord[3][1] + center_y0) // 2) - 2  # -3,-2

rot_rectangle5 = ((TEC_region5_mid_X, TEC_region5_mid_Y), (10, 10), -16)
box5 = cv2.boxPoints(rot_rectangle5) 
box5 = np.int0(box5) #Convert into integer values
print('TEC5 coordinate: ', box5)
cv2.drawContours(IM,[box5],0,(255,255,255),1)

TEC_region6_mid_X, TEC_region6_mid_Y = ((corner_coord[2][0] + center_x0) // 2) + 6, ((corner_coord[2][1] + center_y0) // 2) + 3  # +3,+2

rot_rectangle6 = ((TEC_region6_mid_X, TEC_region6_mid_Y), (10, 10), -16)
box6 = cv2.boxPoints(rot_rectangle6) 
box6 = np.int0(box6) #Convert into integer values
print('TEC6 coordinate: ', box6)
cv2.drawContours(IM,[box6],0,(255,255,255),1)


# -------------------------------------------------------------------------------------------------------------------------

# Rotate the CTGP selection frame
IM_ROTATED = cv2.rotate(IM,cv2.ROTATE_90_COUNTERCLOCKWISE)

# Display the frame
fig, ax = plt.subplots()
plt.title('CTGP Selections')
##image = ax.imshow(IM_ROTATED,cmap = 'gray')
image = ax.imshow(IM,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()     

print('------------------------')
print('CTGP corner coordinates:')
print(corner_coord)
print('------------------------')

###############################################################################################################################
"""  First Approach to divide CTGP selection sub-regions for acquiring average temperatures, which will be used as the process
     variable for PID control algorithm. But the PID controller does not stabalize the temperature of the regions because larger
     area of regions number of pixels inside the region increases, which effects the average temperature of the sub regions.
     1. if the background temperature is 36 [degC], then average temperature of the sub-region respective to the background
        is maintained to that level but the TEC is set to higher temperature. Thus thermal contour of the camou-pixel-II region
        deviate from the background.
        Thus, the second approach is to reduce the process-variable regions of the camou-pixel. Monitoring on the TECs regions
"""

### CTGP REGION 1 selection
##points = np.array([[center_x1,center_y1], [corner_coord[0][0],corner_coord[0][1] ], [center_x6,center_y6], [center_x0,center_y0]])
##cv2.fillPoly(IM, pts = [points], color = (0,0,255))
##
### CTGP REGION 2 selection
##points1 = np.array([[center_x6, center_y6], [corner_coord[2][0], corner_coord[2][1]], [center_x5, center_y5],[center_x0, center_y0]])
##cv2.fillPoly(IM, pts=[points1], color=(0, 255, 255))
##
### CTGP REGION 3 selection
##points2 = np.array([[center_x5, center_y5], [corner_coord[4][0], corner_coord[4][1]], [center_x4, center_y4],[center_x0, center_y0]])
##cv2.fillPoly(IM, pts=[points2], color=(.55, .90, .55))
##
### CTGP REGION 4 selection
##points3 = np.array([[center_x4, center_y4], [corner_coord[5][0], corner_coord[5][1]], [center_x3, center_y3],[center_x0, center_y0]])
##cv2.fillPoly(IM, pts=[points3], color=(.70, .20, .20))
##
### CTGP REGION 5 selection
##points4 = np.array([[center_x3, center_y3], [corner_coord[3][0], corner_coord[3][1]], [center_x2, center_y2],[center_x0, center_y0]])
##cv2.fillPoly(IM, pts=[points4], color=(255, 255, 0))
##
### CTGP REGION 6 selection
##points5 = np.array([[center_x2, center_y2], [corner_coord[1][0], corner_coord[1][1]], [center_x1, center_y1],[center_x0, center_y0]])
##cv2.fillPoly(IM, pts=[points5], color=(.55, .255, .55))
##
##
#################################################################################################################################
"""
     Second Approach
"""

cv2.fillPoly(IM, pts = [box1], color = (.70, .20, .20))
cv2.fillPoly(IM, pts = [box2], color = (0,0,255))
cv2.fillPoly(IM, pts = [box3], color = (.55, .90, .55))
cv2.fillPoly(IM, pts = [box4], color = (.55, .255, .55))
cv2.fillPoly(IM, pts = [box5], color = (255, 255, 0))
cv2.fillPoly(IM, pts = [box6], color = (0, 255, 255))
# Display the frame
fig, ax = plt.subplots()
plt.title('CTGP TEC Selections')
##image = ax.imshow(IM_ROTATED,cmap = 'gray')
image = ax.imshow(IM,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()     

## Grabing coordinates of CTGP regions

# CTGP REGION 1 coordinates
region1_X_coord, region1_Y_coord = np.where(np.all(IM == [0, 0, 255],axis=2))
# print("Region 1:", region1_X_coord)
##cv2.circle(IM, (region1_Y_coord[100], region1_X_coord[100]), 5, (255, 255, 255), thickness=3)

# CTGP REGION 2 coordinates
region2_X_coord, region2_Y_coord = np.where(np.all(IM == [0, 255, 255],axis=2))
# print("Region 1:", region1_X_coord)
##cv2.circle(IM, (region2_Y_coord[100], region2_X_coord[100]), 5, (255, 255, 255), thickness=3)

# CTGP REGION 3 coordinates
region3_X_coord, region3_Y_coord = np.where(np.all(IM == [.55, .90, .55],axis=2))
# print("Region 1:", region1_X_coord)
##cv2.circle(IM, (region3_Y_coord[100], region3_X_coord[100]), 5, (255, 255, 255), thickness=3)

# CTGP REGION 4 coordinates
region4_X_coord, region4_Y_coord = np.where(np.all(IM == [.70, .20, .20],axis=2))
# print("Region 1:", region1_X_coord)
##cv2.circle(IM, (region4_Y_coord[100], region4_X_coord[100]), 5, (255, 255, 255), thickness=3)

# CTGP REGION 5 coordinates
region5_X_coord, region5_Y_coord = np.where(np.all(IM == [255, 255, 0],axis=2))
# print("Region 1:", region1_X_coord)
##cv2.circle(IM, (region5_Y_coord[100], region5_X_coord[100]), 5, (255, 255, 255), thickness=3)

# CTGP REGION 6 coordinates
region6_X_coord, region6_Y_coord = np.where(np.all(IM == [.55, .255, .55],axis=2))
# print("Region 1:", region1_X_coord)
##cv2.circle(IM, (region6_Y_coord[100], region6_X_coord[100]), 5, (255, 255, 255), thickness=3)


## Calculating CTGP REGIONs temperatures

# CTGP REGION 1 average temperature
cc1 = len(region1_X_coord)
temps1 = [0] * cc1
for i in range(0, cc1):
     temps1[i] = img[int(region1_X_coord[i]), int(region1_Y_coord[i])]
tmp_max1 = max(temps1)
tmp_avg1 = sum(temps1) / len(temps1)
print("Region1 TEMP:", tmp_avg1)

# CTGP REGION 2 average temperature
cc2 = len(region2_X_coord)
temps2 = [0] * cc2
for i in range(0, cc2):
     temps2[i] = img[int(region2_X_coord[i]), int(region2_Y_coord[i])]
tmp_max2 = max(temps2)
tmp_avg2 = sum(temps2) / len(temps2)
print("Region2 TEMP:", tmp_avg2)

# CTGP REGION 3 average temperature
cc3 = len(region3_X_coord)
temps3 = [0] * cc3
for i in range(0, cc3):
     temps3[i] = img[int(region3_X_coord[i]), int(region3_Y_coord[i])]
tmp_max3 = max(temps3)
tmp_avg3 = sum(temps3) / len(temps3)
print("Region3 TEMP:", tmp_avg3)

# CTGP REGION 4 average temperature
cc4 = len(region4_X_coord)
temps4 = [0] * cc4
for i in range(0, cc4):
     temps4[i] = img[int(region4_X_coord[i]), int(region4_Y_coord[i])]
tmp_max4 = max(temps4)
tmp_avg4 = sum(temps4) / len(temps4)
print("Region4 TEMP:", tmp_avg4)

# CTGP REGION 5 average temperature
cc5 = len(region5_X_coord)
temps5 = [0] * cc5
for i in range(0, cc5):
     temps5[i] = img[int(region5_X_coord[i]), int(region5_Y_coord[i])]
tmp_max5 = max(temps5)
tmp_avg5 = sum(temps5) / len(temps5)
print("Region5 TEMP:", tmp_avg5)

# CTGP REGION 6 average temperature
cc6 = len(region6_X_coord)
temps6 = [0] * cc6
for i in range(0, cc6):
     temps6[i] = img[int(region6_X_coord[i]), int(region6_Y_coord[i])]
tmp_max6 = max(temps6)
tmp_avg6 = sum(temps6) / len(temps6)
print("Region6 TEMP:", tmp_avg6)


##IM_coord = cv2.rotate(IM,cv2.ROTATE_90_COUNTERCLOCKWISE)
# Display the frame 
##fig, ax = plt.subplots()
##plt.title('CTGP selection REGIONS')
##image = ax.imshow(IM,cmap = 'gray')
##plt.xticks([])
##plt.yticks([])
##fig.show()     

"""
     Surrounding AREA defining
"""
SURR_IM = np.zeros((rows, cols, 3))
CTGP_IM = np.zeros((rows, cols, 3))

upper_left_coordX, upper_left_coordY = corner_coord[3][0] - 10, corner_coord[5][1] - 10
upper_right_coordX, upper_right_coordY = corner_coord[4][0] + 10, corner_coord[5][1] - 10
bottom_left_coordX, bottom_left_coordY = corner_coord[3][0] - 10, corner_coord[0][1] + 10
bottom_right_coordX, bottom_right_coordY = corner_coord[4][0] + 10, corner_coord[0][1] + 10

SURR_points = np.array([[upper_left_coordX, upper_left_coordY], [bottom_left_coordX, bottom_left_coordY],
                        [bottom_right_coordX, bottom_right_coordY],[upper_right_coordX, upper_right_coordY],
                        [upper_left_coordX, upper_left_coordY]])
cv2.fillPoly(SURR_IM, pts = [SURR_points], color = (255,255,255))

SURR_coord = np.where(SURR_IM == 255)
print('SURR AREA', SURR_coord)

fig, ax = plt.subplots()
plt.title('CTGP SURROUNDING CREATION')
image = ax.imshow(SURR_IM,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()     

CTGP_points = np.array([[corner_coord[0][0], corner_coord[0][1]], [corner_coord[1][0], corner_coord[1][1]],
                        [corner_coord[3][0], corner_coord[3][1]], [corner_coord[5][0], corner_coord[5][1]],
                        [corner_coord[4][0], corner_coord[4][1]], [corner_coord[2][0], corner_coord[2][1]],
                        [corner_coord[0][0], corner_coord[0][1]]])
cv2.fillPoly(CTGP_IM, pts = [CTGP_points], color = (255,255,255))

CTGP_coord = np.where(CTGP_IM == 255)
print('CTGP AREA', CTGP_coord)

fig, ax = plt.subplots()
plt.title('CTGP VIEW')
image = ax.imshow(CTGP_IM,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

# removing the new hexagon coordinate from surrounding coordinates
SURR_IM[CTGP_coord[0][:], CTGP_coord[1][:]] = 0

##############################
m1 = (center_y4 - center_y1)/(center_x4 - center_x1)

surr_x1 = int(np.round_(((bottom_left_coordY - center_y1)/m1) + center_x1))  # EXTRAPOLATINO COORDINATED FORMULA APPLIEd
print(surr_x1, bottom_left_coordY)
surr_x2 = int(np.round_(((upper_right_coordY - center_y4)/m1) + center_x4))
print(surr_x2, upper_right_coordY)

cv2.line(SURR_IM, (center_x1, center_y1), (surr_x1, bottom_left_coordY),(255, 255, 0), thickness=1)
cv2.line(SURR_IM, (center_x4, center_y4), (surr_x2, upper_right_coordY),(255, 255, 0), thickness=1)
##############################
m2 = (center_y6 - center_y3)/(center_x6 - center_x3)

surr_x3 = int(np.round_(((bottom_left_coordY - center_y3)/m2) + center_x3))
print(surr_x3, bottom_left_coordY)
surr_x4 = int(np.round_(((upper_right_coordY - center_y6)/m2) + center_x6))
print(surr_x4, upper_right_coordY)

cv2.line(SURR_IM, (center_x6, center_y6), (surr_x3, bottom_left_coordY),(255, 255, 0), thickness=1)
cv2.line(SURR_IM, (center_x3, center_y3), (surr_x4, upper_right_coordY),(255, 255, 0), thickness=1)
##############################
m3 = (center_y5 - center_y2)/(center_x5 - center_x2)

surr_y1 = int(np.round_(center_y2 + ((upper_left_coordX - center_x2)*m3)))
print(upper_left_coordX,surr_y1)
surr_y2 = int(np.round_(center_y5 + ((upper_right_coordX - center_x5)*m3)))
print(upper_right_coordX,surr_y2)

cv2.line(SURR_IM, (center_x2, center_y2), (upper_left_coordX, surr_y1),(255, 255, 0), thickness=1)
cv2.line(SURR_IM, (center_x5, center_y5), (upper_right_coordX, surr_y2),(255, 255, 0), thickness=1)

##############################
##cv2.line(SURR_IM, (center_x2, center_y2), (center_x5, center_y5),(255, 255, 0), thickness=1)
##print('x1:', center_x2, ' y1:', center_y2)
##print('x4:', center_x5, ' y4:', center_y5)
##print('SLOPE:', m3)
fig, ax = plt.subplots()
plt.title('SURR SURROUNDING DIVIDED')
image = ax.imshow(SURR_IM,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

# Filling the surrounding regions

# Surrounding region 1
SURR_region1_points = np.array([[center_x2, center_y2], [corner_coord[1][0], corner_coord[1][1]],
                        [center_x1, center_y1],[surr_x1, bottom_left_coordY],
                        [bottom_left_coordX, bottom_left_coordY], [upper_left_coordX, surr_y1]])
cv2.fillPoly(SURR_IM, pts = [SURR_region1_points], color = (255,255,0))

# Surrounding region 2
SURR_region2_points = np.array([[center_x2, center_y2], [corner_coord[3][0], corner_coord[3][1]],
                        [center_x3, center_y3],[surr_x4, upper_left_coordY],
                        [upper_left_coordX, upper_left_coordY], [upper_left_coordX, surr_y1]])
cv2.fillPoly(SURR_IM, pts = [SURR_region2_points], color = (255,0,0))

# Surrounding region 3
SURR_region3_points = np.array([[center_x3, center_y3], [corner_coord[5][0], corner_coord[5][1]],
                        [center_x4, center_y4],[surr_x2, upper_right_coordY],
                        [surr_x4, upper_left_coordY], [center_x3, center_y3]])
cv2.fillPoly(SURR_IM, pts = [SURR_region3_points], color = (0,255,0))

# Surrounding region 4
SURR_region4_points = np.array([[center_x5, center_y5], [corner_coord[4][0], corner_coord[4][1]],
                        [center_x4, center_y4],[surr_x2, upper_right_coordY],
                        [upper_right_coordX, upper_right_coordY], [upper_right_coordX, surr_y2]])
cv2.fillPoly(SURR_IM, pts = [SURR_region4_points], color = (0,0,255))

# Surrounding region 5
SURR_region5_points = np.array([[center_x5, center_y5], [corner_coord[2][0], corner_coord[2][1]],
                        [center_x6, center_y6],[surr_x3, bottom_left_coordY],
                        [bottom_right_coordX, bottom_right_coordY], [upper_right_coordX, surr_y2]])
cv2.fillPoly(SURR_IM, pts = [SURR_region5_points], color = (0,255,255))

# Surrounding region 6
SURR_region6_points = np.array([[center_x1, center_y1], [corner_coord[0][0], corner_coord[0][1]],
                        [center_x6, center_y6],[surr_x3, bottom_left_coordY],
                        [surr_x1, bottom_left_coordY], [center_x1, center_y1]])
cv2.fillPoly(SURR_IM, pts = [SURR_region6_points], color = (255,0,255))



## Grabing coordinates of CTGP SURROUNDING regions

# CTGP SURROUNDING REGION 1 coordinates
SURR_region1_X_coord, SURR_region1_Y_coord = np.where(np.all(SURR_IM == [255,255,0],axis=2))
cv2.circle(SURR_IM, (SURR_region1_Y_coord[50], SURR_region1_X_coord[50]), 3, (255, 255, 255), thickness=3)

# CTGP SURROUNDING REGION 2 coordinates
SURR_region2_X_coord, SURR_region2_Y_coord = np.where(np.all(SURR_IM == [255,0,0],axis=2))
cv2.circle(SURR_IM, (SURR_region2_Y_coord[50], SURR_region2_X_coord[50]), 3, (255, 255, 255), thickness=3)

# CTGP SURROUNDING REGION 3 coordinates
SURR_region3_X_coord, SURR_region3_Y_coord = np.where(np.all(SURR_IM == [0,255,0],axis=2))
cv2.circle(SURR_IM, (SURR_region3_Y_coord[50], SURR_region3_X_coord[50]), 3, (255, 255, 255), thickness=3)

# CTGP SURROUNDING REGION 4 coordinates
SURR_region4_X_coord, SURR_region4_Y_coord = np.where(np.all(SURR_IM == [0,0,255],axis=2))
cv2.circle(SURR_IM, (SURR_region4_Y_coord[50], SURR_region4_X_coord[50]), 3, (255, 255, 255), thickness=3)

# CTGP SURROUNDING REGION 5 coordinates
SURR_region5_X_coord, SURR_region5_Y_coord = np.where(np.all(SURR_IM == [0,255,255],axis=2))
cv2.circle(SURR_IM, (SURR_region5_Y_coord[50], SURR_region5_X_coord[50]), 3, (255, 255, 255), thickness=3)

# CTGP SURROUNDING REGION 6 coordinates
SURR_region6_X_coord, SURR_region6_Y_coord = np.where(np.all(SURR_IM == [255,0,255],axis=2))
cv2.circle(SURR_IM, (SURR_region6_Y_coord[50], SURR_region6_X_coord[50]), 3, (255, 255, 255), thickness=3)

fig, ax = plt.subplots()
plt.title('SURR SURROUNDING regions')
image = ax.imshow(SURR_IM,cmap = 'gray')
plt.xticks([])
plt.yticks([])
fig.show()

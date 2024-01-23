"""
    Code by RMA

    computing the Image Quality Assessment (IQA) metrics of camou-pixel-II
    regions located on the CO with their background region. Following IQAs are
    utilized:
    1. SSIM (Structural similarity index measure)
    2. FSIM (Feature based similarity index measure)
    3. PSNR (Peak signal to noise ratio) dB
    4. RMSE (Root mean square error)
    5. UIQ  (Universal image quality index)
    6. SRE  (Signal to reconstruction error) dB

    IQA metrics are adopted to compare the thermal gradiant developed on the camou-pixel
    with the background region 
"""
import numpy as np
import matplotlib.pyplot as plt
import cv2
from numpy import genfromtxt
from scipy import ndimage
import imutils
import math
import pandas as pd
from pyhelpers.store import save_fig
import image_similarity_measures
from image_similarity_measures.quality_metrics import rmse, psnr, ssim, fsim, uiq, issm, sre


# Load TBGs frame and convert to grayscale 
img_TBGs = genfromtxt('86.csv', delimiter=',')
img_TBGs = cv2.rotate(img_TBGs, cv2.ROTATE_90_COUNTERCLOCKWISE)
up_scale = (192 * 12, 256 * 12)
IMG2 = cv2.resize(img_TBGs, up_scale, interpolation=cv2.INTER_LINEAR)

img2 = cv2.resize(img_TBGs, up_scale, interpolation=cv2.INTER_LINEAR)


fig, ax = plt.subplots()
plt.title('TBGS Frame')
image = ax.imshow(IMG2,cmap='jet')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show()
save_fig("TBGS frame (Original).svg", dpi=600, verbose=True, conv_svg_to_emf=True)

tp_min = img2.min()
tp_max = img2.max()
print('TBGS Frame')
print('T_min:', tp_min, 'T_max:', tp_max)
# normalization of thermal data to Grey scale values (0-255)
Norm_image = (img2 - tp_min) * ((255-0) / (tp_max - tp_min)) + 0
round_off = np.round_(Norm_image)

fig, ax = plt.subplots()
plt.title('TBGS Frame (Gray scale)')
image = ax.imshow(round_off,cmap='jet')
plt.xticks([])
plt.yticks([]) 
plt.colorbar(image)
fig.show()

OFFSET_x = 890
OFFSET_y = 60+35

Sim_data_file = str('LOC1_CV90')
string1 = Sim_data_file + str('.txt')
string2 = Sim_data_file + str('.csv')
# Load CTGP-1 array temperature simulated data and find the coordinates of the geometry 
read_file = pd.read_csv(string1)
read_file.to_csv (string2, index=None)
CTGP_1_SIM = genfromtxt(string2, delimiter=',')

step = 0
CHUNKS = 1

SSIM_arr = [0] * (250)
PSNR_arr = [0] * (250)
RMSE_arr = [0] * (250)
FSIM_arr = [0] * (250)
UIQ_arr = [0] * (250)
##ISSM_arr = [0] * (len(OFFSET_x) * len(OFFSET_y))
SRE_arr = [0] * (250)

y_offset = [60+35, 60+0,60+35,60+0,60+35,60+0,60+(3*35),60+(4*35),60+(5*35),60+(6*35)]
no_of_px = [18, 22, 25, 28, 29, 30, 28, 26, 23, 21]

thermal_color = 5

for LOC_y in range(0,10):
    offset_x = OFFSET_x
    offset_y = y_offset[LOC_y]

    for LOC_x in range(0, no_of_px[LOC_y]):
        print('LOCx:', offset_y, ',', 'LOCy:', offset_x)
        
        CTGP_1_SIM_temp = CTGP_1_SIM[:,3+step]
        CTGP_1_SIM_temp_reshape = np.reshape(CTGP_1_SIM_temp, [201,241])  # CTGP-1 (ARRAY) [201,241]      # [181,201]
        CTGP_1_SIM_temp_reshape = cv2.rotate(CTGP_1_SIM_temp_reshape, cv2.ROTATE_90_COUNTERCLOCKWISE)

        ROW, COL = int(len(CTGP_1_SIM_temp_reshape[0])/2.5), int(len(CTGP_1_SIM_temp_reshape)/2.5)

        CTGP_downscale_scale = (ROW, COL)

        CTGP_1_SIM_temp_reshape = cv2.resize(CTGP_1_SIM_temp_reshape, CTGP_downscale_scale, interpolation=cv2.INTER_LINEAR)

        CTGP_1_SIM_temp_reshape[np.isnan(CTGP_1_SIM_temp_reshape)] = 273.15

        CTGP_1_SIM_temp_reshape_degC = CTGP_1_SIM_temp_reshape - 273.15

        ##CTGP_1_SIM_temp_reshape_degC = cv2.rotate(CTGP_1_SIM_temp_reshape_degC, cv2.ROTATE_90_CLOCKWISE)

        coord = np.where((CTGP_1_SIM_temp_reshape_degC > 0.0) & (CTGP_1_SIM_temp_reshape_degC <= CTGP_1_SIM_temp_reshape_degC.max() ))

        CTGP_II_min_temp = CTGP_1_SIM_temp_reshape_degC[coord[0], coord[1]].min()
        CTGP_II_max_temp = CTGP_1_SIM_temp_reshape_degC[coord[0], coord[1]].max()
        
        coord_CTGP_simulation = np.where((CTGP_1_SIM_temp_reshape_degC >= 0.0) & (CTGP_1_SIM_temp_reshape_degC <= CTGP_II_max_temp))
        coord_outside = np.where((CTGP_1_SIM_temp_reshape_degC >= 0.0) & (CTGP_1_SIM_temp_reshape_degC < CTGP_II_min_temp))

        CTGP1_tp_min = CTGP_1_SIM_temp_reshape_degC.min()
        CTGP1_tp_max = CTGP_1_SIM_temp_reshape_degC.max()
        # normalization of thermal data to Grey scale values (0-255)
        CTGP1_Norm_image = (CTGP_1_SIM_temp_reshape_degC - CTGP1_tp_min) * ((255-0) / (CTGP1_tp_max - CTGP1_tp_min)) + 0
        CTGP1_round_off = np.round_(CTGP1_Norm_image)
        CTGP1_round_off = np.uint8(CTGP1_round_off)

        Edgecanny = cv2.Canny(CTGP1_round_off, 200.0, 255.0)
        ##Edgecanny = cv2.rotate(Edgecanny, cv2.ROTATE_90_CLOCKWISE)

        thresh = cv2.threshold(Edgecanny, 150, 255, cv2.THRESH_BINARY)[1]
        thresh_dilate = cv2.dilate(thresh, None, iterations=5)
        thresh_u8 = thresh_dilate.astype(np.uint8)
        contours = cv2.findContours(thresh_u8.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        i = 0
        corner_coord = []
        rows, cols = len(CTGP1_round_off), len(CTGP1_round_off[0])
        IM = np.zeros((rows,cols,3))

        # Calculation of the central coordinates of the corners 
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Mark central coordinates of corners with REd color
            cv2.circle(Edgecanny, (cX, cY), 1, (255), cv2.FILLED)
            # Storing the corner corner coordinates
            corner_coord.append((cX, cY))
            i = i+1


        val_gray = np.where(Edgecanny == 255)
        Edgecanny[val_gray[0][:],val_gray[1][:]] = 1
        detect_hex = ndimage.binary_fill_holes(Edgecanny).astype(int)
        fill_detect_hex = np.where(detect_hex == 1)
##        print(fill_detect_hex)
        detect_hex[cY,cX] = 0.0

        CY = cY
        CX = cX
        while True:
            CY =  CY - 1
            if detect_hex[CY,CX] == 0.0:
                break;
            elif detect_hex[CY,CX] == 1.0:
                detect_hex[CY,CX] = 0

        CY1 = cY
        CX1 = cX
        while True:
            CY1 =  CY1 + 1
            if detect_hex[CY1,CX1] == 0.0:
                break;
            elif detect_hex[CY1,CX1] == 1.0:
                detect_hex[CY1,CX1] = 0

        vertical_dist = math.dist([CX1,CY1],[CX,CY])
        half_vert_dist = int(vertical_dist//2) + CY
        cv2.circle(Edgecanny, (cX,half_vert_dist), 2, (1), cv2.FILLED)

        onethird_vert_dist = int(vertical_dist//3) + CY
        cv2.circle(Edgecanny, (cX,onethird_vert_dist), 2, (1), cv2.FILLED)

        onesixth_vert_dist = int(vertical_dist//6) + CY
        cv2.circle(Edgecanny, (cX,onesixth_vert_dist), 2, (1), cv2.FILLED)

        diff = cY - onethird_vert_dist

        d = cY + diff
        cv2.circle(Edgecanny, (cX,d), 2, (1), cv2.FILLED)

        d1 = d + diff
        cv2.circle(Edgecanny, (cX,d1), 2, (1), cv2.FILLED)

        # TEC shape
        D = (d - onethird_vert_dist) * (40/100)
        TEC_dim = int((d - onethird_vert_dist) - D)

        rot_rectangle2 = ((cX, d1), (TEC_dim, TEC_dim), -135)
        box2 = cv2.boxPoints(rot_rectangle2) 
        box2 = np.int0(box2) #Convert into integer values
        cv2.drawContours(Edgecanny,[box2],0,(1),1)
        ##cv2.circle(Edgecanny, [cX,d1],8,(1),1)

        rot_rectangle3 = ((cX, onesixth_vert_dist), (TEC_dim, TEC_dim), -135)
        box3 = cv2.boxPoints(rot_rectangle3) 
        box3 = np.int0(box3) #Convert into integer values
        cv2.drawContours(Edgecanny,[box3],0,(1),1)
        ##cv2.circle(Edgecanny, [cX,onesixth_vert_dist],8,(1),1)

        CY2 = onethird_vert_dist
        CX2 = cX
        while True:
            CX2 =  CX2 + 1
            if detect_hex[CY2,CX2] == 0.0:
                break;
            elif detect_hex[CY2,CX2] == 1.0:
                detect_hex[CY2,CX2] = 0

        CY3 = onethird_vert_dist
        CX3 = cX
        while True:
            CX3 =  CX3 - 1
            if detect_hex[CY3,CX3] == 0.0:
                break;
            elif detect_hex[CY3,CX3] == 1.0:
                detect_hex[CY3,CX3] = 0

        horizontal_dist1 = math.dist([CX3,CY3],[CX2,CY2])
        half_horizontal_dist1 = int(horizontal_dist1//2) + CX3
        cv2.circle(Edgecanny, (half_horizontal_dist1, CY2), 2, (1), cv2.FILLED)

        left_horizontal_dist1 = int(horizontal_dist1//5) + CX3
        cv2.circle(Edgecanny, (left_horizontal_dist1, CY2), 2, (1), cv2.FILLED)

        diff1 = cX - left_horizontal_dist1

        right_horizontal_dist1 = cX + diff1
        cv2.circle(Edgecanny, (right_horizontal_dist1, CY2), 2, (1), cv2.FILLED)

        rot_rectangle4 = ((left_horizontal_dist1, CY2), (TEC_dim, TEC_dim), -14)
        box4 = cv2.boxPoints(rot_rectangle4) 
        box4 = np.int0(box4) #Convert into integer values
        cv2.drawContours(Edgecanny,[box4],0,(1),1)
        ##cv2.circle(Edgecanny, [left_horizontal_dist1, CY2],8,(1),1)

        rot_rectangle5 = ((right_horizontal_dist1, CY2), (TEC_dim, TEC_dim), 16)
        box5 = cv2.boxPoints(rot_rectangle5) 
        box5 = np.int0(box5) #Convert into integer values
        cv2.drawContours(Edgecanny,[box5],0,(1),1)
        ##cv2.circle(Edgecanny, [right_horizontal_dist1, CY2],8,(1),1)

        CY4 = d
        CX4 = cX
        while True:
            CX4 =  CX4 + 1
            if detect_hex[CY4,CX4] == 0.0:
                break;
            elif detect_hex[CY4,CX4] == 1.0:
                detect_hex[CY4,CX4] = 0

        CY5 = d
        CX5 = cX
        while True:
            CX5 =  CX5 - 1
            if detect_hex[CY5,CX5] == 0.0:
                break;
            elif detect_hex[CY5,CX5] == 1.0:
                detect_hex[CY5,CX5] = 0

        horizontal_dist2 = math.dist([CX5,CY5],[CX4,CY4])
        half_horizontal_dist2 = int(horizontal_dist2//2) + CX5
        cv2.circle(Edgecanny, (half_horizontal_dist2, CY4), 2, (1), cv2.FILLED)

        left_horizontal_dist2 = int(horizontal_dist2//5) + CX5
        cv2.circle(Edgecanny, (left_horizontal_dist2, CY4), 2, (1), cv2.FILLED)

        diff2 = cX - left_horizontal_dist2

        right_horizontal_dist2 = cX + diff2
        cv2.circle(Edgecanny, (right_horizontal_dist2, CY4), 2, (1), cv2.FILLED)

        rot_rectangle6 = ((left_horizontal_dist2, CY4), (TEC_dim, TEC_dim), 14)
        box6 = cv2.boxPoints(rot_rectangle6) 
        box6 = np.int0(box6) #Convert into integer values
        cv2.drawContours(Edgecanny,[box6],0,(1),1)
        ##cv2.circle(Edgecanny, [left_horizontal_dist2, CY4],8,(1),1)

        rot_rectangle7 = ((right_horizontal_dist2, CY4), (TEC_dim, TEC_dim), -16)
        box7 = cv2.boxPoints(rot_rectangle7) 
        box7 = np.int0(box7) #Convert into integer values
        cv2.drawContours(Edgecanny,[box7],0,(1),1)

        IM = np.zeros((len(Edgecanny), len(Edgecanny[0]),3))
        IM[:,:,0] = Edgecanny

        ##cv2.fillPoly(IM, pts = [box1], color = (.70, .20, .20))
        cv2.fillPoly(IM, pts = [box2], color = (0,0,255))
        cv2.fillPoly(IM, pts = [box3], color = (.55, .90, .55))
        cv2.fillPoly(IM, pts = [box4], color = (.55, .255, .55))
        cv2.fillPoly(IM, pts = [box5], color = (255, 255, 0))
        cv2.fillPoly(IM, pts = [box6], color = (0, 255, 255))
        cv2.fillPoly(IM, pts = [box7], color = (100, 255, 255))


        ### TEC 1 coordinates
        ##region1_X_coord, region1_Y_coord = np.where(np.all(IM == [.70, .20, .20],axis=2))
        # TEC 1 coordinates
        region1_X_coord, region1_Y_coord = np.where(np.all(IM == [.55, .90, .55],axis=2))
        # TEC 2 coordinates
        region2_X_coord, region2_Y_coord = np.where(np.all(IM == [255, 255, 0],axis=2))
        # TEC 3 coordinates
        region3_X_coord, region3_Y_coord = np.where(np.all(IM == [100, 255, 255],axis=2))
        # TEC 4 coordinates
        region4_X_coord, region4_Y_coord = np.where(np.all(IM == [0,0,255],axis=2))
        # TEC 5 coordinates
        region5_X_coord, region5_Y_coord = np.where(np.all(IM == [0, 255, 255],axis=2))
        # TEC 6 coordinates
        region6_X_coord, region6_Y_coord = np.where(np.all(IM == [.55, .255, .55],axis=2))

        # TEC 1 average temperature
        cc2 = len(region1_X_coord)
        temps2 = [0] * cc2
        for i in range(0, cc2):
            temps2[i] = img2[int(region1_X_coord[i]) + offset_x, int(region1_Y_coord[i]) + offset_y]
        tmp_max2 = max(temps2)
        tmp_avg2 = sum(temps2) / len(temps2)
##        print("TEC2 TEMP:", "{:.2f}".format(tmp_avg2))

        # TEC 2 average temperature
        cc3 = len(region2_X_coord)
        temps3 = [0] * cc3
        for i in range(0, cc3):
            temps3[i] = img2[int(region2_X_coord[i]) + offset_x, int(region2_Y_coord[i]) + offset_y]
        tmp_max3 = max(temps3)
        tmp_avg3 = sum(temps3) / len(temps3)
##        print("TEC3 TEMP:", "{:.2f}".format(tmp_avg3))

        # TEC 3 average temperature
        cc4 = len(region3_X_coord)
        temps4 = [0] * cc4
        for i in range(0, cc4):
            temps4[i] = img2[int(region3_X_coord[i]) + offset_x, int(region3_Y_coord[i]) + offset_y]
        tmp_max4 = max(temps4)
        tmp_avg4 = sum(temps4) / len(temps4)
##        print("TEC4 TEMP:", "{:.2f}".format(tmp_avg4))

        # TEC 4 average temperature
        cc5 = len(region4_X_coord)
        temps5 = [0] * cc5
        for i in range(0, cc5):
            temps5[i] = img2[int(region4_X_coord[i]) + offset_x, int(region4_Y_coord[i]) + offset_y]
        tmp_max5 = max(temps5)
        tmp_avg5 = sum(temps5) / len(temps5)
##        print("TEC5 TEMP:", "{:.2f}".format(tmp_avg5))

        # TEC 5 average temperature
        cc6 = len(region5_X_coord)
        temps6 = [0] * cc6
        for i in range(0, cc6):
            temps6[i] = img2[int(region5_X_coord[i]) + offset_x, int(region5_Y_coord[i]) + offset_y]
        tmp_max6 = max(temps6)
        tmp_avg6 = sum(temps6) / len(temps6)
##        print("TEC6 TEMP:", "{:.2f}".format(tmp_avg6))

        # TEC 6 average temperature
        cc7 = len(region6_X_coord)
        temps7 = [0] * cc7
        for i in range(0, cc7):
            temps7[i] = img2[int(region6_X_coord[i]) + offset_x, int(region6_Y_coord[i]) + offset_y]
        tmp_max7 = max(temps7)
        tmp_avg7 = sum(temps7) / len(temps7)
##        print("TEC7 TEMP:", "{:.2f}".format(tmp_avg7))

        r , c = len(coord),len(coord[0])

        IMAGE_REGION = IMG2[coord_CTGP_simulation[0].min() + offset_x: coord_CTGP_simulation[0].max() + offset_x, coord_CTGP_simulation[1].min() + offset_y: coord_CTGP_simulation[1].max() + offset_y]
        IMAGE_region = np.zeros((coord_CTGP_simulation[0].max(), coord_CTGP_simulation[1].max()))
        IMAGE_region[coord[0], coord[1]] = IMAGE_REGION[coord[0], coord[1]]
    
        for i in range(0,c):
            img2[coord[0][i] + offset_x, coord[1][i] + offset_y] = CTGP_1_SIM_temp_reshape_degC[coord[0][i], coord[1][i]]

        IMG2_BLENDED_CHUNK = img2[coord_CTGP_simulation[0].min() + offset_x: coord_CTGP_simulation[0].max() + offset_x, coord_CTGP_simulation[1].min() + offset_y: coord_CTGP_simulation[1].max() + offset_y]
        IMG2_blended_chunk = np.zeros((coord_CTGP_simulation[0].max(), coord_CTGP_simulation[1].max()))
        IMG2_blended_chunk[coord[0], coord[1]] = IMG2_BLENDED_CHUNK[coord[0], coord[1]]
        
        ##  ------------------ SSIM COMPUTATION ---------------- 
        C1 = (0.01 * 255) ** 2
        C2 = (0.03 * 255) ** 2

        ##print('IMAGE222: ', len(img222), 'X', len(img222[0]))
        ##print('IMAGE2: ', len(im1), 'X', len(im1[0]))

        IMAGE1 = IMAGE_region.astype(np.float32) # img2_blended_ref1
        IMAGE2 = IMG2_blended_chunk.astype(np.float32)
        kernel = cv2.getGaussianKernel(11, 1.5)
        window = np.outer(kernel, kernel.transpose())

        mu1 = cv2.filter2D(IMAGE1, -1, window)[5:-5, 5:-5]  # valid
        mu2 = cv2.filter2D(IMAGE2, -1, window)[5:-5, 5:-5]
        mu1_sq = mu1 ** 2
        mu2_sq = mu2 ** 2
        mu1_mu2 = mu1 * mu2
        sigma1_sq = cv2.filter2D(IMAGE1 ** 2, -1, window)[5:-5, 5:-5] - mu1_sq
        sigma2_sq = cv2.filter2D(IMAGE2 ** 2, -1, window)[5:-5, 5:-5] - mu2_sq
        sigma12 = cv2.filter2D(IMAGE1 * IMAGE2, -1, window)[5:-5, 5:-5] - mu1_mu2

        ssim_map = ((2 * mu1_mu2 + C1) * (2 * sigma12 + C2)) / ((mu1_sq + mu2_sq + C1) * (sigma1_sq + sigma2_sq + C2))

        ## ------------------PSNR COMPUTATION-------------------
        mse = np.mean((IMAGE_region - IMG2_blended_chunk) ** 2)
        max_pixel = 255.0
        psnr = 20 * math.log10(max_pixel / math.sqrt(mse))

        ## -----------------------------------------------------

        ## ------------------RMSE COMPUTATION-------------------
        rmse = np.sqrt(mse)

        ## -----------------------------------------------------        

        A1 = np.expand_dims(IMAGE_region, axis = -1)
        A2 = np.expand_dims(IMG2_blended_chunk, axis=-1)

##        SSIM = ssim(A1,A2)
        FSIM = fsim(A1,A2)
        UIQ = uiq(A1,A2)
##        ISSM = issm(A1,A2)
        SRE = sre(A1,A2)

        print(f"PSNR value is {psnr} dB")
        print(f"RMSE value is {rmse}")
        print(f"SSIM value is {ssim_map.mean()}")
        print(f"FSIM value is {FSIM}")
        print(f"UIQ value is {UIQ}")
##        print(f"ISSM value is {ISSM}")
        print(f"SRE value is {SRE}")
        print('-------------------------')

        SSIM_arr[CHUNKS-1] = ssim_map.mean()
        PSNR_arr[CHUNKS-1] = psnr
        RMSE_arr[CHUNKS-1] = rmse
        FSIM_arr[CHUNKS-1] = FSIM
        UIQ_arr[CHUNKS-1] = UIQ
##        ISSM_arr[CHUNKS-1] = ISSM
        SRE_arr[CHUNKS-1] = SRE

        offset_y = offset_y + 70
        CHUNKS = CHUNKS + 1
        
        step = step + 1
    OFFSET_x = OFFSET_x + 60




SSIM_arr = np.array(SSIM_arr)
PSNR_arr = np.array(PSNR_arr)
RMSE_arr = np.array(RMSE_arr)
FSIM_arr = np.array(FSIM_arr)
UIQ_arr = np.array(UIQ_arr)
##ISSM_arr = np.array(ISSM_arr)
SRE_arr = np.array(SRE_arr)

IQA_metrics = [SSIM_arr, PSNR_arr, RMSE_arr, FSIM_arr, UIQ_arr, SRE_arr]
##f.write(str(SSIM_arr) + '\n')
##f.write(str(PSNR_arr) + '\n')
##f.write(str(RMSE_arr) + '\n')

np.savetxt('CTGP-II TBGS 86 Locations (IQA).txt', IQA_metrics, fmt='%s')
##f.close()

fig, ax = plt.subplots()
plt.title('CTGP-I Bending on TBGS frame')
image = ax.imshow(img2,cmap='jet')
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()
save_fig("TBGS frame (Blended).svg", dpi=600, verbose=True, conv_svg_to_emf=True)


fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION')
image = ax.imshow(IMAGE_REGION,cmap='jet', vmin = tp_min, vmax = tp_max)
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()

fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - BLENDED CTGP-II')
image = ax.imshow(IMG2_BLENDED_CHUNK,cmap='jet', vmin = tp_min, vmax = tp_max)
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()


fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - CTGP-II BEYOND SELECTION (CB TEMP -> 0 to max)')
image = ax.imshow(IMAGE_region,cmap='jet', vmin = 0, vmax = tp_max)
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()

fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - CTGP-II SELECTION (CB TEMP -> 0 to max)')
image = ax.imshow(IMG2_blended_chunk,cmap='jet', vmin = 0, vmax = tp_max)
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()

fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - CTGP-II BEYOND SELECTION (CB TEMP -> image min to max)')
image = ax.imshow(IMAGE_region,cmap='jet', vmin = tp_min, vmax = tp_max)
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()

fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - CTGP-II SELECTION (CB TEMP -> image min to max)')
image = ax.imshow(IMG2_blended_chunk,cmap='jet', vmin = tp_min, vmax = tp_max)
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()


fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - CTGP-II BEYOND SELECTION (CB TEMP -> region min to max)')
image = ax.imshow(IMAGE_region,cmap='jet', vmin = IMAGE_REGION.min(), vmax = IMAGE_REGION.max())
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()

fig, ax = plt.subplots()
plt.title('TBGS FRAME REGION - CTGP-II SELECTION (CB TEMP -> region min to max)')
image = ax.imshow(IMG2_blended_chunk,cmap='jet', vmin = IMAGE_REGION.min(), vmax = IMAGE_REGION.max())
plt.xticks([])
plt.yticks([])
plt.colorbar(image)
fig.show()

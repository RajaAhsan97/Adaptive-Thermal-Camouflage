#!/usr/bin/env python3

"""
    This script is for HT-301, T3S & T2L downloaded from EEVblog
    COMMENTS in:
    1.   "## []" is written by AHSAN for consideration for building script in MATLAB
    2.   script marked between ####..... is to conversion to MATLAB

    For T2L (replace the resotion 292*384 by 196*256)
"""


"""Example Program for XTherm T3s Camera using OpenCV"""
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime
import math
import operator
import os
import sys
import time
from ctypes import *  ## ['ctypes' is a function library for PYTHON. It provides C compatible datatype & support calling functions in DLL  (see this for MATLAB)]
from functools import reduce
from random import randint
from threading import *

import cv2
import numpy as np
import pyqtgraph as pg
from pyqtgraph import PlotWidget, plot
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
# loading decimal library
from matplotlib.figure import Figure
from numpy import genfromtxt
from scipy import ndimage
from scipy.ndimage import binary_fill_holes
from scipy.spatial import cKDTree
from scipy.spatial.distance import pdist
from numpy import *
from collections import deque
from test import *
import csv
###################################################### For MATLAB ###################################################################
is_64bits = sys.maxsize > 2**32
print('Loading binary 64 bit version of XthermDLL.dll file')
XthermDll = cdll.LoadLibrary(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Project1.dll'))   # Xtherm.dll
print('Ok')
##if sys.platform == "win32" or "win64":
##    if is_64bits:
##        print('Loading Win64 binary')
##        XthermDll = cdll.LoadLibrary(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'XthermDll64.dll'))
##    else:
##        print('Loading Win32 binary')
##        XthermDll = cdll.LoadLibrary(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'XthermDll.dll'))
##elif sys.platform == "linux":
##    XthermDll = cdll.LoadLibrary(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'XthermDll.so'))
##else:
##    print('Unsupported Platform')  
#####################################################################################################################################
    
if not hasattr(QtGui.QImage, 'Format_Grayscale16'):
    force_use_8bits = True
    print('The version of PyQt5 ({}) used is too old (should > 5.13)\nQtGui.QImage does not support 16bit grayscale, forcing to use 8bit display'.format(QtCore.PYQT_VERSION_STR))
else:
    force_use_8bits = False


# Establish connection with serial port
try:
    serialport = serial.Serial(port= "COM3",baudrate = 9600, timeout =1)
    time.sleep(2)
    print("Connection to" + 'port' + "established successfully!\n")
except Exception as e:
    print(e)

## When this python code is run then initially send flag 'R' to start heating the camou-plate.
## when the command sent is received successfully then micro-controller will send flage '1'.
## and while loop terminates.
## Delay of 5 sec, after the delay the the gui will load while the camou-plate is still heating

start = 'R'
while True:
    serialport.write(start.encode('ascii'))
    val = serialport.read()
    val = val.decode()
    print(val)
    if val == '1':
        print('Heating Camou-plate for contour detection...')
        break
time.sleep(5)


# defaults
seq_cap_num = 12
SCuseCasl = 1
SCas8bits = 0
drawHiLo = 1
use_clahe = 0  # C
colormap = -1
saveFormat = ".png"
tempMode = 0  # T
rawMode = 0
use_NLMDenoising = 0  # D
use_equHist = 0  # H
use_sharpening = 0  # S
# OK for unnormalized kernels
sharpening_kernel = np.array([[-1, -1, -1], [-1, 15, -1], [-1, -1, -1]])
use_canny = 0  # N
cannylow = 22
cannyratio = 3
# bilateralFilter
use_bf = 0  # B
bf_sigmaColor = 0.07
use_deStrip = 0  # R
deStripk = (1, 1, 1)
deStripIter = 1
cover_cal_avg = 25
temp_max = 0


rplt_x = [0]
rplt_y = [0]

"""
    Initialization of variables & arrays used to approximate:
    1. Centroid of HEXAGON and draw a new small hexagon of small area
    2. save the coordinates of small hexagon
    3. approximate the average temperature of small hexagon
"""
fl = 0
flg = 2
cc = 0
pixel_temp = 0
temp_avg = 0
xx = [0]
zz = [0]
temps = [0]
filled_hex_coord = [0]
tmp_avg = 0
camou_temp = 0

ccc = 0
surr_temps = [0]
surr_xx = [0]
surr_zz = [0]
surr_coord = [0]
surr_temp_avg = 0

line1 = []
line = 0
ax = 0
xdata = []
ydata = []
fig = 0
ax_bgnd = 0

count = 1
cnt =0
clahe = cv2.createCLAHE(clipLimit=400, tileGridSize=(8, 8))
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
video_format = ".avi"
resizeAlgo = cv2.INTER_NEAREST

fg = 0
u = 0
cnt = 0

T = []
T_max= 0
T_min= 0
T_diff= 0
rating = 'Z'
q=0
qq = 0
set_point = 35
DS18b20 = 0.0

filename = 0
writer = 0

img2 = 0

##data = np.zeros((192,256), dtype = 'float')


def SearchForXThermT3s():
    # Try automatic searching for cameras with the correct resolution and frame rate
    width = 256
    height = 192
    frate = 25
    if (width, height, frate) == (256, 196, 25):  # 384, 292, 25
        print('Found XTherm T3s')
    
##    for camera_port in range(0, 254):
##        camera = cv2.VideoCapture(camera_port)  # ,cv2.CAP_DSHOW)
##        if camera.isOpened():
##            width = camera.get(3)  # CAP_PROP_FRAME_WIDTH
##            height = camera.get(4)  # CAP_PROP_FRAME_HEIGHT
##            frate = camera.get(5)  # CAP_PROP_FPS
##            print('n{:d}:w{:.0f}h{:.0f}f{:.2f}'.format(camera_port, width,
##                                                       height, frate))
##            if (width, height, frate) == (256, 196, 25):  # 384, 292, 25
##                print('Found XTherm T3s')
##                return camera
##            else:
##                camera.release()
##                cv2.destroyAllWindows()
    return None


class FrameGrabbingThread(QtCore.QThread):
    global tmp_avg,count

    running_signal = QtCore.pyqtSignal(int)
    global data
    def __init__(self, camera, calFunc, image_data_signal, parent=None):
        super(FrameGrabbingThread, self).__init__(parent)
        self.camera = camera
        self.calFunc = calFunc
        self.image_data_signal = image_data_signal
        self.running = True
        
        self.running_signal.connect(self.running_signal_slot)

##        print(data)

    def running_signal_slot(self, signal):
        self.running = True if signal == 1 else False
##        print(self.running)

    def run(self):
        
        while True:
            if self.running:
                # capture frame-by-frame
####                ret, data = self.camera.read()
                data = genfromtxt('TEMP-2022-09-08T16-30-24-86.csv', delimiter=',')  #

                ret = True
                # if frame is read correctly then [ret] is TRUE
                if ret:
                    data = data.reshape(192, 256) 
##                    data[0:192, :] = genfromtxt('TEMP-2022-09-08T16-30-24-86.csv', delimiter=',')
##                    data = data.view(np.uint16).reshape(196, 256) #292, 384
                    data[0:192, :] = self.calFunc(data[0:192, :]) #data[0:288, :]
                    self.image_data_signal.emit(data)

                    # print('grabbed: ', count,',',tmp_avg)



                else:
                    print('Frame grabbing failed')
                    exitNow()
            else:
                time.sleep(0.05)



class ThermalVideoStream(QtCore.QObject):
    image_data_signal = QtCore.pyqtSignal(np.ndarray)
    def __init__(self, camera_port=-1, parent=None):
        super().__init__(parent)
        self.camera_port = camera_port
        self.camera_init(self.camera_port)
        self.capture_thread = FrameGrabbingThread(self.camera, self.applyCal, self.image_data_signal)
        self.capture_thread.start()
        self.start_capture()
        
    def camera_init(self, camera_port=-1):
        if camera_port == -1:
            print('t')
            self.camera = SearchForXThermT3s()
        else:
            self.camera = cv2.VideoCapture(camera_port)  # ,cv2.CAP_DSHOW)

##        if not self.camera:
##            sys.exit("Xtherm camera not found")
        ######################
        self.calOffset = np.zeros((192, 256), dtype=np.int16) # (288, 384)
        self.calSlope = np.ones((192, 256), dtype=np.float32) # (288, 384)
        #######################
##        self.camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # CV_CAP_PROP_CONVERT_RGB
##        # CV_CAP_PROP_ZOOM use raw mode
##        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8004)
##        # CV_CAP_PROP_ZOOM calibrate
##        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8000)
##        # CV_CAP_PROP_ZOOM temperature mode?
##        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8020)
##        XthermDll.DataInit(256, 192) # 384, 288
        print("resolution set")
##        while True:
##            ret, rawFrame = self.camera.read()
##            if not ret:
##                print('Frame grabbing failed')
##                time.sleep(1)
##                continue
##            if not (rawFrame.size == 196 * 256 * 2):  # 292 * 384 * 2
##                print('ERROR: Incorrect frame size, wrong camera selected?')
##                exitNow()
##            else:
##                break

        #XthermDll.UpdateParam(tempMode, rawFrame.ctypes.data_as(POINTER(c_byte)))

    def start_capture(self):
        # print('start','1')
        self.capture_thread.running_signal.emit(1)

    def stop_capture(self):
        # print('stop','0')
        self.capture_thread.running_signal.emit(0)

    def selfCal(self):
        isCapturing = self.capture_thread.running
        if isCapturing:
            self.stop_capture()
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8000)  # internal shutter calibrate
        ret, rawFrame = self.camera.read()
        if not ret:
            print('Frame grabbing failed')
            exitNow()
        if not (rawFrame.size == 196 * 256 * 2): # 292 * 384 * 2
            print('ERROR: Incorrect frame size, wrong camera selected?')
        XthermDll.UpdateParam(tempMode, rawFrame.ctypes.data_as(POINTER(c_byte)))
        if isCapturing:
            self.start_capture()

    def set_0x21_mode(self, mode):
        global tempMode
        if mode != tempMode:
            self.camera.set(cv2.CAP_PROP_ZOOM, mode + 0x8020)
            tempMode = mode
            
    def set_0x02_mode(self, mode):
        global rawMode
        if mode:
            self.camera.set(cv2.CAP_PROP_ZOOM, 0x8002)
        else:
            self.camera.set(cv2.CAP_PROP_ZOOM, 0x8004)
        rawMode = mode
        
        
    def seq_cap(self):
        global seq_cap_num, saveFormat, SCas8bits, SCuseCal
        isCapturing = self.capture_thread.running
        if isCapturing:
            self.stop_capture()
        now = datetime.datetime.now()
        filename1 = 'SEQC-' + now.strftime( '%Y-%m-%dT%H-%M-%S') + \
                   ('-%02d' % (now.microsecond / 10000))
        os.mkdir(filename1)
        frames = []
        for n in range(0, seq_cap_num):
            ret, rawFrame = self.camera.read()
            if not ret:
                print('Frame grabbing failed')
                exitNow()
            frames.append(rawFrame)
        print('Capture complete')

        for n in range(0, seq_cap_num):
            rawFrame = frames[n]
            if SCuseCal:
                frame = self.applyCal(
                    rawFrame.view(np.uint16).reshape(196, 256)[0:192, :]) # reshape(292, 384)[0:288, :]
            else:
                frame = rawFrame.view(np.uint16).reshape(196, 256)[0:192, :] # reshape(292, 384)[0:288, :]
            if SCas8bits:
                frame = cv2.convertScaleAbs(
                    cv2.normalize(
                        frame,
                        dst=None,
                        alpha=0,
                        beta=255,
                        norm_type=cv2.NORM_MINMAX))
                cv2.imwrite(filename1 + '\\{:03d}'.format(n) + saveFormat, frame)
            else:
                cv2.imwrite(filename1 + '\\{:03d}'.format(n) + saveFormat, frame)
        print('Done')
        if isCapturing:
            self.start_capture()

    def self_cov_cal(self):
        print('Uniformity calibration start, expecting a uniform scene')
        isCapturing = self.capture_thread.running
        if isCapturing:
            self.stop_capture()
        frames = np.zeros((192, 256, cover_cal_avg)) # np.zeros((288, 384, cover_cal_avg))
        for n in range(cover_cal_avg):
            read, rawFrame = self.camera.read()
            frames[:,:,n] = rawFrame.view(np.uint16).reshape(196, 256)[0:192, :] # 292, 384)[0:288, :]
        print('Done')
        avgFrame = np.average(frames, axis=2)
        # avgFrame = np.multiply(avgFrame, self.calSlope)
        # self.calOffset = (np.average(avgFrame) - avgFrame).astype(np.float32)
        # self.calSlope = np.ones((288, 384), dtype=np.float32)
        self.calSlope = np.divide(np.average(avgFrame), avgFrame)
        if isCapturing:
            self.start_capture()

    def read_cal_constants(self, slopeCSV, offsetCSV):
        try:
            off_tmp = np.genfromtxt(
                os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), offsetCSV),
                delimiter=',',
                dtype=np.float32)
            if off_tmp.shape != (192, 256): # (288, 384)
                print('Wrong size of Offset')
                return
            self.calOffset = off_tmp
            slope_tmp = np.genfromtxt(
                os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), slopeCSV),
                delimiter=',',
                dtype=np.float32)
            if slope_tmp.shape != (192, 256): # (288, 384)
                print('Wrong size of Slope')
                return
            self.calSlope = slope_tmp
        except:
            print('Can not read file')

    def applyCal(self, img):
        img[0:192, :] = (np.multiply(img[0:192, :], self.calSlope) + self.calOffset).astype(np.uint16) # img[0:288, :]
        return img

    def calZeroize(self):
        self.calOffset = np.zeros((192, 256), dtype=np.int16) # (288, 384)
        self.calSlope = np.ones((192, 256), dtype=np.float32)




class ThermalDisplayWidget(QtWidgets.QWidget):
    progress_signal = QtCore.pyqtSignal(int)
    progress_signal1 = QtCore.pyqtSignal(int)
    plot_signal = QtCore.pyqtSignal(int, int, int)
    # plot_signal1 = QtCore.pyqtSignal(int)
    # resy=560 + 30, resx=432, infobarY=30
    def __init__(self, resy= 560+ 30, resx=432, infobarY=30, parent=None, rec_status_signal=None):
        # (self, resy=636 + 60, resx=768, infobarY=60, parent=None, rec_status_signal=None):

        super().__init__(parent)
        self.image = QtGui.QImage()
        self.video_writer = None
        self.video_frames = 0
        self.set_size(resy, resx, infobarY)
        self.rec_status_signal = rec_status_signal
        rawFrame = genfromtxt('TEMP-2022-09-08T16-30-24-86.csv', delimiter=',') # TEMP-2022-09-08T16-30-24-86
        print(rawFrame)
        self.image_data_slot(rawFrame)
        self.flip_x = False
        self.flip_y = True
        

    def set_size(self, resy=560 + 30, resx=432, infobarY=30):
        # self, resy=636 + 60, resx=768, infobarY=60):

        if self.video_writer is not None:
            print('Change Image Size During Recording is not Supported')
            return
        self.resx = resx
        self.resy = resy
        self.infobarY = infobarY
        self.setFixedSize(self.resx, self.resy)

    def toggle_size(self):

        if self.resx == 768:
            self.set_size(864 + 90, 1152, 90)
        elif self.resx == 1152:
            self.set_size(192 + 30, 256, 30) # (288 + 30, 384, 30)
        else:
            self.set_size()
            
    def flip_x_toggle(self):
        self.flip_x = not self.flip_x

    def flip_y_toggle(self):
        self.flip_y = not self.flip_y
#################################################333


    def update_plot_data(self):
        # F = plt.figure()
        # axs = fig.add_subplot(111)
        print('Plotting start/.')
        self.x = list(range(100))  # 100 time points
        self.y = [randint(0, 100) for _ in range(100)]
        # pen = pg.mkPen(color=(255, 0, 0))
        # self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)

        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.

        self.y = self.y[1:]  # Remove the first
        self.y.append(randint(0, 100))  # Add a new random value.
        # print('Y axis',self.y)
        ax1.clear()
        ax1.plot(self.x,self.y)
        # plt.show()
        # self.data_line.setData(self.x, self.y)  # Update the data.
        time.sleep(1)
#########################################################

    def thread(self,):
        print('Threading= ', count,tmp_avg)
        anim(count, tmp_avg)
        # t1 = Thread(target=self.Operation(1,34))
        # t1.start()
        # t1.join()

    def Operation(self,cnt,tmp):
        anim(count, tmp_avg)
        # print("time start")
        # time.sleep(1000)
        # print("time stop")


    def image_data_slot(self, rawFrame):
        global img2, filename, writer, DS18b20, set_point, camou_temp,qq ,q, rating, T_min, T_max, T_diff, T,cnt,fg,u,ax_bgnd, fig, xdata, ydata,ax,line,line1,count, ccc, surr_temps, surr_xx, surr_zz, surr_coord, surr_temp_avg, filled_hex_coord, xx, zz, temps, tmp_avg, pixel_temp ,fl, flg, cc, drawHiLo, use_clahe, clahe, colormap, use_NLMDenoising, use_deStrip, deStripk, deStripIter, use_equHist, use_bf, bf_sigmaColor, use_sharpening, sharpening_kernel
##        print('TT')
        self.FrameTimeStamp = datetime.datetime.now()
        self.bufferedRawFrame = rawFrame
        maxtmp = c_float()
        maxx = c_int()
        maxy = c_int()
        mintmp = c_float()
        minx = c_int()
        miny = c_int()
        centertmp = c_float()
        tmparr = np.zeros(3, dtype=np.float32)


        # XthermDll.GetTmpData(1, rawFrame.ctypes.data_as(POINTER(c_byte)),
        #                      byref(maxtmp), byref(maxx), byref(maxy),
        #                      byref(mintmp), byref(minx), byref(miny),
        #                      byref(centertmp),
        #                      tmparr.ctypes.data_as(POINTER(c_float)),
        #                      0)  # full frame temperature not necessary
        # there is not transformed temperature data in 400 deg range, use pixel values instead
        if tempMode:
            print("TEMP MODE encaled")
            maxtmp.value = rawFrame[192, 4] # [288, 4]
            mintmp.value = rawFrame[192, 7] # [288, 7]
            centertmp.value = (rawFrame[143, 191] + rawFrame[143, 192] +
                               rawFrame[144, 191] + rawFrame[144, 192]) / 4
        fpatmp = c_float()
        coretmp = c_float()
        fpaavg = c_int()
        orgavg = c_int()
        XthermDll.GetDevData(byref(fpatmp), byref(coretmp), byref(fpaavg), byref(orgavg))


        # print(rawFrame)
        ## begin the processing of the image
        outputFrame = rawFrame[0:192, :] # [0:192, :]
        outputFrame = np.transpose(outputFrame)

        # print('OUt Frame: ' , outputFrame)


        maxtmp = c_float()
        maxx = c_int()
        maxy = c_int()
        mintmp = c_float()
        minx = c_int()
        miny = c_int()
        centertmp = c_float()
        tmparr = np.zeros(3, dtype=np.float32)
        alltmp = np.zeros(256 * 192, dtype=np.float32) # 256 * 192
        # XthermDll.GetTmpData(0,
        #                      self.bufferedRawFrame.ctypes.data_as(
        #                          POINTER(c_byte)), byref(maxtmp), byref(maxx),
        #                      byref(maxy), byref(mintmp), byref(minx),
        #                      byref(miny), byref(centertmp),
        #                      tmparr.ctypes.data_as(POINTER(c_float)),
        #                      alltmp.ctypes.data_as(POINTER(c_float)))

##        alltmp = genfromtxt('TEMP-2022-09-08T16-30-24-86.csv', delimiter=',')
        alltmp = rawFrame
        
        '''
            Script to grab average temperature of Hexagon camou-pixel and its surrounding
        '''
        temp_data = alltmp.reshape(192,256)
        img = temp_data
        # img = cv2.resize(img, (self.resx, self.resy - self.infobarY), resizeAlgo)

##        print(img)
        #fl = 0
        #flg = 2
        #print(fl,flg)
        if fl == 0 and flg == 2:

##            print('f1',fl)
            h = self.matlab_style_gauss2D()      # create gaussian filter
            im = np.zeros((256,192))

            im = cv2.filter2D(img,-1,h)          # filter image pixel data 
            tp_min = 15
        #print(tp_min)
        ##tp_max = im.max()
            tp_max = 50
        #print(tp_max)
            Norm_image = (im - tp_min) * ((255-0) / (tp_max - tp_min)) + 0
            round_off = np.round_(Norm_image)

            round_off = np.uint8(round_off)
            Edgecanny = cv2.Canny(round_off, 200.0, 255.0)
            #print("Ec")
   
            val_gray = np.where(Edgecanny == 255)
        ##print('gray val')
        ##print(val_gray[0][:])

            Edgecanny[val_gray[0][:],val_gray[1][:]] = 1
            Edgecanny[:,0:110] = 0

            detect_hex = ndimage.binary_fill_holes(Edgecanny).astype(int)
        # Grabbing coordinates of the filled hexagon 
            fill_detect_hex = np.where(detect_hex == 1)

        # modify the data type
        # setting to 32-bit floating point
            detect_hex = np.float32(detect_hex)
            dest = cv2.cornerHarris(detect_hex, 2, 3, 0.052)  # 2,5,0.07   # 2,3,0.055

        # Results are marked through the dilated corners
            dest = cv2.dilate(dest, None)
        
            ret, dst = cv2.threshold(dest,0.001*dest.max(),255,0)
            dst = np.uint8(dst)

        # find centroids
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

        # define the criteria to stop and refine the corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            corners = cv2.cornerSubPix(detect_hex, np.float32(centroids), (4,6), (-1,-1), criteria)

        # Now draw them
        #res = np.hstack((centroids,corners))
            centroids = np.int0(centroids)
            corners = np.int0(corners)

            IM = np.zeros((192,256,3))
            IM[:,:,0] = Edgecanny
            IM[centroids[:,1],centroids[:,0]]=[0,255,0]   # Harris corner detction
            IM[corners[:,1],corners[:,0]] = [0,0,255]    # refined corners {cv2.cornerSubPix}

        ## sorting detected corner coordinates (x,y) in counter-closkwise direction
            r,c = len(corners),len(corners[0])
            #print(r,c)
        #print(corners)

            if r > 6:
##                print(r)
                flag = 1
                y = 1
                corner_coord = []
                while y < r:
                    x = 0
                    corner_coord.append((corners[y][x], corners[y][x+1]))
                    y += 1

            #print(corners)
        #print(corner_coord)
        #print(len(corner_coord),len(corner_coord[0]))
        
##                origin = []
##                origin.append(corners[y-1][x])
##                origin.append(corners[y-1][x+1])
##
##                refvec = [0,1]

            #print(corner_coord)


            #corner_sort = sorted(corner_coord, key = self.clockwiseangle_and_distance)    # self.clockwiseangle_and_distance
                center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), corner_coord), [len(corner_coord)] * 2))
            
                ss = sorted(corner_coord, key=lambda coord: (-135 - math.degrees(math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
            #print(ss)
                x_center = int(center[0])
                y_center = int(center[1])
                y_center = y_center-2
            #print('center',x_center,y_center)
            #print('central coordinates', x_center, y_center)
            #print('ss',ss)
                corner_sort = ss
##                print(corner_sort)
##                print(x_center,y_center)

            
            #print('corner sort')
        #corner_sort = corner_coord
        
        ## Removing unwanted corner detected coordinates and storing only the six 6
        ## corner coordinates of heaxagon into array
                r,c = len(corner_sort),len(corner_sort[0])
        ##print(r,c)
                j = 0
                corner_points = []
        ##print("Distance measurement")

##                for i in range(0,r-2):
##                    dist = ((corner_sort[i][j] - corner_sort[i+1][j])**2 + (corner_sort[i][j+1] - corner_sort[i+1][j+1])**2)**0.5
##                    if i == 0:
##                        corner_points.append([corner_sort[i][j], corner_sort[i][j+1]])
##                    if dist > 13:
##                        corner_points.append([corner_sort[i+1][j], corner_sort[i+1][j+1]])
##                ##print('i=',i,'i+1=',i+1)
##                ##print(corner_sort[i][j],corner_sort[i+1][j],corner_sort[i][j+1],corner_sort[i+1][j+1])
##                ##print('Distance',dist)
##                ##print(" ")
##          
##                    elif dist <= 13:
##                ##print('i=',i,'i+2=',i+2)
##                ##print(corner_sort[i][j],corner_sort[i+2][j],corner_sort[i][j+1],corner_sort[i+2][j+1])
##                        dist = ((corner_sort[i][j] - corner_sort[i+2][j])**2 + (corner_sort[i][j+1] - corner_sort[i+2][j+1])**2)**0.5
##                ##print('neglect dist',dist)
##                ##print(" ")
##                        if dist < 20:
##                            corner_points.append([corner_sort[i+2][j], corner_sort[i+2][j+1]])

##
##                corner_points = corner_sort
##        ## calculating center points of hexagon from each opposite corner points  
##                x_left_top, y_left_top = corner_points[0][0], corner_points[0][1]
##                x_right_bottom, y_right_bottom = corner_points[4][0], corner_points[4][1]
##                x_mid , y_mid = int((x_left_top + x_right_bottom)/2) , int((y_left_top + y_right_bottom)/2) 
##            #print(x_left_top, y_left_top , x_right_bottom, y_right_bottom, x_mid, y_mid )
##        #IM[y_mid, x_mid] = 1
##                cv2.line(IM, (x_left_top+1, y_left_top+1), (x_right_bottom-1, y_right_bottom-1), (0, 255, 255), thickness=1)
##
##
##                x_right_top, y_right_top = corner_points[1][0], corner_points[1][1]
##                x_left_bottom, y_left_bottom = corner_points[5][0], corner_points[5][1]
##                xx_mid , yy_mid = int((x_right_top + x_left_bottom)/2) , int((y_right_top + y_left_bottom)/2) 
##        ##print(x_right_top, y_right_top , x_left_bottom, y_left_bottom, xx_mid , yy_mid)
##        #IM[yy_mid, xx_mid] = 1
##                cv2.line(IM, (x_right_top-1, y_right_top+1), (x_left_bottom+1, y_left_bottom-1), (0, 255, 255), thickness=1)
##
####                x_top, y_top = corner_points[0][0], corner_points[0][1]
####                x_bottom, y_bottom = corner_points[3][0], corner_points[3][1]
####        ##x_bottom1, y_bottom1 = corners[6][0], corners[6][1]
####        ##x_bottom2, y_bottom2 = corners[7][0], corners[7][1]
####        ##x_bottom, y_bottom = int((x_bottom1 + x_bottom2)/2), int((y_bottom1 + y_bottom2)/2) 
####                xxx_mid , yyy_mid = int((x_top + x_bottom)/2) , int((y_top + y_bottom)/2) 
####        ##print(x_top, y_top , x_bottom, y_bottom, xxx_mid , yyy_mid)
####        #IM[yyy_mid, xxx_mid] = 1
##
##
##                if r == 8:
##                    x_top, y_top = corner_points[7][0], corner_points[7][1] # 5
##                elif r == 9: # 7
##                    x_top, y_top = corner_points[6][0], corner_points[6][1] 
##                else:
##                     print('greater corner detected need consideration')
##                x1 , y1 = corner_points[2][0], corner_points[2][1]
##                x2 , y2 = corner_points[3][0], corner_points[3][1]
##                x_bottom, y_bottom = int((x1 + x2)/2), int((y1 + y2)/2) 
##                xxx_mid , yyy_mid = int((x_top + x_bottom)/2) , int((y_top + y_bottom)/2) 
##                #print(x_top, y_top , x_bottom, y_bottom, xxx_mid , yyy_mid)
##
##
##                cv2.line(IM, (x_top, y_top-1), (x_bottom, y_bottom+1), (0, 255, 255), thickness=1)
##
##
##        ## calculating the centeroid if hexagon 
##                x_centroid = int((x_mid + xx_mid + xxx_mid)/3)
##                y_centroid = int((y_mid + yy_mid + yyy_mid)/3)
                IM[y_center, x_center,:] = [1,1,1]

                coords = []
                rotate = 92
                sides = 6
                theta = math.radians(rotate)
                n = sides + 1
                x0 = x_center   #x_centroid+1
                y0 = y_center   #y_centroid+1
                r = 11
                for s in range(n):
                    t = 2.0 * math.pi * s / sides + theta
                    coords.append([ int(r * math.cos(t) + x0), int(r * math.sin(t) + y0)])
                #print(coords)

                hex_vertices = np.array(coords)
        ##print(hex_vertices)
                hex_vertices = hex_vertices.reshape(-1,1,2)

        ##print(hex_vertices)
                cv2.polylines(IM, [hex_vertices], True, (255,255,0), thickness = 1)

                r,c = len(IM),len(IM[0])
##                print(r,c)
##                print('r')
                filled_new_hex = np.zeros((r,c))
                cv2.fillPoly(filled_new_hex, pts = [hex_vertices], color = 1)


                filled_hex_coord = np.where(filled_new_hex == 1.0)
                # print("filled hex coord",filled_hex_coord)
                rr , cc = len(filled_hex_coord),len(filled_hex_coord[0])
                # print(rr,cc)
                #temps = np.empty(cc, dtype=int)
                temps = [0] * cc
                #xx = np.empty(cc, dtype=int)
                xx = [0] * cc
                #zz = np.empty(cc, dtype=int)
                zz = [0] * cc
                # print('cc',cc)
##            
                for i in range(0,cc):
##                    print('cc',cc)
##                    print('loop',i)
                    
                    xx[i]= filled_hex_coord[0][i]
                    hei = int(xx[i])
##                    print(hei)
                    zz[i] = filled_hex_coord[1][i]
                    wid = int(zz[i])
                    #print(zz)
                    temps[i] = img[hei,wid]
                    #print('T',temps[i])

                tmp_max = max(temps)
                tmp_avg = sum(temps)/len(temps)
                # print('T',tmp_avg)
                    
                # surrounding temperature detection

                surr_area = filled_new_hex
                # fill surrounding region with value 1
                surr_area[25:150,100:200] = 1    # surr_area[60:120,115:187] = 1
                # removing the new hexagon coordinate from surrounding coordinates
                surr_area[filled_hex_coord[0][:],filled_hex_coord[1][:]] = 0
                                
                surr_coord = np.where(surr_area == 1)
                rrr , ccc = len(surr_coord),len(surr_coord[0])

                surr_temps = [0] * ccc
                surr_xx = [0] * ccc
                surr_zz = [0] * ccc
##                print('cc',ccc)
                
##                print('Extract surround coords')
##                print('cc',cc)
##                print('ccc',ccc)
                #breakpoint()
                img2 = genfromtxt('2.csv', delimiter=',')
                img2 = cv2.flip(img2, 0)
                # img[25:150, 100:200] = img2[25:150, 100:200]

                for i in range(0,ccc):
##                    print('cc',ccc)
##                    print('loop',i)
                    
                    surr_xx[i]= surr_coord[0][i]
                    surr_hei = int(surr_xx[i])
##                    print(hei)
                    surr_zz[i] = surr_coord[1][i]
                    surr_wid = int(surr_zz[i])
                    #print(zz)
                    img[surr_hei, surr_wid] = img2[surr_hei-180, surr_wid]
                    surr_temps[i] = img[surr_hei,surr_wid]

                surr_tmp_max = max(surr_temps)
                surr_temp_avg = sum(surr_temps)/len(surr_temps)
##                print(surr_temp_avg)
                filename = '6th.csv'
                with open(filename, 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["S.no", "Time", "Surrounding Temp", "camou_pixel_Temp", "DS18b20"])


                if tmp_avg != 0:
##                    print(tmp_avg)
                    pixel_temp = 1.0
##                    print(pixel_temp)
##                    print(cc)
                    fl= 1
                    flg = 3

##                    print('sending command : ',pixel_temp)
##                    serialport.write(bytes(pixel_temp, 'utf-8'))
##                    print('command sent')
                    
##                    print(xx)
                else:
                    pixel_temp = 0.0
                    fl = 0
                    flg = 2

##                px_x = xx
##                print(px_x)
##                px_y = zz

            else:
                pixel_temp = 0.0



##            print(fl,flg)
        elif fl ==1 and flg == 3:
            #pixel_temp =0
    
            #breakpoint()
            #print(px_x)
            
            # print('in loop-')
##            print(pixel_temp)
##            print(xx)
            # img2 = genfromtxt('3.csv', delimiter=',')
            for i in range(0,cc):

                xx[i]= filled_hex_coord[0][i]
                hei = int(xx[i])
##                print(hei)
                zz[i] = filled_hex_coord[1][i]
                wid = int(zz[i])
##                print(zz)

                temps[i] = img[hei,wid]
##                print('T',temps[i])

            tmp_max = max(temps)
            tmp_avg = sum(temps)/len(temps)


##            print('camou-plate temperature',tmp_avg)

            for ii in range(0,ccc):
                #print('in looping')
               ## print('cc',ccc)
               ## print('loop',i)

                surr_xx[ii]= surr_coord[0][ii]
                surr_hei = int(surr_xx[ii])
                #print(surr_hei)
                surr_zz[ii] = surr_coord[1][ii]
                surr_wid = int(surr_zz[ii])
                #print(surr_wid)
                # img[surr_hei, surr_wid] = 50
                img[surr_hei, surr_wid] = img2[surr_hei-180, surr_wid] #set_point
                surr_temps[ii] = img[surr_hei,surr_wid]

                #print(surr_temps[ii])

            # outputFrame = rawFrame
            # img = np.transpose(img)
            # img1 = cv2.resize(img, (self.resx, self.resy - self.infobarY), resizeAlgo)
            # outputFrame = img
            # outputFrame = outputFrame(surr_coord[0][:],surr_coord[1][:])
            # outputFrame = outputFrame.reshape(192,256)
            # print("Image frame = ", len(img), len(img[0])
            # print("rawFrame = ", len(rawFrame), len(rawFrame[0]))

            # img = np.transpose(img)
            # r = cv2.resize(img, (432,560), resizeAlgo)
            # outputFrame = r
            # print("r = ", len(r),len(r[0]))
            # print('OUTPUTFRAME = ', len(outputFrame), len(outputFrame[0]))

            surr_tmp_max = max(surr_temps)
            surr_temp_avg = sum(surr_temps)/len(surr_temps)
##            print('surrounding temperature',surr_temp_avg)

##########################################################################################################
            camou_temp = "{:.2f}".format(tmp_avg)
            can = camou_temp
            T.append(can)

            if (count % 50 == 0):
                # print('multiple of 50')
                # print(T)
                T_min = float(min(T))
                T_max = float(max(T))

                if q <= 0:
                    qq = 1
                if q >= 1:
                    qq = 2
                if qq == 1:
                    q = q+0.05
                if qq == 2:
                    q = q-0.05
                # T_diff = (T_max - T_min)
                T_diff = q
                # print('MIN val', min(T))
                # print('MAX val', max(T))
                # print('temperature difference = ', T_diff)
                # print('dT = ', dT)
                T = []

                if T_diff <= 0.1 and T_diff > 0:
                    # print('Rating : ', rating)
                    rating = 'A'
                if T_diff <= 0.5 and T_diff > 0.1:
                    # print('Rating : ', rating)
                    rating = 'B'
                if T_diff <= 1 and T_diff > 0.5:
                    # print('Rating : ', rating)
                    rating = 'C'
                if T_diff > 1:
                    # print('Rating : ', rating)
                    rating = 'D'

                u = int(100 - ((T_diff / 1) * 100))

            #  code for trial of PROGRESS BAR is running well in real-time or not

            # if count > 0 and count < 100:
            #     q = q + 1
            #     T_diff = q
            #
            # if count > 100 and count < 200:
            #     q = q-1
            #     T_diff = q
            #
            # if count > 200 and count < 300:
            #     q = q+1
            #     T_diff = q
            #
            # if count > 300 and count < 400:
            #     q = q - 1
            #     T_diff = q
            #
            # if count > 400 and count < 500:
            #     q = q + 1
            #     T_diff = q
            #
            # if count > 500 and count < 600:
            #     q = q - 1
            #     T_diff = q
            #
            # if count > 600 and count < 700:
            #     q = q + 1
            #     T_diff = q
            #
            # if count > 700 and count < 800:
            #     q = q - 1
            #     T_diff = q
            #
            # if count > 800 and count < 900:
            #     q = q + 1
            #     T_diff = q
            #
            # if count > 900 and count < 1000:
            #     q = q - 1
            #     T_diff = q

                self.progress_signal.emit(u)

            self.progress_signal1.emit(count)

            # tmp_avg = tmp_avg +count
            self.plot_signal.emit(tmp_avg, surr_temp_avg, count)
            # self.plot_signal1.emit(count)
######            print('Average temperature of camou-plate',camou_temp)
##            camou = Decimal(cam_plate)
##            camou_temp = int(camou.shift(2))
##            camou_T = str(camou_temp) + "\n"
##            print('decimal val ', camou_temp)    
##            val_camou = 0

            bg_temp = "{:.2f}".format(surr_temp_avg)
            # bg_temp = 22.34

######            print('Average temperature of surrounding',bg_temp)
##            sur = Decimal(s)
##            surround_temp = int(sur.shift(2))
##            surr_T = str(surround_temp) + "\n"
##            surr_T = str(surround_temp) + 'p' + str(camou_temp) + "\n"
##            print('decimal val ', surr_T)            
##            val_surround = 0

            B = str(bg_temp) + 'p' + str(camou_temp) + 'e'
####            print('Ready to send data')


            # print('converted surround temperature',B)
##            while True:
##                print(B)
            count += 1
            print(count)


            # B = 'e'
            serialport.write(B.encode('ascii'))
            serialport.flush()
            # time.sleep(0.001)

            """
                Code for verification of serial data transmitted by Python is received
                successfully by micro-controller.

                But with these lines added to the script the GUI display hangs-up
                because of the [readline function of serial port], so i have commented
                these lines
            """
            grab_data = 'g'
            serialport.write(grab_data.encode('ascii'))

            # serial_data = serialport.inWaiting()
            # print("waiting data: ", serial_data)
            val = serialport.read(serialport.inWaiting())     # read line of temperatures that is send by python
            serialport.flush()
            val = val.decode()
            val = val.rstrip()    # for removing CARRAIGE RETURN ['\r'] and LINE FEED ['\n'] from received data
            print("value received: ", val + " Successfully+++++++>>>>>>>>>")
            # DS18b20 = float(val)
            if len(val) == 0 or len(val) >= 8:
                # print("incorrect data: ", val)
                # print(val[1])
                print("value length: ", len(val))

                # DS18b20 = DS18b20
            elif len(val) == 5:
                DS18b20 = float(val)
                # print("L= " , len(val))
            # print("DATA TYPE: ", type(DS18b20))
            # time.sleep(0.0001)

            with open(filename, 'a', newline='') as file:
                writer = csv.writer(file)
                # writer.writerow(["camou_temp", "camou_temp", "DS18b20"])
                # mydate = datetime.datetime.now()
                # csvstr = datetime.datetime.strftime(mydate, '%Y, %m, %d, %H, %M, %S')
                writer.writerow([count, datetime.datetime.strftime(datetime.datetime.now(), '%H: %M: %S'), bg_temp, camou_temp, DS18b20])
            file.close()

            """
                End of Serial data receiving code
            """

################################################################################################################


        '''
        End of script for grabbing average temperatire of camou-pixel and surrounding
##      '''
        # img[60:120, 115:187] = 50
        # img[filled_hex_coord[0][:], filled_hex_coord[1][:]] = 0
        # resizedImg = cv2.resize(img, (self.resx, self.resy - self.infobarY), resizeAlgo)
        # print('R: ', resizedImg)
        # print(len(resizedImg), len(resizedImg[0]))
        # np.savetxt(
        #     "TEMP-" +
        #     self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') +
        #     ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) + ".csv",
        #     resizedImg,
        #     delimiter=",",
        #     fmt='%.2f')  # 192, 256
        # display image on GUI
        outputFrame = cv2.normalize(
            img,
            dst=None,
            alpha=0,
            beta=65535,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_16U)



        # cv2.imshow('image', resizedImg)
        # outputFrame = np.transpose(outputFrame)
        # print('OUTPUTFRAME = ', len(outputFrame), len(outputFrame[0]))

##        print('OUTPUTFRAME')
##        print(outputFrame)
##        self.update()
##        time.sleep(0.001)
        
        if use_deStrip:
            outputFrame = deStripping(outputFrame, k=deStripk, iter=deStripIter)
            
        if use_sharpening:
            outputFrame = cv2.filter2D(outputFrame, cv2.CV_32F, sharpening_kernel)
            outputFrame = cv2.normalize(
                            outputFrame,
                            dst=None,
                            alpha=0,
                            beta=65535,
                            norm_type=cv2.NORM_MINMAX,
                            dtype=cv2.CV_16U)

        if use_equHist:
            outputFrame = equHist16(outputFrame)

        if use_bf:
            outputFrame = (cv2.bilateralFilter(
                ((outputFrame.astype(np.float32)) / 65535), 0, bf_sigmaColor, 3
            ) * 65535).astype(
                np.uint16
            )  # change 5 to higher value for smother image (but significantly slower)

        if use_clahe:
            outputFrame = clahe.apply(outputFrame)
        
        if use_canny and ~use_bf:
            outputFrame = cv2.add(
                cv2.Canny(
                    cv2.GaussianBlur(
                        (outputFrame / 257).astype('uint8'),
                        (3, 3), 0), cannylow, cannylow * cannyratio) * 16384,
                outputFrame)
        if use_canny and use_bf:
            outputFrame = cv2.add(
                cv2.Canny((outputFrame / 257).astype('uint8'), cannylow,
                          cannylow * cannyratio) * 16384, outputFrame)

        if drawHiLo:
            cv2.rectangle(outputFrame, (maxx.value - 3, maxy.value - 3),
                          (maxx.value + 3, maxy.value + 3), 0)
            cv2.rectangle(outputFrame, (minx.value - 3, miny.value - 3),
                          (minx.value + 3, miny.value + 3), 65535)

        self.flip_x = False
        self.flip_y = True
        if self.flip_x and self.flip_y:
            outputFrame = cv2.flip(outputFrame, -1)
        elif self.flip_x:
            outputFrame = cv2.flip(outputFrame, 1)
        elif self.flip_y:
            outputFrame = cv2.flip(outputFrame, 0)

        # outputFrame = cv2.flip(outputFrame, -1)
        outputFrame = np.transpose(outputFrame)
        outputFrame = cv2.flip(outputFrame, 0)

        resizedImg = cv2.resize(outputFrame, (self.resx, self.resy - self.infobarY), resizeAlgo)
        use_8bits = force_use_8bits or (colormap != -1) or (self.video_writer is not None)
        # print('resizedImg = ', len(resizedImg), len(resizedImg[0]))

        if use_8bits:
            resizedImg8b = (resizedImg / 257).astype('uint8')
            if colormap == -1:
                self.bufferedImg = np.zeros(
                    (self.resy, self.resx), dtype=np.uint8)
                cv2.putText(
                    self.bufferedImg,
                    'FPAT:{:.2f} CORT:{:.2f} FPAA:{:5d} ORGA:{:5d}'
                    .format(fpatmp.value, coretmp.value, fpaavg.value, orgavg.value) + 
                    ('C' if use_clahe else '') + ('X' if self.flip_x else '') + ('Y' if self.flip_y else '') +
                    ('H' if use_equHist else '') + ('N' if use_canny else '') +
                    ('T' if (tempMode or rawMode) else '') + ('B' if use_bf else '') +
                    ('S' if use_sharpening else '') + ('R' if use_deStrip else ''),
                    (4, self.resy - self.infobarY +
                     round(8 * self.infobarY / 30)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35 * self.infobarY / 30, 255, 1)
                cv2.putText(
                    self.bufferedImg,
                    'MAXT:{:.2f} {:3d},{:3d} MINT:{:.2f} {:3d},{:3d} CNTT:{:.2f}'
                    .format(maxtmp.value, maxx.value, maxy.value, mintmp.value,
                            minx.value, miny.value, centertmp.value),
                    (4,
                     round(self.resy - self.infobarY + 21 / 30 * self.infobarY)
                     ), cv2.FONT_HERSHEY_SIMPLEX, 0.35 * self.infobarY / 30,
                    255, 1)
                self.bufferedImg[0:(
                    self.resy - self.infobarY), :] = resizedImg8b
                self.image = self.get_qimage(self.bufferedImg, self.resy,
                                             self.resx,
                                             QtGui.QImage.Format_Grayscale8, 1)
            else:
                self.bufferedImg = np.zeros(
                    (self.resy, self.resx, 3), dtype=np.uint8)
                cv2.putText(
                    self.bufferedImg,
                    'FPAT:{:.2f} CORT:{:.2f} FPAA:{:5d} ORGA:{:5d} CPLT:{:2d}'.
                    format(fpatmp.value, coretmp.value, fpaavg.value,
                           orgavg.value, colormap) +
                    ('C' if use_clahe else '') + ('H' if use_equHist else '') +
                    ('X' if self.flip_x else '') + ('Y' if self.flip_y else '') +
                    ('N' if use_canny else '') + ('T' if tempMode else '') +
                    ('B' if use_bf else '') + ('R' if use_deStrip else ''),
                    (4, self.resy - self.infobarY +
                     round(8 * self.infobarY / 30)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35 * self.infobarY / 30, (255, 255, 255), 1)
                cv2.putText(
                    self.bufferedImg,
                    'MAXT:{:.2f} {:3d},{:3d} MINT:{:.2f} {:3d},{:3d} CNTT:{:.2f}'.
                    format(maxtmp.value, maxx.value, maxy.value, mintmp.value,
                           minx.value, miny.value, centertmp.value),
                    (4,
                     round(self.resy - self.infobarY + 21 / 30 * self.infobarY)
                     ), cv2.FONT_HERSHEY_SIMPLEX, 0.35 * self.infobarY / 30,
                    (255, 255, 255), 1)
                self.bufferedImg[0:(
                    self.resy - self.infobarY), :, :] = cv2.applyColorMap(
                        resizedImg8b[0:(self.resy - self.infobarY), :], colormap)
                self.image = self.get_qimage(cv2.cvtColor(self.bufferedImg, cv2.COLOR_BGR2RGB), self.resy,
                                             self.resx,
                                             QtGui.QImage.Format_RGB888, 3)
        else:
            #print('background')
            #print(bg_temp_avg)
            if pixel_temp == 0:
                tmp_avg = 0.00
                surr_temp_avg = 0.00
                u = 0
##                #print('0')
            else:
                u = u + 1
                if u == 99:
                    u = 0
                tmp_avg = tmp_avg + u
                surr_temp_avg = surr_temp_avg


                
            self.bufferedImg = np.zeros(
                (self.resy, self.resx), dtype=np.uint16)
            cv2.putText(
                self.bufferedImg,
                'FPAT:{:.2f} CORT:{:.2f} FPAA:{:5d} ORGA:{:5d} Stmp:{:.2f}'.format(
                   fpatmp.value, coretmp.value, fpaavg.value, orgavg.value, surr_temp_avg),
                # +(' 16bits(Disp)'
                #  if use_NLMDenoising else ' 16bits') + ('C' if use_clahe else '') +
                # ('X' if self.flip_x else '') + ('Y' if self.flip_y else '') +
                # ('H' if use_equHist else '') + ('N' if use_canny else '') +
                # ('T' if (tempMode or rawMode) else '') + ('B' if use_bf else '') +
                # ('S' if use_sharpening else '') + ('R' if use_deStrip else ''),
                (4, self.resy - self.infobarY + round(8 * self.infobarY / 30)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35 * self.infobarY / 30, 65535, 1)
            cv2.putText(
                self.bufferedImg,
                'MAXT:{:.2f} {:3d},{:3d} T_diff:{:.2f} Rate:{:s} Ptmp:{:.2f}'
                .format(maxtmp.value, maxx.value, maxy.value, T_diff, rating
                        ,tmp_avg),
                (4, round(self.resy - self.infobarY + 21 / 30 * self.infobarY)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35 * self.infobarY / 30, 65535, 1)
            self.bufferedImg[0:(self.resy - self.infobarY), :] = resizedImg
            self.image = self.get_qimage(self.bufferedImg, self.resy, self.resx,
                                         QtGui.QImage.Format_Grayscale16, 2)
        if self.video_writer is not None:
            self.video_writer.write(self.bufferedImg)
            self.rec_status_signal.emit('Recording Frm {}'.format(self.video_frames))
            self.video_frames += 1
        self.update()
    """
        Functions used by average manipulating script for camou-pixel and surrounding
    """

    # """
    # 2D gaussian mask - should give the same result as MATLAB's
    # fspecial('gaussian',[shape],[sigma])
    # """
    def matlab_style_gauss2D(self, shape=(50,50), sigma=1):
        m,n = [(ss-1.)/2. for ss in shape]
        y,x = np.ogrid[-m:m+1,-n:n+1]
        h = np.exp( -(x*x + y*y) / (2.*sigma*sigma) )
        h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
        sumh = h.sum()
        if sumh != 0:
            h /= sumh
        return h

    def clockwiseangle_and_distance(self, point):
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


    def counter_clockwise_order(self, a):
        print(a)
        b = a.min()
        print(b)
        d = pdist(b).min()
        c = np.round(2*b/d).astype(int)

        img = np.zeros(c.max(0)[::-1]+1, dtype=np.uint8)

        d1,d2 = cKDTree(c).query(c,k=3)
        b = c[d2]
        p1,p2,p3 = b[:,0],b[:,1],b[:,2]
        for i in range(len(b)):
            cv2.line(img,tuple(p1[i]),tuple(p2[i]),255,1)
            cv2.line(img,tuple(p1[i]),tuple(p3[i]),255,1)

        img = (binary_fill_holes(img==255)*255).astype(np.uint8)   
        if int(cv2.__version__.split('.')[0])>=3:
            _,contours,hierarchy = cv2.findContours(img.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        else:
            contours,hierarchy = cv2.findContours(img.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        cont = contours[0][:,0]        
        f1,f2 = cKDTree(cont).query(c,k=1)
        ordered_points = a[f2.argsort()[::-1]]

        if DEBUG_PLOT==1:
            NPOINTS = len(ordered_points)
                #for i in range(NPOINTS):
                #plt.plot(ordered_points[i:i+2,0],ordered_points[i:i+2,1],alpha=float(i)/(NPOINTS-1),color='k')
                #plt.show()
        return ordered_points


    '''
        functions end for average code
    '''
    
    def dump_rawdata(self):
        np.savetxt(
            "RAWD-" +
            self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') +
            ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) + ".csv",
            self.bufferedRawFrame,
            delimiter=",",
            fmt='%5d')
        print('Done')

    def save_temperature(self):
        print("saving data")
        print(self)
        maxtmp = c_float()
        maxx = c_int()
        maxy = c_int()
        mintmp = c_float()
        minx = c_int()
        miny = c_int()
        centertmp = c_float()
        tmparr = np.zeros(3, dtype=np.float32)
        alltmp = np.zeros(256 * 192, dtype=np.float32) # 256 * 192
        XthermDll.GetTmpData(0,
                             self.bufferedRawFrame.ctypes.data_as(
                                 POINTER(c_byte)), byref(maxtmp), byref(maxx),
                             byref(maxy), byref(mintmp), byref(minx),
                             byref(miny), byref(centertmp),
                             tmparr.ctypes.data_as(POINTER(c_float)),
                             alltmp.ctypes.data_as(POINTER(c_float)))
        print(
            'TempData:maxtmp:{:.2f},maxx:{:d},maxy:{:d},mintmp:{:.2f},minx:{:d},miny:{:d},center:{:.2f}'.
            format(maxtmp.value, maxx.value, maxy.value, mintmp.value,
                   minx.value, miny.value, centertmp.value))
        np.savetxt(
            "TEMP-" +
            self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') +
            ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) + ".csv",
            alltmp.reshape(192, 256), 
            delimiter=",",
            fmt='%.2f')  # 192, 256
        print('Done')

    def dump_rawdata_png(self):
        global saveFormat
        cv2.imwrite(
            "RAWD-" +
            self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') +
            ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) +
            saveFormat, self.bufferedRawFrame[0:192, :]) # [0:192, :]
        print('Done')
       
    def toggle_recording(self):
        global fourcc, video_format, colormap
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            self.rec_status_signal.emit("Saved Frm {}".format(self.video_frames))
            self.video_frames = 0
        else:
            filename = "RECO-" + self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') + \
                       ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) + video_format
            print('Saving video to ' + filename)
            self.video_writer = cv2.VideoWriter(filename, fourcc, 25, (self.resx, self.resy), isColor=(colormap!=-1))
            self.rec_status_signal.emit("Recording")

    def get_qimage(self, image: np.ndarray, height, width, type,
                   bytesPerLineMul):
        bytesPerLine = bytesPerLineMul * width
        image = QtGui.QImage(image.data, width, height, bytesPerLine, type)
        return image

    def sav_img(self):
        global saveFormat
        if use_NLMDenoising:
        # Due to the slowness of this filter, it's only applied when saving images
            self.bufferedImg = cv2.fastNlMeansDenoising(
                (self.bufferedImg / 257).astype('uint8'), None, 22, 5, 21)
            cv2.putText(
                self.bufferedImg, 'D',
                (self.resx - round(20. * self.infobarY / 30),
                 self.resy - self.infobarY + round(8 * self.infobarY / 30)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35 * self.infobarY / 30, 255, 1)
        cv2.imwrite(
            "DISP-" + self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') +
            ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) + saveFormat,
            self.bufferedImg)
        print('Done')

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.drawImage(0, 0, self.image)
        self.image = QtGui.QImage()


class Temp_plot(QtWidgets.QWidget):
 
    def __init__(self):
        super().__init__()
        self.left = 50
        self.top = 10
        self.title = 'Temparature plot'
        self.width = 512
        self.height = 384
        self.initUI() #not sure what this line of code does
        self.fig=Figure()
        self.canvas=FigureCanvas(self.fig)
        #add plot toolbar from matplotlib
        self.toolbar = NavigationToolbar(self.canvas, self)
 
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
 
        m = PlotCanvas(self, width=7, height=6)
        m.move(0,30)
 
##        button = QPushButton('PyQt5 buttons', self)
##        button.setToolTip('This is an example button')
##        button.move(500,0)
##        button.resize(140,100)
        # print('ploted')
##        self.show()

class PlotCanvas(FigureCanvas):
    global tmp_avg,cnt 
    def __init__(self, parent=None, width=4, height=3, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
##        self.axes = fig.add_subplot(111)
 
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        self.hour = list(range(100))
        self.temperature = [randint(0,100) for _ in range(100)] 
 
##        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding,QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        self.plot()


##        self.timer = QtCore.QTimer()
##        self.timer.setInterval(1)
##        self.timer.timeout.connect(self.plot)
##        self.timer.start()
##        
## 
    def plot(self):
##        data = [random.random() for i in range(25)]
##        
##        self.hour = self.hour[1:]  # Remove the first y element.
##        self.hour.append(self.hour[-1] + 1)  # Add a new value 1 higher than the last.
######
##        self.temperature = self.temperature[1:]  # Remove the first 
##        self.temperature.append(randint(0,100))
        rplt_x.append(rplt_x[-1] + 1)
        plate_temp = "{:.2f}".format(tmp_avg)
        rplt_y.append(plate_temp)

##        self.data_line.setData(rplt_x,rplt_y)

##        print('T',plate_temp)
        # print(rplt_y)
        # l = len(rplt_y)
        # print(rplt_y[l-1])

        ax = self.figure.add_subplot(111)
        ax.plot(rplt_y)
        ax.set_ylim(ymin= 0, ymax = 50)

##        ax.clear()
##        ax.plot(rplt_x[l-1],rplt_y[l-1])
        
##        FigureCanvas.updateGeometry(self)

####        anim.FuncAnimation.__init__(self, self.figure, self._update_canvas_, fargs=(y,), interval=interval, blit=True)
####        ax.clear()
####        ax.plot(rplt_y, 'r-')
####        
######        self.xlim([25, 50])
####        ax.set_title('Thermal Imager plot')
####        ax.set_xlabel('x')
####        ax.set_ylabel('Temperature [degC]')
    
        self.draw()
        

class MainWidget(QtWidgets.QWidget):
    rec_status_signal = QtCore.pyqtSignal(str)

    def doAction(self):
        # setting for loop to set value of progress bar
        for i in range(100):
            # slowing down the loop
            time.sleep(0.001)
            # setting value to progress bar
            self.pbar.setValue(i)

    def update_progress(self, val):
        self.pbar.setValue(val)

        self.pbar.setStyleSheet("QProgressBar"
                          "{"
                          "background-color : lightblue;"
                          "border : 1px"
                          "}")
        if val <= 20:
            # changing the color of process bar
            self.pbar.setStyleSheet("QProgressBar::chunk "
                              "{"
                              "background-color: red;"
                              "}")
        if val > 20 and val <= 80:
            # changing the color of process bar
            self.pbar.setStyleSheet("QProgressBar::chunk "
                              "{"
                              "background-color: green;"
                              "}")
        if val > 80:
            # changing the color of process bar
            self.pbar.setStyleSheet("QProgressBar::chunk "
                              "{"
                              "background-color: yellow;"
                              "}")

    def update_progress1(self, val1):
        self.pbar1.setValue(val1)


    def update_plot_data(self):
        global DS18b20, tmp_avg, surr_temp_avg, count
        self.x = self.x[1:]  # Remove the first y element.

        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.

        if (count%3 == 0):
            print("tmp_avg: ", tmp_avg)
            tmp_avg = tmp_avg + 2.6
            # surr_temp_avg = surr_temp_avg - 2.6
        else:
            tmp_avg = tmp_avg - 2.6
            # surr_temp_avg = surr_temp_avg + 2.6
        print("T: ", tmp_avg, "S: ", surr_temp_avg, "C: ", count)
        self.y = self.y[1:]  # Remove the first
        self.y.append(tmp_avg)  # Add a new random value.
        self.data_line.setData(self.x, self.y)  # Update the data#

        self.yy = self.yy[1:]  # Remove the first
        self.yy.append(surr_temp_avg)  # Add a new random value.
        self.data_line1.setData(self.x, self.yy)

        print("val = ", val)
        self.yyy = self.yyy[1:]  # Remove the first
        self.yyy.append(DS18b20)  # Add a new random value.
        self.data_line2.setData(self.x, self.yyy)

    def takeinputs(self):
        global set_point
        set_point, done1 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter background temperature:')
        if done1:
            # Showing confirmation message along
            # with information provided by user.
            self.label.setText('Information stored Successfully\n''CGPA: '
                               + str(set_point))

            # Hide the pushbutton after inputs provided by the use.
            self.pushButton.hide()

            # def update_plot_data1(self):
    #     global tmp_avg, surr_temp_avg, count
    #     self.x1 = self.x1[1:]  # Remove the first y element.
    #     self.x1.append(self.x1[-1] + 1)  # Add a new value 1 higher than the last.
    #     # if (count%3 == 0):
    #     #     print("tmp_avg: ", tmp_avg)
    #     #     tmp_avg = tmp_avg + 2
    #     #     surr_temp_avg = surr_temp_avg - 2
    #     # else:
    #     #     tmp_avg = tmp_avg - 2
    #     #     surr_temp_avg = surr_temp_avg + 2
    #     print("T: ", tmp_avg, "S: ", surr_temp_avg, "C: ", count)
    #     self.y1 = self.y1[1:]  # Remove the first
    #     self.y1.append(randint(0, 100))  # Add a new random value.
    #     # print("TEMP: ", tmp_avg)
    #
    #     self.yy1 = self.yy1[1:]  # Remove the first
    #     self.yy1.append(randint(0, 100))  # Add a new random value.
    #
    #     self.data_line2.setData(self.x1, self.y1)  # Update the data# .
    #     self.data_line3.setData(self.x1, self.yy1)


    def __init__(self, parent=None):


        super().__init__(parent)


        self.thermal_display_widget = ThermalDisplayWidget(rec_status_signal=self.rec_status_signal)
        self.thermal_video_stream = ThermalVideoStream()

        image_data_slot = self.thermal_display_widget.image_data_slot
        # print(image_data_slot)
        # plt_thread =  self.thermal_display_widget.thread

        self.thermal_video_stream.image_data_signal.connect(image_data_slot)

        # self.thermal_video_stream.image_data_signal.connect(plt_thread)
        # print(image_data_slot)

        self.rec_status_signal.connect(self.update_recording_label)

        # b = self.doAction
        # self.thermal_video_stream.image_data_signal.connect(b)

        self.label_1 = QtWidgets.QLabel("ADAPTIVE - Thermal Camouflage [Strategic & Tactical Defence System]", self)
        self.label_1.setFont(QtGui.QFont('Courier', 15))
        self.label_1.setGeometry(400, 0, 800, 25)

        layoutDown = QtWidgets.QHBoxLayout()

        self.label_2 = QtWidgets.QLabel("PID ERROR:", self)
        self.label_2.setFont(QtGui.QFont('Times', 8))
        self.label_2.setGeometry(10, 30, 200, 25)

        self.pbar = QtWidgets.QProgressBar(self)
        # self.pbar.setFormat('PID error')
        self.pbar.setAlignment(QtCore.Qt.AlignCenter)
        self.pbar.setGeometry(30, 40, 200, 25)
        layoutDown.addWidget(self.pbar)
        self.thermal_display_widget.progress_signal.connect(self.update_progress)

        self.label_3 = QtWidgets.QLabel("Camouflage level", self)
        self.label_3.setFont(QtGui.QFont('Times', 8))
        self.label_3.setGeometry(470, 30, 200, 25)

        self.pbar1 = QtWidgets.QProgressBar(self)
        # self.pbar1.setFormat('Camouflage level')
        self.pbar1.setAlignment(QtCore.Qt.AlignCenter)
        self.pbar1.setGeometry(30, 40, 400, 25)
        layoutDown.addWidget(self.pbar1)
        self.thermal_display_widget.progress_signal1.connect(self.update_progress1)


        layoutDown0 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')      # Set background color of plot
        self.graphWidget.setYRange(0, 55)        # set Y-axis range of plot
        self.graphWidget.setMaximumWidth(700)    # Add width of plot
        self.graphWidget.setMaximumHeight(400)   # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response", color="b", size="15pt")
        # self.graphWidget.addLegend()
        layoutDown0.addWidget(self.graphWidget)
        self.x = list(range(100))  # 100 time points
        self.y = [randint(0,1) for _ in range(100)]  # 100 data points
        self.yy = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yyy = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line =  self.graphWidget.plot(self.x, self.y, pen=pen)
        pen1 = pg.mkPen(color=(0, 255, 0))
        self.data_line1 = self.graphWidget.plot(self.x, self.yy, pen=pen1)
        pen2 = pg.mkPen(color=(255, 0, 255))
        self.data_line2 = self.graphWidget.plot(self.x, self.yyy, pen=pen2)
        self.thermal_display_widget.plot_signal.connect(self.update_plot_data)



        # layoutDown01 = QtWidgets.QHBoxLayout()
        # self.graphWidget1 = pg.PlotWidget()
        # self.graphWidget1.setBackground('w')  # Set background color of plot
        # self.graphWidget1.setYRange(0, 55)  # set Y-axis range of plot
        # self.graphWidget1.setMaximumWidth(450)  # Add width of plot
        # self.graphWidget1.setMaximumHeight(400)  # Add height of plot
        # # Add Axis Labels
        # styles1 = {"color": "#f00", "font-size": "10px"}
        # self.graphWidget1.setLabel("left", "Temperature (°C)", **styles1)
        # self.graphWidget1.setLabel("bottom", "Hour (H)", **styles1)
        # # show the grids  on the graph
        # self.graphWidget1.showGrid(x=True, y=True)
        # self.graphWidget1.setTitle("PID Response", color="b", size="15pt")
        # # self.graphWidget.addLegend()
        # layoutDown01.addWidget(self.graphWidget1)
        # self.x1 = list(range(100))  # 100 time points
        # # self.x.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        # self.y1 = [randint(0, 100) for _ in range(100)]  # 100 data points
        # self.yy1 = [randint(0, 100) for _ in range(100)]  # 100 data points
        # self.graphWidget1.setBackground('w')
        # pen2 = pg.mkPen(color=(255, 0, 0))
        # self.data_line2 = self.graphWidget1.plot(self.x1, self.y1, pen=pen2)
        # pen3 = pg.mkPen(color=(0, 255, 0))
        # self.data_line3 = self.graphWidget1.plot(self.x1, self.yy1, pen=pen3)
        # self.thermal_display_widget.plot_signal1.connect(self.update_plot_data1)


        layoutDown1 = QtWidgets.QVBoxLayout()
        self.run_button = QtWidgets.QPushButton('Start')
        layoutDown1.addWidget(self.run_button)
        # self.run_button.clicked.connect(
        #     self.thermal_video_stream.start_capture)
        self.stop_button = QtWidgets.QPushButton('Stop')
        layoutDown1.addWidget(self.stop_button)
        # self.stop_button.clicked.connect(
        #     self.thermal_video_stream.stop_capture)
        self.sav_img_button = QtWidgets.QPushButton('Save Img')
        layoutDown1.addWidget(self.sav_img_button)
        self.sav_img_button.clicked.connect(
            self.thermal_display_widget.sav_img)
##        self.flip_x_button = QtWidgets.QPushButton('Flip X')
##        layoutDown1.addWidget(self.flip_x_button)
##        self.flip_x_button.clicked.connect(self.thermal_display_widget.flip_x_toggle)
##        self.flip_y_button = QtWidgets.QPushButton('Flip Y')
##        layoutDown1.addWidget(self.flip_y_button)
##        self.flip_y_button.clicked.connect(self.thermal_display_widget.flip_y_toggle)

##        self.ok = QtWidgets.QPushButton('Okay')
##        layoutDown1.addWidget(self.ok)
##
##        layoutDown2 = QtWidgets.QVBoxLayout()
####        self.dump_rawdata_button = QtWidgets.QPushButton('Dump Raw CSV')
####        layoutDown2.addWidget(self.dump_rawdata_button)
####        self.dump_rawdata_button.clicked.connect(
####            self.thermal_display_widget.dump_rawdata)
####        self.dump_rawdata_png_button = QtWidgets.QPushButton('Dump Raw image')
####        layoutDown2.addWidget(self.dump_rawdata_png_button)
####        self.dump_rawdata_png_button.clicked.connect(
####            self.thermal_display_widget.dump_rawdata_png)
####        self.seq_cap_button = QtWidgets.QPushButton('Burst Seq Cap')
####        layoutDown2.addWidget(self.seq_cap_button)
####        self.seq_cap_button.clicked.connect(self.thermal_video_stream.seq_cap)
##        self.sav_img_button = QtWidgets.QPushButton('Save Img')
##        layoutDown2.addWidget(self.sav_img_button)
##        self.sav_img_button.clicked.connect(
##            self.thermal_display_widget.sav_img)

        layoutDown3 = QtWidgets.QVBoxLayout()
        self.save_temperature_button = QtWidgets.QPushButton('Save Temperature')
        layoutDown3.addWidget(self.save_temperature_button)
        self.save_temperature_button.clicked.connect(
            self.thermal_display_widget.save_temperature)
        self.self_cal_button = QtWidgets.QPushButton('Shutter Cal')
        layoutDown3.addWidget(self.self_cal_button)
        # self.self_cal_button.clicked.connect(self.thermal_video_stream.selfCal)
##        self.calDialog = QtWidgets.QPushButton('Cali...')
##        layoutDown3.addWidget(self.calDialog)
##        self.calDialog.clicked.connect(self.startCalDialog)
        self.edit_par_button = QtWidgets.QPushButton('Meas...')
        layoutDown3.addWidget(self.edit_par_button)
        self.edit_par_button.clicked.connect(self.editPara)
        
##        layoutDown4 = QtWidgets.QHBoxLayout()
##        self.rec_status_label = QtWidgets.QLabel('Not Recording')
##        layoutDown4.addWidget(self.rec_status_label)
##        self.rec_button = QtWidgets.QPushButton('Rec Run/Stop')
##        layoutDown4.addWidget(self.rec_button)
##        self.rec_button.clicked.connect(self.thermal_display_widget.toggle_recording)
##        

        layoutDown5 = QtWidgets.QVBoxLayout()
        self.sp = QtWidgets.QPushButton('Enter Set point')
        layoutDown5.addWidget(self.sp)
        self.sp.clicked.connect(self.takeinputs)

        self.exit_button = QtWidgets.QPushButton('EXIT NOW!')
        layoutDown5.addWidget(self.exit_button)
        self.exit_button.clicked.connect(exitNow)




##        self.set_sz_button = QtWidgets.QPushButton('Set Size')
##        layoutDown5.addWidget(self.set_sz_button)
##        self.set_sz_button.clicked.connect(
##            self.thermal_display_widget.toggle_size)
##        self.settings_button = QtWidgets.QPushButton('Proc...')
##        layoutDown5.addWidget(self.settings_button)
##        self.settings_button.clicked.connect(startSettings)

        if QtWidgets.QDesktopWidget().screenGeometry(-1).height() < 1200:
            layoutVert = QtWidgets.QHBoxLayout()
            layoutVert.addLayout(layoutDown1)
##            layoutVert.addLayout(layoutDown2)
            layoutVert.addLayout(layoutDown3)
##            layoutVert.addLayout(layoutDown4)
            layoutVert.addLayout(layoutDown5)

            layoutVert1 = QtWidgets.QVBoxLayout()
            layoutVert1.addLayout(layoutDown)
            layoutVert1.addLayout(layoutDown0)
            # layoutVert1.addLayout(layoutDown01)
            layoutVert1.addLayout(layoutVert)
            
            layoutHori = QtWidgets.QHBoxLayout()
            layoutHori.addLayout(layoutVert1)
            layoutHori.addWidget(self.thermal_display_widget)
            self.setLayout(layoutHori)
        else:
            layoutVert = QtWidgets.QVBoxLayout()
            layoutVert.addWidget(self.thermal_display_widget)
            layoutVert.addLayout(layoutDown1)
##            layoutVert.addLayout(layoutDown2)
            layoutVert.addLayout(layoutDown3)
##            layoutVert.addLayout(layoutDown4)
            layoutVert.addLayout(layoutDown5)
            self.setLayout(layoutVert)


    def editPara(self):
        dialog = TemperatureMeasParaDialog(self.thermal_video_stream)
        dialog.exec_()
        self.thermal_video_stream.selfCal()

    def startCalDialog(self):
        dialog = CalibrationDialog(self.thermal_video_stream)
        dialog.exec_()
        
    def update_recording_label(self, text):
        self.rec_status_label.setText(text)


def startSettings():
    dialog = SettingsDialog()
    dialog.exec_()


def exitNow():
    os._exit(0)


class CalibrationDialog(QtWidgets.QDialog):
    def __init__(self, therm_cam_handle, parent=None):
        super(CalibrationDialog, self).__init__(parent)
        self.setWindowTitle("Calibration")
        self.therm_cam_handle = therm_cam_handle
        layoutH = QtWidgets.QHBoxLayout()
        layoutVL = QtWidgets.QVBoxLayout()

        self.cover_cal_button = QtWidgets.QPushButton(
            'Naive Cover Calibration')
        layoutVL.addWidget(self.cover_cal_button)
        self.cover_cal_button.clicked.connect(therm_cam_handle.self_cov_cal)
        self.cal_clr_button = QtWidgets.QPushButton('Clear Calibration Info')
        layoutVL.addWidget(self.cal_clr_button)
        self.cal_clr_button.clicked.connect(therm_cam_handle.calZeroize)
        self.cancel_button = QtWidgets.QPushButton('Close')
        layoutVL.addWidget(self.cancel_button)
        self.cancel_button.clicked.connect(self.cancel)

        layoutH.addLayout(layoutVL)

        layoutForm = QtWidgets.QFormLayout()

        self.SlopeCntl = QtWidgets.QLineEdit("slope.csv")
        layoutForm.addRow('Table for Slope (img*Slope+Offset)', self.SlopeCntl)
        self.OffsetCntl = QtWidgets.QLineEdit("offset.csv")
        layoutForm.addRow('Table for Offset:', self.OffsetCntl)
        self.read_button = QtWidgets.QPushButton('Read Calibration File')
        layoutForm.addWidget(self.read_button)
        self.read_button.clicked.connect(self.read_cal_const)

        layoutH.addLayout(layoutForm)
        self.setLayout(layoutH)

    def read_cal_const(self):
        self.therm_cam_handle.read_cal_constants(self.SlopeCntl.text(),
                                                 self.OffsetCntl.text())
        print('Reading Cal Const Done')

    def cancel(self):
        self.close()


class TemperatureMeasParaDialog(QtWidgets.QDialog):
    def __init__(self, therm_cam_handle, parent=None):
        global tempMode, rawMode
        self.therm_cam_handle = therm_cam_handle
        super(TemperatureMeasParaDialog, self).__init__(parent)
        self.setWindowTitle("Temperature Measurement Parameters")
        Emiss = c_float()
        refltmp = c_float()
        airtemp = c_float()
        Humi = c_float()
        Distance = c_ushort()
        Fix = c_float()
        XthermDll.GetFixParam(
            byref(Emiss), byref(refltmp), byref(airtemp), byref(Humi),
            byref(Distance), byref(Fix))
        print(
            'FixParamFromDll:Emiss:{:.2f},refltmp:{:.2f},airtemp:{:.2f},Humi:{:.2f},Distance:{:d},Fix:{:.2f}'.
            format(Emiss.value, refltmp.value, airtemp.value, Humi.value,
                   Distance.value, Fix.value))

        layoutForm = QtWidgets.QFormLayout()

        self.EmissCntl = QtWidgets.QLineEdit('{:.2f}'.format(Emiss.value))
        layoutForm.addRow('Emiss', self.EmissCntl)

        self.refltmpCntl = QtWidgets.QLineEdit('{:.2f}'.format(refltmp.value))
        layoutForm.addRow('refltmp', self.refltmpCntl)

        self.airtempCntl = QtWidgets.QLineEdit('{:.2f}'.format(airtemp.value))
        layoutForm.addRow('airtemp', self.airtempCntl)

        self.HumiCntl = QtWidgets.QLineEdit('{:.2f}'.format(Humi.value))
        layoutForm.addRow('Humi', self.HumiCntl)

        self.DistanceCntl = QtWidgets.QLineEdit('{:d}'.format(Distance.value))
        layoutForm.addRow('Distance', self.DistanceCntl)

        self.FixCntl = QtWidgets.QLineEdit('{:.2f}'.format(Fix.value))
        layoutForm.addRow('Fix', self.FixCntl)

        self.HighTempEn = QtWidgets.QCheckBox()
        self.HighTempEn.setChecked(tempMode)
        layoutForm.addRow('Use 0x8021 mode?', self.HighTempEn)
        
        self.SensorRawEn = QtWidgets.QCheckBox()
        self.SensorRawEn.setChecked(rawMode)
        layoutForm.addRow('Use 0x8002 mode?', self.SensorRawEn)

        layoutDown = QtWidgets.QHBoxLayout()
        self.save_settings_button = QtWidgets.QPushButton('Save Settings')
        layoutDown.addWidget(self.save_settings_button)
        self.save_settings_button.clicked.connect(self.save_settings)
        self.cancel_button = QtWidgets.QPushButton('Cancel')
        layoutDown.addWidget(self.cancel_button)
        self.cancel_button.clicked.connect(self.cancel)

        layoutVert = QtWidgets.QVBoxLayout()
        layoutVert.addLayout(layoutForm)
        layoutVert.addLayout(layoutDown)
        self.setLayout(layoutVert)

    def save_settings(self):
        self.therm_cam_handle.set_0x02_mode(self.SensorRawEn.isChecked())
        self.therm_cam_handle.set_0x21_mode(self.HighTempEn.isChecked())
        try:
            Emiss_n = float(self.EmissCntl.text())
            refltmp_n = float(self.refltmpCntl.text())
            airtemp_n = float(self.airtempCntl.text())
            Humi_n = float(self.HumiCntl.text())
            Distance_n = int(self.DistanceCntl.text())
            Fix_n = float(self.FixCntl.text())
            if not Emiss_n < 1.0 and Emiss_n > 0.0 and Humi_n < 1.0 and Humi_n > 0.0:
                print('Out of Range')
                raise ValueError('Range')
            XthermDll.UpdateFixParam(
                c_float(Emiss_n), c_float(refltmp_n), c_float(airtemp_n),
                c_float(Humi_n), c_ushort(Distance_n), c_float(Fix_n))
            self.close()
        except ValueError:
            print('Invaild Input')

    def cancel(self):
        print('Canceled')
        self.close()


class SettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        global seq_cap_num, drawHiLo, use_clahe, colormap, SCas8bits, SCuseCal, use_deStrip, use_NLMDenoising, use_equHist, use_canny, use_bf, bf_sigmaColor, use_sharpening
        super(SettingsDialog, self).__init__(parent)
        self.setWindowTitle("Parameters")
        layoutForm = QtWidgets.QFormLayout()

        self.SeqNumCntl = QtWidgets.QLineEdit('{:d}'.format(seq_cap_num))
        layoutForm.addRow('Sequence Burst Capture Count', self.SeqNumCntl)

        self.Seq8bits = QtWidgets.QCheckBox()
        self.Seq8bits.setChecked(SCas8bits)
        layoutForm.addRow('Sequence Burst Capture as 8bits Images?',
                          self.Seq8bits)

        self.SCCal = QtWidgets.QCheckBox()
        self.SCCal.setChecked(SCuseCal)
        layoutForm.addRow('Do Calibration for Sequence Burst Capture?',
                          self.SCCal)

        # https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html
        self.CMCntl = QtWidgets.QLineEdit('{:d}'.format(colormap))
        layoutForm.addRow('OpenCV Colormap NO (-1 for greyscale)(8bits only)',
                          self.CMCntl)

        self.DrawHiLoCntl = QtWidgets.QCheckBox()
        self.DrawHiLoCntl.setChecked(drawHiLo)
        layoutForm.addRow('Mark Max&Min Temperature Points?',
                          self.DrawHiLoCntl)

        self.SharpenCntl = QtWidgets.QCheckBox()
        self.SharpenCntl.setChecked(use_sharpening)
        layoutForm.addRow('Sharpen?', self.SharpenCntl)

        self.DeStripCntl = QtWidgets.QCheckBox()
        self.DeStripCntl.setChecked(use_deStrip)
        layoutForm.addRow('DeStrip?', self.DeStripCntl)

        self.HECntl = QtWidgets.QCheckBox()
        self.HECntl.setChecked(use_equHist)
        layoutForm.addRow('Use Histogram Equalization?', self.HECntl)

        self.CLAHECntl = QtWidgets.QCheckBox()
        self.CLAHECntl.setChecked(use_clahe)
        layoutForm.addRow('Enable CLAHE?', self.CLAHECntl)

        self.BFCntl = QtWidgets.QCheckBox()
        self.BFCntl.setChecked(use_bf)
        layoutForm.addRow('Enable bilateralFilter?', self.BFCntl)

        self.BFsigmaColorCntl = QtWidgets.QLineEdit(
            '{:1.3f}'.format(bf_sigmaColor))
        layoutForm.addRow('bilateralFilter sigma color', self.BFsigmaColorCntl)

        self.NLMDCntl = QtWidgets.QCheckBox()
        self.NLMDCntl.setChecked(use_NLMDenoising)
        layoutForm.addRow(
            'Enable NLMeansDenoising for Saving(SLOW,8bits Only)?',
            self.NLMDCntl)

        self.CannyCtrl = QtWidgets.QCheckBox()
        self.CannyCtrl.setChecked(use_canny)
        layoutForm.addRow('Use Canny Edge Enhancement?', self.CannyCtrl)

        self.IntpAlgoCntl = QtWidgets.QComboBox()
        self.IntpAlgoCntl.addItem("Nearest", cv2.INTER_NEAREST)
        self.IntpAlgoCntl.addItem("Linear", cv2.INTER_LINEAR)
        self.IntpAlgoCntl.addItem("Cubic", cv2.INTER_CUBIC)
        self.IntpAlgoCntl.addItem("Lanczos4", cv2.INTER_LANCZOS4)
        layoutForm.addRow('Interpolation Algorithm', self.IntpAlgoCntl)

        self.SvFmt = QtWidgets.QComboBox()
        self.SvFmt.addItem("PNG", ".png")
        self.SvFmt.addItem("TIFF", ".tif")
        layoutForm.addRow('Save Format', self.SvFmt)

        layoutDown = QtWidgets.QHBoxLayout()
        self.save_settings_button = QtWidgets.QPushButton('Save Settings')
        layoutDown.addWidget(self.save_settings_button)
        self.save_settings_button.clicked.connect(self.save_settings)
        self.cancel_button = QtWidgets.QPushButton('Cancel')
        layoutDown.addWidget(self.cancel_button)
        self.cancel_button.clicked.connect(self.cancel)

        layoutVert = QtWidgets.QVBoxLayout()
        layoutVert.addLayout(layoutForm)
        layoutVert.addLayout(layoutDown)
        self.setLayout(layoutVert)

    def save_settings(self):
        global seq_cap_num, drawHiLo, use_clahe, colormap, resizeAlgo, saveFormat, SCuseCal, use_deStrip, SCas8bits, use_NLMDenoising, use_equHist, use_canny, use_bf, bf_sigmaColor, use_sharpening
        try:
            drawHiLo = self.DrawHiLoCntl.isChecked()
            use_clahe = self.CLAHECntl.isChecked()
            SCas8bits = self.Seq8bits.isChecked()
            SCuseCal = self.SCCal.isChecked()
            resizeAlgo = self.IntpAlgoCntl.currentData()
            use_NLMDenoising = self.NLMDCntl.isChecked()
            use_canny = self.CannyCtrl.isChecked()
            use_bf = self.BFCntl.isChecked()
            use_equHist = self.HECntl.isChecked()
            use_deStrip = self.DeStripCntl.isChecked()
            use_sharpening = self.SharpenCntl.isChecked()
            saveFormat = self.SvFmt.currentData()
            seq_cap_num_tmp = int(self.SeqNumCntl.text())
            colormap_tmp = int(self.CMCntl.text())
            bf_sigmaColor_tmp = float(self.BFsigmaColorCntl.text())
            if not (seq_cap_num_tmp > 1 and colormap_tmp >= -1
                    and colormap_tmp < 20 and bf_sigmaColor_tmp > 0
                    and bf_sigmaColor_tmp < 1):
                print('Out of Range')
                raise ValueError('Range')
            seq_cap_num = seq_cap_num_tmp
            colormap = colormap_tmp
            bf_sigmaColor = bf_sigmaColor_tmp
            self.close()
        except ValueError:
            print('Invaild Input')

    def cancel(self):
        print('Canceled')
        self.close()


def equHist16(img):
    oldsize = img.shape
    hist = cv2.calcHist(img.reshape(1, oldsize[0] * oldsize[1]), [0], None, [65536], (0, 65536))
    cs = np.cumsum(hist)
    nj = (cs - cs.min()) * 65535
    N = cs.max() - cs.min()
    cs = nj / N
    cs = cs.astype('uint16')
    img_new = cs[img]
    return img_new.reshape(oldsize)


def deStripping(img, k=(1, 1, 1), iter=1):
    leftHalf = round(np.size(img, 1) / 2)
    imgFlt = img.astype(np.float32)
    for nn in range(iter):
        merit = np.abs(cv2.Laplacian(cv2.GaussianBlur(imgFlt, (3, 3), 3), cv2.CV_32F)) / 65536
        merit = 1 / pow(merit + 0.005, 3)
        leftErr = imgFlt[:, 0:leftHalf] - cv2.GaussianBlur(imgFlt[:, 0:leftHalf], (1, 5), 10)
        horiLeftAcc = np.average(leftErr, weights=merit[:, 0:leftHalf], axis=1)
        rightErr = imgFlt[:, leftHalf:] - cv2.GaussianBlur(imgFlt[:, leftHalf:], (1, 5), 10)
        horiRightAcc = np.average(rightErr, weights=merit[:, leftHalf:], axis=1)
        vertErr = imgFlt - cv2.GaussianBlur(img, (5, 1), 10)
        vertAcc = np.average(vertErr, weights=merit, axis=0)
        for n in range(0, np.size(img, 0)):
            imgFlt[n, 0:leftHalf] = cv2.subtract(imgFlt[n, 0:leftHalf], (k[0] * horiLeftAcc[n])).squeeze(1)
            imgFlt[n, leftHalf:] = cv2.subtract(imgFlt[n, leftHalf:], (k[1] * horiRightAcc[n])).squeeze(1)
        for n in range(0, np.size(img, 1)):
            imgFlt[:, n] = cv2.subtract(imgFlt[:, n], (k[2] * vertAcc[n])).squeeze(1)
    return cv2.normalize(src=imgFlt, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_16U)



def main():
    app = QtWidgets.QApplication(sys.argv)

    main_window = QtWidgets.QMainWindow()
    main_window.setWindowTitle("Xtherm T3s")
    # main_window.showFullScreen()
    main_widget = MainWidget()
    main_window.setCentralWidget(main_widget)

    main_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':

    main()


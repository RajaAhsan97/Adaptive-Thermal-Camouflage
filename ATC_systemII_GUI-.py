"""
    Code bt RMA
    ATC system II GUI:
    The GUI is designed to blend the structure of CO ATC system II 
"""

"""Example Program for XTherm T3s Camera using OpenCV"""
import cProfile
from ctypes import *  ## ['ctypes' is a function library for PYTHON. It provides C compatible datatype & support calling functions in DLL  (see this for MATLAB)]
import sys
import os
import datetime, time
import pandas as pd
import serial
import cv2
import numpy as np
from numpy import genfromtxt
from scipy import ndimage
import math
from scipy.spatial.distance import pdist
from scipy.spatial import cKDTree
from scipy.ndimage import binary_fill_holes
import operator
from functools import reduce
# loading decimal library
from decimal import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from random import randint
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import matplotlib.animation as animation
import datetime as dt
import csv
import imutils

from numpy import savetxt
from SSIM_PIL import compare_ssim

from PIL import Image as IMAGE

from multiprocessing import Process

from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5 import QtGui
from datetime import datetime

#_______________________________________________________________________________________________________
# Loading thermal camera DLL file
is_64bits = sys.maxsize > 2 ** 32
print('Loading binary 64 bit version of XthermDLL.dll file')
XthermDll = cdll.LoadLibrary(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Project1.dll'))  # Xtherm.dll
print('DLL loaded successfully')
#_______________________________________________________________________________________________________

if not hasattr(QtGui.QImage, 'Format_Grayscale16'):
    force_use_8bits = True
    print(
        'The version of PyQt5 ({}) used is too old (should > 5.13)\nQtGui.QImage does not support 16bit grayscale, forcing to use 8bit display'.format(
            QtCore.PYQT_VERSION_STR))
else:
    force_use_8bits = False

# Establish connection with serial port 
try:
    serialport = serial.Serial(port="COM3", baudrate=115200, timeout=1)
    time.sleep(2)
    print("Connection to" + 'port1' + "established successfully!\n")
except Exception as e:
    print(e)


#___________________________________________________________________________
# Reading Setpoint temperature data for TECs on ATC-II characteristic pixel
print("Reading SP temperatures data from database........")
ATC2_SP_data_file = str('CTGP-II Locations Temperatures')
string1 = ATC2_SP_data_file + str('.txt')
string2 = ATC2_SP_data_file + str('.csv')
read_file = pd.read_csv(string1)
read_file.to_csv(string2, index=None)

with open(string1) as f:
    CTGP_II_SPs = f.readlines()

strt_offset = 6
TEC1_SP = [0]*250
TEC2_SP = [0]*250
TEC3_SP = [0]*250
TEC4_SP = [0]*250
TEC5_SP = [0]*250
TEC6_SP = [0]*250

for i in range(0,250):
    TEC1_SP[i] = float(CTGP_II_SPs[0][strt_offset] + CTGP_II_SPs[0][strt_offset + 1] + CTGP_II_SPs[0][strt_offset + 2] + CTGP_II_SPs[0][strt_offset + 3] + CTGP_II_SPs[0][strt_offset + 4])
    TEC2_SP[i] = float(CTGP_II_SPs[1][strt_offset] + CTGP_II_SPs[1][strt_offset + 1] + CTGP_II_SPs[1][strt_offset + 2] + CTGP_II_SPs[1][strt_offset + 3] + CTGP_II_SPs[1][strt_offset + 4])
    TEC3_SP[i] = float(CTGP_II_SPs[2][strt_offset] + CTGP_II_SPs[2][strt_offset + 1] + CTGP_II_SPs[2][strt_offset + 2] + CTGP_II_SPs[2][strt_offset + 3] + CTGP_II_SPs[2][strt_offset + 4])
    TEC4_SP[i] = float(CTGP_II_SPs[3][strt_offset] + CTGP_II_SPs[3][strt_offset + 1] + CTGP_II_SPs[3][strt_offset + 2] + CTGP_II_SPs[3][strt_offset + 3] + CTGP_II_SPs[3][strt_offset + 4])
    TEC5_SP[i] = float(CTGP_II_SPs[4][strt_offset] + CTGP_II_SPs[4][strt_offset + 1] + CTGP_II_SPs[4][strt_offset + 2] + CTGP_II_SPs[4][strt_offset + 3] + CTGP_II_SPs[4][strt_offset + 4])
    TEC6_SP[i] = float(CTGP_II_SPs[5][strt_offset] + CTGP_II_SPs[5][strt_offset + 1] + CTGP_II_SPs[5][strt_offset + 2] + CTGP_II_SPs[5][strt_offset + 3] + CTGP_II_SPs[5][strt_offset + 4])
    strt_offset = strt_offset + 12
print("DATA read successful")
ATC2_px_loc = 249

curr_time = datetime.now()
prev_time = curr_time
load_flag = 0
#___________________________________________________________________________

# defaults
seq_cap_num = 12
SCuseCasl = 1
SCas8bits = 0
drawHiLo = 1
use_clahe = 0  # C
colormap = 2  # -1
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
# flag to view plots for set of 3 TECs on GUI
P = 0

#___________________________________________________________________
# variables used to transmit PV temperatures to microcontroller
tmp_avg1 = 0
tmp_avg2 = 0
tmp_avg3 = 0
tmp_avg4 = 0
tmp_avg5 = 0
tmp_avg6 = 0
#___________________________________________________________________

#___________________________________________________________________
# variables to store SP and PV temperatures for plotting them on GUI
t_avg1 = 0
s_temp_avg1 = 0
t_avg2 = 0
s_temp_avg2 = 0
t_avg3 = 0
s_temp_avg3 = 0
t_avg4 = 0
s_temp_avg4 = 0
t_avg5 = 0
s_temp_avg5 = 0
t_avg6 = 0
s_temp_avg6 = 0
#___________________________________________________________________

#___________________________________________________________________
# for storing no. of coordinates resides insides the TECs selections
cc1 = 0     # TEC1
cc2 = 0     # TEC2
cc3 = 0     # TEC3
cc4 = 0     # TEC4
cc5 = 0     # TEC5
cc6 = 0     # TEC6
#___________________________________________________________________

#___________________________________________________________________
# for storinf the coordinates of whole ATC-II geometry
CTGP_region_X = [0]
CTGP_region_Y = [0]
#___________________________________________________________________

#___________________________________________________________________
# for storing X and Y coordinates of TECs 
region1_X_coord = [0]
region1_Y_coord = [0]
region2_X_coord = [0]
region2_Y_coord = [0]
region3_X_coord = [0]
region3_Y_coord = [0]
region4_X_coord = [0]
region4_Y_coord = [0]
region5_X_coord = [0]
region5_Y_coord = [0]
region6_X_coord = [0]
region6_Y_coord = [0]
#___________________________________________________________________

#___________________________________________________________________
# for storing calculated average temperatures TECs and background
# regions 
CTGP_SURR_region4 = 0
CTGP_region3 = 0
CTGP_SURR_region5 = 0
CTGP_region2 = 0
CTGP_SURR_region6 = 0
CTGP_region1 = 0
CTGP_SURR_region1 = 0
CTGP_region6 = 0
CTGP_SURR_region2 = 0
CTGP_region5 = 0
CTGP_SURR_region3 = 0
CTGP_region4 = 0
#___________________________________________________________________

ax = 0
fig = 0
prnt_temps_flg = 0

# for storing the thermal frame for further processing it
img = [0]

count = 1
cnt = 0

start_capture = 0
Frame_cap = 0
clahe = cv2.createCLAHE(clipLimit=400, tileGridSize=(8, 8))
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
video_format = ".avi"
resizeAlgo = cv2.INTER_NEAREST


def SearchForXThermT3s():
    # Try automatic searching for cameras with the correct resolution and frame rate
    for camera_port in range(0, 254):
        camera = cv2.VideoCapture(camera_port)  # ,cv2.CAP_DSHOW)
        if camera.isOpened():
            width = camera.get(3)  # CAP_PROP_FRAME_WIDTH
            height = camera.get(4)  # CAP_PROP_FRAME_HEIGHT
            frate = camera.get(5)  # CAP_PROP_FPS
            print('n{:d}:w{:.0f}h{:.0f}f{:.2f}'.format(camera_port, width,
                                                       height, frate))
            if (width, height, frate) == (256, 196, 25):  # 384, 292, 25
                print('Found XTherm T3s')
                return camera
            else:
                camera.release()
                cv2.destroyAllWindows()
    return None


class FrameGrabbingThread(QtCore.QThread):
    running_signal = QtCore.pyqtSignal(int)

    def __init__(self, camera, calFunc, image_data_signal, parent=None):
        super(FrameGrabbingThread, self).__init__(parent)
        self.camera = camera
        self.calFunc = calFunc
        self.image_data_signal = image_data_signal
        self.running = False
        self.running_signal.connect(self.running_signal_slot)

    def running_signal_slot(self, signal):
        self.running = True if signal == 1 else False

    def run(self):
        while True:
            if self.running:
                # capture frame-by-frame
                ret, data = self.camera.read()
                # if frame is read correctly then [ret] is TRUE
                if ret:
                    data = data.view(np.uint16).reshape(196, 256)  # 292, 384
                    data[0:192, :] = self.calFunc(data[0:192, :])  # data[0:288, :]
                    self.image_data_signal.emit(data)
                else:
                    print('Frame grabbing failed')
                    # exitNow()
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
            self.camera = SearchForXThermT3s()
        else:
            self.camera = cv2.VideoCapture(camera_port)  # ,cv2.CAP_DSHOW)

        if not self.camera:
            sys.exit("Xtherm camera not found")
        ######################
        self.calOffset = np.zeros((192, 256), dtype=np.int16)  # (288, 384)
        self.calSlope = np.ones((192, 256), dtype=np.float32)  # (288, 384)
        #######################
        self.camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # CV_CAP_PROP_CONVERT_RGB
        # CV_CAP_PROP_ZOOM use raw mode
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8004)
        # CV_CAP_PROP_ZOOM calibrate
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8000)
        # CV_CAP_PROP_ZOOM temperature mode?
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8020)
        XthermDll.DataInit(256, 192)  # 384, 288
        print("resolution set")
        while True:
            ret, rawFrame = self.camera.read()
            if not ret:
                print('Frame grabbing failed')
                time.sleep(1)
                continue
            if not (rawFrame.size == 196 * 256 * 2):  # 292 * 384 * 2
                print('ERROR: Incorrect frame size, wrong camera selected?')
                exitNow()
            else:
                break

        # XthermDll.UpdateParam(tempMode, rawFrame.ctypes.data_as(POINTER(c_byte)))

    def start_capture(self):
        global serialport
        try:
            serialport = serial.Serial(port="COM3", baudrate=115200, timeout=1)
            time.sleep(2)
            print("Connection to" + 'port1' + "established successfully!\n")
        except Exception as e:
            print(e)

        self.capture_thread.running_signal.emit(1)

    def stop_capture(self):
        self.capture_thread.running_signal.emit(0)

    def selfCal(self):
        global Frame_cap
        isCapturing = self.capture_thread.running
        if isCapturing:
            self.stop_capture()
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8000)  # internal shutter calibrate
        Frame_cap = 1
        ret, rawFrame = self.camera.read()
        if not ret:
            print('Frame grabbing failed')
            exitNow()
        if not (rawFrame.size == 196 * 256 * 2):  # 292 * 384 * 2
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
        filename1 = 'SEQC-' + now.strftime('%Y-%m-%dT%H-%M-%S') + \
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
                    rawFrame.view(np.uint16).reshape(196, 256)[0:192, :])  # reshape(292, 384)[0:288, :]
            else:
                frame = rawFrame.view(np.uint16).reshape(196, 256)[0:192, :]  # reshape(292, 384)[0:288, :]
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
        frames = np.zeros((192, 256, cover_cal_avg))  # np.zeros((288, 384, cover_cal_avg))
        for n in range(cover_cal_avg):
            read, rawFrame = self.camera.read()
            frames[:, :, n] = rawFrame.view(np.uint16).reshape(196, 256)[0:192, :]  # 292, 384)[0:288, :]
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
            if off_tmp.shape != (192, 256):  # (288, 384)
                print('Wrong size of Offset')
                return
            self.calOffset = off_tmp
            slope_tmp = np.genfromtxt(
                os.path.join(
                    os.path.dirname(os.path.realpath(__file__)), slopeCSV),
                delimiter=',',
                dtype=np.float32)
            if slope_tmp.shape != (192, 256):  # (288, 384)
                print('Wrong size of Slope')
                return
            self.calSlope = slope_tmp
        except:
            print('Can not read file')

    ###############################3
    def applyCal(self, img):
        img[0:192, :] = (np.multiply(img[0:192, :], self.calSlope) + self.calOffset).astype(np.uint16)  # img[0:288, :]
        return img

    def calZeroize(self):
        self.calOffset = np.zeros((192, 256), dtype=np.int16)  # (288, 384)
        self.calSlope = np.ones((192, 256), dtype=np.float32)


class ThermalDisplayWidget(QtWidgets.QWidget):
    progress_signal = QtCore.pyqtSignal(int)
    progress_signal1 = QtCore.pyqtSignal(int)
    plot_signal = QtCore.pyqtSignal(int, int, int)
    plot_signal2 = QtCore.pyqtSignal(int, int, int)
    plot_signal3 = QtCore.pyqtSignal(int, int, int)
    plot_signal4 = QtCore.pyqtSignal(int, int, int)
    plot_signal5 = QtCore.pyqtSignal(int, int, int)
    plot_signal6 = QtCore.pyqtSignal(int, int, int)
    transmit_data = QtCore.pyqtSignal(int)
    load_SPs = QtCore.pyqtSignal(int)

    s1_progress = QtCore.pyqtSignal(int)
    s2_progress = QtCore.pyqtSignal(int)

    def __init__(self, resy=560 + 30, resx=432, infobarY=30, parent=None, rec_status_signal=None):
        # (self, resy=636 + 60, resx=768, infobarY=60, parent=None, rec_status_signal=None):

        super().__init__(parent)

        self.image = QtGui.QImage()
        self.video_writer = None
        self.video_frames = 0
        self.set_size(resy, resx, infobarY)
        self.rec_status_signal = rec_status_signal
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
            self.set_size(192 + 30, 256, 30)  # (288 + 30, 384, 30)
        else:
            self.set_size()

    def flip_x_toggle(self):
        self.flip_x = not self.flip_x

    def flip_y_toggle(self):
        self.flip_y = not self.flip_y

    def image_data_slot(self, rawFrame):
        global load_flag, curr_time, prev_time, prnt_temps_flg, ATC2_px_loc, TEC1_SP, TEC2_SP, TEC3_SP, TEC4_SP, TEC5_SP, TEC6_SP, Cropped, tmp_avg1, tmp_avg2, tmp_avg3, tmp_avg4, tmp_avg5, tmp_avg6, new_CTGP_region_Y, new_CTGP_region_X, CTGP_region_Y, CTGP_region_X, P, CTGP_SURR_region3, CTGP_region4, CTGP_SURR_region2, CTGP_region5, t_avg6, s_temp_avg6, t_avg5, s_temp_avg5, t_avg4, s_temp_avg4, CTGP_SURR_region4, CTGP_region3, CTGP_SURR_region5, CTGP_region2, CTGP_SURR_region6, CTGP_region1, CTGP_SURR_region1, CTGP_region6, t_avg3, s_temp_avg3, t_avg2, s_temp_avg2, cc1, cc2, cc3, cc4, cc5, cc6, region1_X_coord, region1_Y_coord, region2_X_coord, region2_Y_coord, region3_X_coord, region3_Y_coord, region4_X_coord, region4_Y_coord, region5_X_coord, region5_Y_coord, region6_X_coord, region6_Y_coord, Frame_cap, start_capture, img, fig, ax, count, t_avg1, s_temp_avg1, fl, flg, drawHiLo, use_clahe, clahe, colormap, use_NLMDenoising, use_deStrip, deStripk, deStripIter, use_equHist, use_bf, bf_sigmaColor, use_sharpening, sharpening_kernel

        #self.FrameTimeStamp = datetime.datetime.now()
        self.bufferedRawFrame = rawFrame
        maxtmp = c_float()
        maxx = c_int()
        maxy = c_int()
        mintmp = c_float()
        minx = c_int()
        miny = c_int()
        centertmp = c_float()
        tmparr = np.zeros(3, dtype=np.float32)
        XthermDll.GetTmpData(1, rawFrame.ctypes.data_as(POINTER(c_byte)),
                             byref(maxtmp), byref(maxx), byref(maxy),
                             byref(mintmp), byref(minx), byref(miny),
                             byref(centertmp),
                             tmparr.ctypes.data_as(POINTER(c_float)),
                             0)  # full frame temperature not necessary
        # there is not transformed temperature data in 400 deg range, use pixel values instead
        if tempMode:
            maxtmp.value = rawFrame[192, 4]  # [288, 4]
            mintmp.value = rawFrame[192, 7]  # [288, 7]
            centertmp.value = (rawFrame[143, 191] + rawFrame[143, 192] +
                               rawFrame[144, 191] + rawFrame[144, 192]) / 4
        fpatmp = c_float()
        coretmp = c_float()
        fpaavg = c_int()
        orgavg = c_int()
        XthermDll.GetDevData(byref(fpatmp), byref(coretmp), byref(fpaavg), byref(orgavg))

        # begin the processing of the image
        outputFrame = rawFrame[0:192, :]  # [0:192, :]

        outputFrame = np.transpose(outputFrame)

        maxtmp = c_float()
        maxx = c_int()
        maxy = c_int()
        mintmp = c_float()
        minx = c_int()
        miny = c_int()
        centertmp = c_float()
        tmparr = np.zeros(3, dtype=np.float32)
        alltmp = np.zeros(256 * 192, dtype=np.float32)  # 256 * 192
        XthermDll.GetTmpData(0,
                             self.bufferedRawFrame.ctypes.data_as(
                                 POINTER(c_byte)), byref(maxtmp), byref(maxx),
                             byref(maxy), byref(mintmp), byref(minx),
                             byref(miny), byref(centertmp),
                             tmparr.ctypes.data_as(POINTER(c_float)),
                             alltmp.ctypes.data_as(POINTER(c_float)))

        '''
            Script to grab average temperature of Hexagon camou-pixel and its surrounding
        '''
        temp_data = alltmp.reshape(192, 256)
        img = temp_data


        if fl == 0 and flg == 2:
            # Creating Gaussain filter Kernal matrix
            h = self.matlab_style_gauss2D()

            # initialization of frame for storing filtered frame
            im = np.zeros((192, 256))

            # Apply filter to the frame
            im = cv2.filter2D(img, -1, h)
            IM = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('Thermal image', IM)

            # Acquire the minimum temperature of the frame
            tp_min = np.min(im)
            # Acquire the miaximum temperature of the frame
            tp_max = np.max(im)

            # Normalization of thermal data to GRAY scale values (0-255)
            Norm_image = (im - tp_min) * ((255 - 0) / (tp_max - tp_min)) + 0

            # Round off the frame values
            round_off = np.round_(Norm_image)

            # Thresholding the frame for acquiring only the corner locations of CTGP
            thresh = cv2.threshold(round_off, 150, 255, cv2.THRESH_BINARY)[1]
            THRESH = cv2.rotate(thresh, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('Threshold Frame', THRESH)

            # Enhancing the corner points for acquiring the corner coordinates
            thresh_dilate = cv2.dilate(thresh, None, iterations=5)
            THRESH_DILATE = cv2.rotate(thresh_dilate, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('DILATED THRESHOLD', THRESH_DILATE)

            # Converting FRAME TYPE (from FLOAT64 to UINT8)
            thresh_u8 = thresh_dilate.astype(np.uint8)
            THRESH_U8 = cv2.rotate(thresh_u8, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('DILATED THRESHOLD (gray scalE)', THRESH_U8)

            # Determining the no. of contours in the frame
            contours = cv2.findContours(thresh_u8.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            print('No. of contours', contours)
            print('Contour points', len(contours))
            print(np.info(contours))

            i = 0
            corner_coord = []
            rows, cols = len(round_off), len(round_off[0])
            IM = np.zeros((rows, cols, 3))
            IM1 = np.zeros((rows, cols, 3))
            IM2 = np.zeros((rows, cols, 3))

            # Calculation of the central coordinates of the corners
            for c in contours:
                # calculate moments for each contour
                M = cv2.moments(c)

                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Mark central coordinates of corners with REd color
                cv2.circle(IM, (cX, cY), 1, (255), cv2.FILLED)

                # Storing the corner corner coordinates
                corner_coord.append((cX, cY))
                i = i + 1

            ## CREATING EDGES OF CTGP by connecting the corner points

            # -------------------------- EDGE 1 --------------------------- #
            cv2.line(IM, (corner_coord[0][0], corner_coord[0][1]), (corner_coord[1][0], corner_coord[1][1]),
                     (255, 255, 255), thickness=1)

            # EDGE 1 distance
            dist1 = math.dist(corner_coord[0], corner_coord[1])
            dist_mid1 = dist1 // 2
            # print("DIST 1:", dist1, 'Half dist:', dist_mid1)

            # finding the centroid coordinate of EDGE 1
            center_x1, center_y1 = ((corner_coord[0][0] + corner_coord[1][0]) // 2), (
                        (corner_coord[0][1] + corner_coord[1][1]) // 2)
            # print("Center coordinate 1:", center_x1, center_y1)
            cv2.circle(IM, (center_x1, center_y1), 2, (0, 255, 0), thickness=1)
            cv2.imshow("Edge1", IM)

            # -------------------------- EDGE 2 --------------------------- #
            cv2.line(IM, (corner_coord[1][0], corner_coord[1][1]), (corner_coord[3][0], corner_coord[3][1]),
                     (255, 255, 255), thickness=1)

            # EDGE 2 distance
            dist2 = math.dist(corner_coord[1], corner_coord[3])
            dist_mid2 = dist2 // 2
            # print("DIST 2:", dist2, 'Half dist:', dist_mid2)

            # finding the centroid coordinate of EDGE 2
            center_x2, center_y2 = ((corner_coord[1][0] + corner_coord[3][0]) // 2), (
                        (corner_coord[1][1] + corner_coord[3][1]) // 2)
            # print("Center coordinate 2:", center_x2, center_y2)
            cv2.circle(IM, (center_x2, center_y2), 2, (0, 255, 0), thickness=1)
            cv2.imshow("Edge2", IM)

            # -------------------------- EDGE 3 --------------------------- #
            cv2.line(IM, (corner_coord[3][0], corner_coord[3][1]), (corner_coord[5][0], corner_coord[5][1]),
                     (255, 255, 255), thickness=1)

            # EDGE 3 distance
            dist3 = math.dist(corner_coord[3], corner_coord[5])
            dist_mid3 = dist3 // 2
            # print("DIST 3:", dist3, 'Half dist:', dist_mid3)

            # finding the centroid coordinate of EDGE 3
            center_x3, center_y3 = ((corner_coord[3][0] + corner_coord[5][0]) // 2), (
                        (corner_coord[3][1] + corner_coord[5][1]) // 2)
            # print("Center coordinate 3:", center_x3, center_y3)
            cv2.circle(IM, (center_x3, center_y3), 2, (0, 255, 0), thickness=1)
            cv2.imshow("Edge3", IM)

            # -------------------------- EDGE 4 --------------------------- #
            cv2.line(IM, (corner_coord[5][0], corner_coord[5][1]), (corner_coord[4][0], corner_coord[4][1]),
                     (255, 255, 255), thickness=1)

            # EDGE 4 distance
            dist4 = math.dist(corner_coord[5], corner_coord[4])
            dist_mid4 = dist4 // 2
            # print("DIST 4:", dist4, 'Half dist:', dist_mid4)

            # finding the centroid coordinate of EDGE 4
            center_x4, center_y4 = ((corner_coord[5][0] + corner_coord[4][0]) // 2), (
                        (corner_coord[5][1] + corner_coord[4][1]) // 2)
            # print("Center coordinate 4:", center_x4, center_y4)
            cv2.circle(IM, (center_x4, center_y4), 2, (0, 255, 0), thickness=1)
            cv2.imshow("Edge4", IM)

            # -------------------------- EDGE 5 --------------------------- #
            cv2.line(IM, (corner_coord[4][0], corner_coord[4][1]), (corner_coord[2][0], corner_coord[2][1]),
                     (255, 255, 255), thickness=1)

            # EDGE 5 distance
            dist5 = math.dist(corner_coord[4], corner_coord[2])
            dist_mid5 = dist5 // 2
            # print("DIST 5:", dist5, 'Half dist:', dist_mid5)

            # finding the centroid coordinate of EDGE 5
            center_x5, center_y5 = ((corner_coord[4][0] + corner_coord[2][0]) // 2), (
                        (corner_coord[4][1] + corner_coord[2][1]) // 2)
            # print("Center coordinate 5:", center_x5, center_y5)
            cv2.circle(IM, (center_x5, center_y5), 2, (0, 255, 0), thickness=1)
            cv2.imshow("Edge5", IM)

            # -------------------------- EDGE 6 --------------------------- #
            cv2.line(IM, (corner_coord[2][0], corner_coord[2][1]), (corner_coord[0][0], corner_coord[0][1]),
                     (255, 255, 255), thickness=1)

            # EDGE 6 distance
            dist6 = math.dist(corner_coord[2], corner_coord[0])
            dist_mid6 = dist6 // 2
            # print("DIST 6:", dist6, 'Half dist:', dist_mid6)

            # finding the centroid coordinate of EDGE 6
            center_x6, center_y6 = ((corner_coord[2][0] + corner_coord[0][0]) // 2), (
                        (corner_coord[2][1] + corner_coord[0][1]) // 2)
            # print("Center coordinate 6:", center_x6, center_y6)
            cv2.circle(IM, (center_x6, center_y6), 2, (0, 255, 0), thickness=1)
            cv2.imshow("Edge6", IM)

            # DIVIDING the entire CTGP into 6 regions
            cv2.line(IM, (center_x1, center_y1), (center_x4, center_y4),
                     (255, 255, 0), thickness=1)
            cv2.imshow("Line1", IM)

            cv2.line(IM, (center_x2, center_y2), (center_x5, center_y5),
                     (255, 255, 0), thickness=1)
            cv2.imshow("Line2", IM)

            cv2.line(IM, (center_x3, center_y3), (center_x6, center_y6),
                     (255, 255, 0), thickness=1)
            cv2.imshow("Line3", IM)

            # finding CTGP centroid 1 from the two opposite centroid coordinate of Edges
            center_x0_1, center_y0_1 = ((center_x1 + center_x4) // 2), ((center_y1 + center_y4) // 2)
            center_x0_2, center_y0_2 = ((center_x2 + center_x5) // 2), ((center_y2 + center_y5) // 2)
            center_x0_3, center_y0_3 = ((center_x3 + center_x6) // 2), ((center_y3 + center_y6) // 2)

            # finding the mean centroid coordinates
            center_x0, center_y0 = ((center_x0_1 + center_x0_2 + center_x0_3) // 3), (
                        (center_y0_1 + center_y0_2 + center_y0_3) // 3)

            # Mark the centroid coordinate of CTGP
            cv2.circle(IM, (center_x0, center_y0), 2, (0, 255, 0), thickness=1)

            # Rotate the CTGP selection frame
            IM_ROTATED = cv2.rotate(IM, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('CTGP selection', IM_ROTATED)

            # ________________________________________________________________________
            # Extracting the coordinates of the entire CTGP region
            # IM1 = IM
            center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), corner_coord),
                               [len(corner_coord)] * 2))
            ss = sorted(corner_coord, key=lambda coord: (-135 - math.degrees(
                math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)

            hex_vertices = np.array(ss)
            hex_vertices = hex_vertices.reshape(-1, 1, 2)
            print("corner coord:", ss)
            print("hex vertices", hex_vertices)
            cv2.fillPoly(IM1, pts=[hex_vertices], color=(.30, .20, .10))

            cv2.imshow('CTGP selection coordinated', IM1)

            CTGP_region_X, CTGP_region_Y = np.where(np.all(IM1 == [.30, .20, .10], axis=2))


            # Drawing TECs selections inside the CTGP sub-regions for acquiring the SP for PID controller
            cv2.line(IM, (corner_coord[0][0], corner_coord[0][1]), (corner_coord[5][0], corner_coord[5][1]),
                     (255, 255, 255), thickness=1)
            cv2.line(IM, (corner_coord[1][0], corner_coord[1][1]), (corner_coord[4][0], corner_coord[4][1]),
                     (255, 255, 255), thickness=1)
            cv2.line(IM, (corner_coord[2][0], corner_coord[2][1]), (corner_coord[3][0], corner_coord[3][1]),
                     (255, 255, 255), thickness=1)

            TEC_region1_mid_X, TEC_region1_mid_Y = ((corner_coord[5][0] + center_x0) // 2), (
                        (corner_coord[5][1] + center_y0) // 2) - 7  # 0, -7
            rot_rectangle1 = ((TEC_region1_mid_X, TEC_region1_mid_Y), (13, 13), -135)
            box1 = cv2.boxPoints(rot_rectangle1)
            box1 = np.int0(box1)  # Convert into integer values
            # print('TEC1 coordinate: ', box1)
            cv2.drawContours(IM, [box1], 0, (255, 255, 255), 1)

            TEC_region2_mid_X, TEC_region2_mid_Y = ((corner_coord[0][0] + center_x0) // 2), (
                        (corner_coord[0][1] + center_y0) // 2) + 7  # 0, +7
            rot_rectangle2 = ((TEC_region2_mid_X, TEC_region2_mid_Y), (13, 13), -135)
            box2 = cv2.boxPoints(rot_rectangle2)
            box2 = np.int0(box2)  # Convert into integer values
            ##            print('TEC2 coordinate: ', box2)
            cv2.drawContours(IM, [box2], 0, (255, 255, 255), 1)

            TEC_region3_mid_X, TEC_region3_mid_Y = ((corner_coord[4][0] + center_x0) // 2) - 2, (
                        (corner_coord[4][1] + center_y0) // 2) - 2  # +8, -3
            rot_rectangle3 = ((TEC_region3_mid_X, TEC_region3_mid_Y), (13, 13), -10)  # 14
            box3 = cv2.boxPoints(rot_rectangle3)
            box3 = np.int0(box3)  # Convert into integer values
            ##            print('TEC3 coordinate: ', box3)
            cv2.drawContours(IM, [box3], 0, (255, 255, 255), 1)

            TEC_region4_mid_X, TEC_region4_mid_Y = ((corner_coord[1][0] + center_x0) // 2) + 7, (
                        (corner_coord[1][1] + center_y0) // 2) + 3  # -6, +6
            rot_rectangle4 = ((TEC_region4_mid_X, TEC_region4_mid_Y), (13, 13), -10)
            box4 = cv2.boxPoints(rot_rectangle4)
            box4 = np.int0(box4)  # Convert into integer values
            ##            print('TEC4 coordinate: ', box4)
            cv2.drawContours(IM, [box4], 0, (255, 255, 255), 1)

            TEC_region5_mid_X, TEC_region5_mid_Y = ((corner_coord[3][0] + center_x0) // 2) + 7, (
                        (corner_coord[3][1] + center_y0) // 2) - 3  # -6, -2
            rot_rectangle5 = ((TEC_region5_mid_X, TEC_region5_mid_Y), (13, 13), 15)
            box5 = cv2.boxPoints(rot_rectangle5)
            box5 = np.int0(box5)  # Convert into integer values
            ##            print('TEC5 coordinate: ', box5)
            cv2.drawContours(IM, [box5], 0, (255, 255, 255), 1)

            TEC_region6_mid_X, TEC_region6_mid_Y = ((corner_coord[2][0] + center_x0) // 2) - 4, (
                        (corner_coord[2][1] + center_y0) // 2) + 3  # +6, +3
            rot_rectangle6 = ((TEC_region6_mid_X, TEC_region6_mid_Y), (13, 13), 15)
            box6 = cv2.boxPoints(rot_rectangle6)
            box6 = np.int0(box6)  # Convert into integer values
            ##            print('TEC6 coordinate: ', box6)
            cv2.drawContours(IM, [box6], 0, (255, 255, 255), 1)

            # Rotate the CTGP selection frame
            IM_ROTATED1 = cv2.rotate(IM, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('CTGP SUB-REGIONS for PID-SP', IM_ROTATED1)

            cv2.fillPoly(IM, pts=[box1], color=(.70, .20, .20))
            cv2.fillPoly(IM, pts=[box2], color=(0, 0, 255))
            cv2.fillPoly(IM, pts=[box3], color=(.55, .90, .55))
            cv2.fillPoly(IM, pts=[box4], color=(.55, .255, .55))
            cv2.fillPoly(IM, pts=[box5], color=(255, 255, 0))
            cv2.fillPoly(IM, pts=[box6], color=(0, 255, 255))

            # Rotate the CTGP selection frame
            IM_ROTATED2 = cv2.rotate(IM, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('CTGP FILLED SUB-REGIONS for PID-SP', IM_ROTATED2)

            ## Grabing coordinates of CTGP regions

            # TEC1 coordinates
            region1_X_coord, region1_Y_coord = np.where(np.all(IM == [0, 0, 255], axis=2))
            # print("Region 1:", region1_X_coord)
            ##cv2.circle(IM, (region1_Y_coord[100], region1_X_coord[100]), 5, (255, 255, 255), thickness=3)

            # TEC2 coordinates
            region2_X_coord, region2_Y_coord = np.where(np.all(IM == [0, 255, 255], axis=2))
            # print("Region 1:", region1_X_coord)
            ##cv2.circle(IM, (region2_Y_coord[100], region2_X_coord[100]), 5, (255, 255, 255), thickness=3)

            # TEC3 coordinates
            region3_X_coord, region3_Y_coord = np.where(np.all(IM == [.55, .90, .55], axis=2))
            # print("Region 1:", region1_X_coord)
            ##cv2.circle(IM, (region3_Y_coord[100], region3_X_coord[100]), 5, (255, 255, 255), thickness=3)

            # TEC4 coordinates
            region4_X_coord, region4_Y_coord = np.where(np.all(IM == [.70, .20, .20], axis=2))
            # print("Region 1:", region1_X_coord)
            ##cv2.circle(IM, (region4_Y_coord[100], region4_X_coord[100]), 5, (255, 255, 255), thickness=3)

            # TEC5 coordinates
            region5_X_coord, region5_Y_coord = np.where(np.all(IM == [255, 255, 0], axis=2))
            # print("Region 1:", region1_X_coord)
            ##cv2.circle(IM, (region5_Y_coord[100], region5_X_coord[100]), 5, (255, 255, 255), thickness=3)

            # TEC6 coordinates
            region6_X_coord, region6_Y_coord = np.where(np.all(IM == [.55, .255, .55], axis=2))
            # print("Region 1:", region1_X_coord)
            ##cv2.circle(IM, (region6_Y_coord[100], region6_X_coord[100]), 5, (255, 255, 255), thickness=3)

            ## Calculating TECs REGIONs temperatures

            # TEC1 average temperature
            cc1 = len(region1_X_coord)
            temps1 = [0] * cc1
            for i in range(0, cc1):
                temps1[i] = img[int(region1_X_coord[i]), int(region1_Y_coord[i])]
            tmp_max1 = max(temps1)
            tmp_avg1 = sum(temps1) / len(temps1)
            # print("Region1 TEMP:", tmp_avg1)

            # TEC2 average temperature
            cc2 = len(region2_X_coord)
            temps2 = [0] * cc2
            for i in range(0, cc2):
                temps2[i] = img[int(region2_X_coord[i]), int(region2_Y_coord[i])]
            tmp_max2 = max(temps2)
            tmp_avg2 = sum(temps2) / len(temps2)
            # print("Region2 TEMP:", tmp_avg2)

            # TEC3 average temperature
            cc3 = len(region3_X_coord)
            temps3 = [0] * cc3
            for i in range(0, cc3):
                temps3[i] = img[int(region3_X_coord[i]), int(region3_Y_coord[i])]
            tmp_max3 = max(temps3)
            tmp_avg3 = sum(temps3) / len(temps3)
            # print("Region3 TEMP:", tmp_avg3)

            # TEC4 average temperature
            cc4 = len(region4_X_coord)
            temps4 = [0] * cc4
            for i in range(0, cc4):
                temps4[i] = img[int(region4_X_coord[i]), int(region4_Y_coord[i])]
            tmp_max4 = max(temps4)
            tmp_avg4 = sum(temps4) / len(temps4)
            # print("Region4 TEMP:", tmp_avg4)

            # TEC5 average temperature
            cc5 = len(region5_X_coord)
            temps5 = [0] * cc5
            for i in range(0, cc5):
                temps5[i] = img[int(region5_X_coord[i]), int(region5_Y_coord[i])]
            tmp_max5 = max(temps5)
            tmp_avg5 = sum(temps5) / len(temps5)
            # print("Region5 TEMP:", tmp_avg5)

            # TEC6 average temperature
            cc6 = len(region6_X_coord)
            temps6 = [0] * cc6
            for i in range(0, cc6):
                temps6[i] = img[int(region6_X_coord[i]), int(region6_Y_coord[i])]
            tmp_max6 = max(temps6)
            tmp_avg6 = sum(temps6) / len(temps6)
            # print("Region6 TEMP:", tmp_avg6)

            IM_coord = cv2.rotate(IM, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('CTGP selection REGIONs', IM_coord)

            if len(contours) == 6:
                fl = 1
                flg = 3
            else:
                fl = 0
                flg = 2
            img = np.transpose(img)
            outputFrame = img

        elif fl == 1 and flg == 3:

            # CTGP REGION 1 average temperature
            cc1 = len(region1_X_coord)
            temps1 = [0] * cc1
            for i in range(0, cc1):
                temps1[i] = img[int(region1_X_coord[i]), int(region1_Y_coord[i])]
            tmp_max1 = max(temps1)
            tmp_avg1 = sum(temps1) / len(temps1)
            # print("Region1 TEMP:", tmp_avg1)

            # CTGP REGION 2 average temperature
            cc2 = len(region2_X_coord)
            temps2 = [0] * cc2
            for i in range(0, cc2):
                temps2[i] = img[int(region2_X_coord[i]), int(region2_Y_coord[i])]
            tmp_max2 = max(temps2)
            tmp_avg2 = sum(temps2) / len(temps2)
            # print("Region2 TEMP:", tmp_avg2)

            # CTGP REGION 3 average temperature
            cc3 = len(region3_X_coord)
            temps3 = [0] * cc3
            for i in range(0, cc3):
                temps3[i] = img[int(region3_X_coord[i]), int(region3_Y_coord[i])]
            tmp_max3 = max(temps3)
            tmp_avg3 = sum(temps3) / len(temps3)
            # print("Region3 TEMP:", tmp_avg3)

            # CTGP REGION 4 average temperature
            cc4 = len(region4_X_coord)
            temps4 = [0] * cc4
            for i in range(0, cc4):
                temps4[i] = img[int(region4_X_coord[i]), int(region4_Y_coord[i])]
            tmp_max4 = max(temps4)
            tmp_avg4 = sum(temps4) / len(temps4)
            # print("Region4 TEMP:", tmp_avg4)

            # CTGP REGION 5 average temperature
            cc5 = len(region5_X_coord)
            temps5 = [0] * cc5
            for i in range(0, cc5):
                temps5[i] = img[int(region5_X_coord[i]), int(region5_Y_coord[i])]
            tmp_max5 = max(temps5)
            tmp_avg5 = sum(temps5) / len(temps5)
            # print("Region5 TEMP:", tmp_avg5)

            # CTGP REGION 6 average temperature
            cc6 = len(region6_X_coord)
            temps6 = [0] * cc6
            for i in range(0, cc6):
                temps6[i] = img[int(region6_X_coord[i]), int(region6_Y_coord[i])]
            tmp_max6 = max(temps6)
            tmp_avg6 = sum(temps6) / len(temps6)
            # print("Region6 TEMP:", tmp_avg6)

            #TEC1
            CTGP_region4 = "{:.2f}".format(tmp_avg4)
            CTGP_SURR_region6 = "{:.2f}".format(TEC1_SP[ATC2_px_loc])
            #TEC2
            CTGP_region5 = "{:.2f}".format(tmp_avg5)
            CTGP_SURR_region1 = "{:.2f}".format(TEC2_SP[ATC2_px_loc])
            #TEC3
            CTGP_region6 = "{:.2f}".format(tmp_avg6)
            CTGP_SURR_region3 = "{:.2f}".format(TEC3_SP[ATC2_px_loc])
            #TEC4
            CTGP_region1 = "{:.2f}".format(tmp_avg1)
            CTGP_SURR_region4 = "{:.2f}".format(TEC4_SP[ATC2_px_loc])
            #TEC5
            CTGP_region2 = "{:.2f}".format(tmp_avg5)
            CTGP_SURR_region5 = "{:.2f}".format(TEC5_SP[ATC2_px_loc])
            #TEC6
            CTGP_region3 = "{:.2f}".format(tmp_avg2)
            CTGP_SURR_region2 = "{:.2f}".format(TEC6_SP[ATC2_px_loc])

            # CTGP_region1 = "{:.2f}".format(tmp_avg1)
            # CTGP_SURR_region1 = "{:.2f}".format(TEC1_SP[ATC2_px_loc])
            #
            # CTGP_region2 = "{:.2f}".format(tmp_avg2)
            # CTGP_SURR_region2 = "{:.2f}".format(TEC2_SP[ATC2_px_loc])
            #
            # CTGP_region3 = "{:.2f}".format(tmp_avg3)
            # CTGP_SURR_region3 = "{:.2f}".format(TEC3_SP[ATC2_px_loc])
            #
            # CTGP_region4 = "{:.2f}".format(tmp_avg4)
            # CTGP_SURR_region4 = "{:.2f}".format(TEC4_SP[ATC2_px_loc])
            #
            # CTGP_region5 = "{:.2f}".format(tmp_avg5)
            # CTGP_SURR_region5 = "{:.2f}".format(TEC5_SP[ATC2_px_loc])
            #
            # CTGP_region6 = "{:.2f}".format(tmp_avg6)
            # CTGP_SURR_region6 = "{:.2f}".format(TEC6_SP[ATC2_px_loc])

            if prnt_temps_flg == 1:
                print("PID Manipulating variables Loaded")
                print("------------------------------------------------------------------------------------------")
                TECs_temps = "TEC1: " + str(TEC1_SP[ATC2_px_loc]) + "     " + "TEC2: " + str(TEC2_SP[ATC2_px_loc]) + "     " + \
                             "TEC3: " + str(TEC3_SP[ATC2_px_loc]) + "     " + "TEC4: " + str(TEC4_SP[ATC2_px_loc]) + "     " + \
                             "TEC5: " + str(TEC5_SP[ATC2_px_loc]) + "     " + "TEC6: " + str(TEC6_SP[ATC2_px_loc])
                print(TECs_temps)
                print("------------------------------------------------------------------------------------------")

                print("================================================================================================")
                prnt_temps_flg = 0

            # Timing duration after which the next set point from the database is loaded
            curr_time = datetime.now()
            if (curr_time.hour - prev_time.hour) == 0:
                if (curr_time.minute - prev_time.minute) == 2:
                    prev_time = curr_time
                    # raise flags to store the current thermal image and load the next SPs for PID
                    load_flag = 1
                    self.load_SPs.emit(load_flag)
            elif (curr_time.hour - prev_time.hour) == 1:
                if ((60 - prev_time.minute) + curr_time.minute) == 2:
                    prev_time = curr_time
                    load_flag = 1
                    self.load_SPs.emit(load_flag)
                
                
            if P == 1:
                t_avg1 = tmp_avg4
                s_temp_avg1 = TEC1_SP[ATC2_px_loc]
                self.plot_signal.emit(t_avg1, s_temp_avg1, count)
                #
                t_avg2 = tmp_avg5
                s_temp_avg2 = TEC2_SP[ATC2_px_loc]
                self.plot_signal2.emit(t_avg2, s_temp_avg2, count)
                #
                t_avg3 = tmp_avg6
                s_temp_avg3 = TEC3_SP[ATC2_px_loc]
                self.plot_signal3.emit(t_avg3, s_temp_avg3, count)

            if P == 2:
                t_avg4 = tmp_avg1
                s_temp_avg4 = TEC4_SP[ATC2_px_loc]
                self.plot_signal4.emit(t_avg4, s_temp_avg4, count)
                #
                t_avg5 = tmp_avg2
                s_temp_avg5 = TEC5_SP[ATC2_px_loc]
                self.plot_signal5.emit(t_avg5, s_temp_avg5, count)

                t_avg6 = tmp_avg3
                s_temp_avg6 = TEC6_SP[ATC2_px_loc]
                self.plot_signal6.emit(t_avg6, s_temp_avg6, count)

            # for transmitting PV and SP from PYTHON to ATMEGA
            self.transmit_data.emit(count)

            count += 1

            """
                Code for verification of serial data transmitted by Python is received
                successfully by micro-controller.

                But with these lines added to the script the GUI display hangs-up
                because of the [readline function of serial port], so i have commented
                these lines
            """

        '''
        End of script for grabbing average temperatire of camou-pixel and surrounding
        '''

        img = np.transpose(img)
        outputFrame = img

        # display image on GUI
        outputFrame = cv2.normalize(
            outputFrame,
            dst=None,
            alpha=0,
            beta=65535,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_16U)
        cv2.imshow('OUTPUT frame', outputFrame)

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

        if self.flip_x and self.flip_y:
            outputFrame = cv2.flip(outputFrame, -1)
        elif self.flip_x:
            outputFrame = cv2.flip(outputFrame, 1)
        elif self.flip_y:
            outputFrame = cv2.flip(outputFrame, 0)

        resizedImg = cv2.resize(outputFrame, (self.resx, self.resy - self.infobarY), resizeAlgo)
        # print('resized = ', len(resizedImg), len(resizedImg[0]))
        use_8bits = force_use_8bits or (colormap != -1) or (self.video_writer is not None)

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
            self.bufferedImg = np.zeros(
                (self.resy, self.resx), dtype=np.uint16)
            cv2.putText(
                self.bufferedImg,
                'FPAT:{:.2f} CORT:{:.2f} FPAA:{:5d} ORGA:{:5d}'.format(
                    fpatmp.value, coretmp.value, fpaavg.value, orgavg.value) +
                (' 16bits(Disp)'
                 if use_NLMDenoising else ' 16bits') + ('C' if use_clahe else '') +
                ('X' if self.flip_x else '') + ('Y' if self.flip_y else '') +
                ('H' if use_equHist else '') + ('N' if use_canny else '') +
                ('T' if (tempMode or rawMode) else '') + ('B' if use_bf else '') +
                ('S' if use_sharpening else '') + ('R' if use_deStrip else ''),
                (4, self.resy - self.infobarY + round(8 * self.infobarY / 30)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35 * self.infobarY / 30, 65535, 1)
            cv2.putText(
                self.bufferedImg,
                'MAXT:{:.2f} {:3d},{:3d}'
                .format(maxtmp.value, maxx.value, maxy.value),
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
    def matlab_style_gauss2D(self, shape=(50, 50), sigma=1):
        m, n = [(ss - 1.) / 2. for ss in shape]
        y, x = np.ogrid[-m:m + 1, -n:n + 1]
        h = np.exp(-(x * x + y * y) / (2. * sigma * sigma))
        h[h < np.finfo(h.dtype).eps * h.max()] = 0
        sumh = h.sum()
        if sumh != 0:
            h /= sumh
        return h

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
        global img, img2_blended
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
        alltmp = np.zeros(256 * 192, dtype=np.float32)  # 256 * 192
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
            img,
            delimiter=",",
            fmt='%.2f')  # 192, 256
        print('Done')

    def dump_rawdata_png(self):
        global saveFormat
        cv2.imwrite(
            "RAWD-" +
            self.FrameTimeStamp.strftime('%Y-%m-%dT%H-%M-%S') +
            ('-%02d' % (self.FrameTimeStamp.microsecond / 10000)) +
            saveFormat, self.bufferedRawFrame[0:192, :])  # [0:192, :]
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
            self.video_writer = cv2.VideoWriter(filename, fourcc, 25, (self.resx, self.resy), isColor=(colormap != -1))
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
        self.initUI()  # not sure what this line of code does
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        # add plot toolbar from matplotlib
        self.toolbar = NavigationToolbar(self.canvas, self)

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        m = PlotCanvas(self, width=7, height=6)
        m.move(0, 30)
        print('ploted')


class PlotCanvas(FigureCanvas):
    global t_avg1, cnt

    def __init__(self, parent=None, width=4, height=3, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        ##        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        self.hour = list(range(100))
        self.temperature = [randint(0, 100) for _ in range(100)]

        ##        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding,QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        self.plot()
    def plot(self):
        rplt_x.append(rplt_x[-1] + 1)
        plate_temp = "{:.2f}".format(t_avg1)
        rplt_y.append(plate_temp)
        print(rplt_y)
        l = len(rplt_y)
        print(rplt_y[l - 1])

        ax = self.figure.add_subplot(111)
        ax.plot(rplt_y)
        ax.set_ylim(ymin=0, ymax=50)
        self.draw()


class MainWidget(QtWidgets.QWidget):
    rec_status_signal = QtCore.pyqtSignal(str)

    def update_plot1(self):
        global DS18b20, t_avg1, s_temp_avg1, count
        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.
        
        self.y = self.y[1:]  # Remove the first
        self.y.append(t_avg1)  # Add a new random value.
        self.data_line.setData(self.x, self.y)  # Update the data#

        self.yy = self.yy[1:]  # Remove the first
        self.yy.append(s_temp_avg1)  # Add a new random value.
        self.data_line1.setData(self.x, self.yy)

    def update_plot2(self):
        global DS18b20, t_avg2, s_temp_avg2, count
        self.x1 = self.x1[1:]  # Remove the first y element.
        self.x1.append(self.x1[-1] + 1)  # Add a new value 1 higher than the last.

        self.y1 = self.y1[1:]  # Remove the first
        self.y1.append(t_avg2)  # Add a new random value.
        self.data_line11.setData(self.x1, self.y1)  # Update the data#

        self.yy1 = self.yy1[1:]  # Remove the first
        self.yy1.append(s_temp_avg2)  # Add a new random value.
        self.data_line111.setData(self.x1, self.yy1)

    def update_plot3(self):
        global DS18b20, t_avg3, s_temp_avg3, count
        self.x2 = self.x2[1:]  # Remove the first y element.
        self.x2.append(self.x2[-1] + 1)  # Add a new value 1 higher than the last.

        self.y2 = self.y2[1:]  # Remove the first
        self.y2.append(t_avg3)  # Add a new random value.
        self.data_line22.setData(self.x2, self.y2)  # Update the data#

        self.yy2 = self.yy2[1:]  # Remove the first
        self.yy2.append(s_temp_avg3)  # Add a new random value.
        self.data_line222.setData(self.x2, self.yy2)

    def update_plot4(self):
        global DS18b20, t_avg4, s_temp_avg4, count
        self.x3 = self.x3[1:]  # Remove the first y element.
        self.x3.append(self.x3[-1] + 1)  # Add a new value 1 higher than the last.

        self.y3 = self.y3[1:]  # Remove the first
        self.y3.append(t_avg4)  # Add a new random value.
        self.data_line33.setData(self.x3, self.y3)  # Update the data#

        self.yy3 = self.yy3[1:]  # Remove the first
        self.yy3.append(s_temp_avg4)  # Add a new random value.
        self.data_line333.setData(self.x3, self.yy3)

    def update_plot5(self):
        global DS18b20, t_avg5, s_temp_avg5, count
        self.x4 = self.x4[1:]  # Remove the first y element.
        self.x4.append(self.x4[-1] + 1)  # Add a new value 1 higher than the last.

        self.y4 = self.y4[1:]  # Remove the first
        self.y4.append(t_avg5)  # Add a new random value.
        self.data_line44.setData(self.x4, self.y4)  # Update the data#

        self.yy4 = self.yy4[1:]  # Remove the first
        self.yy4.append(s_temp_avg5)  # Add a new random value.
        self.data_line444.setData(self.x4, self.yy4)

    def update_plot6(self):
        global DS18b20, t_avg6, s_temp_avg6, count
        self.x5 = self.x5[1:]  # Remove the first y element.
        self.x5.append(self.x5[-1] + 1)  # Add a new value 1 higher than the last.

        self.y5 = self.y5[1:]  # Remove the first
        self.y5.append(t_avg6)  # Add a new random value.
        self.data_line55.setData(self.x5, self.y5)  # Update the data#

        self.yy5 = self.yy5[1:]  # Remove the first
        self.yy5.append(s_temp_avg6)  # Add a new random value.
        self.data_line555.setData(self.x5, self.yy5)

    def im_offsety22(self):
        global load_flag, ATC2_px_loc, img, prnt_temps_flg
        # Creating a directory for storing CTGPs regions
        dir_name = "TBGS frame 1"
        if ATC2_px_loc == 1:
            try:
                os.makedirs(dir_name)
                print("Directory created for TBGS frame")
            except FileExistsError:
                print("Directory already exist")
                pass
        # convert array into dataframe
        DF = pd.DataFrame(img)
        # save the dataframe as a csv file
        ST = dir_name + "/" + "ATC2_location " + str(ATC2_px_loc) + ".csv"
        DF.to_csv(ST)
        print("Saved blended frame...")
        # shift to next set of TECs SP 
        ATC2_px_loc = ATC2_px_loc + 1
        print("================================================================================================")
        print("Blending of ATC px 2 on location " + str(ATC2_px_loc) + " is under progress...")
        prnt_temps_flg = 1
        load_flag = 0
        if ATC2_px_loc == 250:
            print("**************************")
            print("Cv90 Blending completed...")
            print("**************************")
            exitNow()


    def Serial_dat(self):
        global CTGP_SURR_region3, CTGP_region4, CTGP_SURR_region2, CTGP_region5, CTGP_SURR_region4, CTGP_region3, CTGP_SURR_region5, CTGP_region2, CTGP_SURR_region6, CTGP_region1, CTGP_SURR_region1, CTGP_region6, count
        B = str(CTGP_SURR_region1) + 'a' + str(CTGP_region5) + 'b'
        TRANSMIT_vals = B.encode('ascii')
        serialport.write(TRANSMIT_vals)
        serialport.flush()

        B = str(CTGP_SURR_region3) + 'c' + str(CTGP_region6) + 'd'
        TRANSMIT_vals = B.encode('ascii')
        serialport.write(TRANSMIT_vals)
        serialport.flush()

        B = str(CTGP_SURR_region4) + 'e' + str(CTGP_region1) + 'f'
        TRANSMIT_vals = B.encode('ascii')
        serialport.write(TRANSMIT_vals)
        serialport.flush()

        B = str(CTGP_SURR_region5) + 'g' + str(CTGP_region2) + 'h'
        TRANSMIT_vals = B.encode('ascii')
        serialport.write(TRANSMIT_vals)
        serialport.flush()

        B = str(CTGP_SURR_region2) + 'i' + str(CTGP_region3) + 'j'
        TRANSMIT_vals = B.encode('ascii')
        serialport.write(TRANSMIT_vals)
        serialport.flush()

        B = str(CTGP_SURR_region6) + 'k' + str(CTGP_region4) + 'l'
        TRANSMIT_vals = B.encode('ascii')
        serialport.write(TRANSMIT_vals)
        serialport.flush()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.thermal_display_widget = ThermalDisplayWidget(rec_status_signal=self.rec_status_signal)
        self.thermal_video_stream = ThermalVideoStream()

        image_data_slot = self.thermal_display_widget.image_data_slot
        self.thermal_video_stream.image_data_signal.connect(image_data_slot)

        self.rec_status_signal.connect(self.update_recording_label)
        # signal raised to transmit PV and SP to microcontroller (serially)  
        self.thermal_display_widget.transmit_data.connect(self.Serial_dat)
        
        self.thermal_display_widget.load_SPs.connect(self.im_offsety22)

        layoutDown0 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')  # Set background color of plot
        self.graphWidget.setMaximumWidth(350)  # Add width of plot
        self.graphWidget.setMaximumHeight(200)  # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response-1", color="b", size="15pt")
        layoutDown0.addWidget(self.graphWidget)
        self.x = list(range(100))  # 100 time points
        self.y = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yy = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)
        pen1 = pg.mkPen(color=(0, 255, 0))
        self.data_line1 = self.graphWidget.plot(self.x, self.yy, pen=pen1)
        self.thermal_display_widget.plot_signal.connect(self.update_plot1)

        layoutDown01 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')  # Set background color of plot
        self.graphWidget.setMaximumWidth(350)  # Add width of plot
        self.graphWidget.setMaximumHeight(200)  # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response-2", color="b", size="15pt")
        layoutDown01.addWidget(self.graphWidget)
        self.x1 = list(range(100))  # 100 time points
        self.y1 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yy1 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen2 = pg.mkPen(color=(255, 0, 0))
        self.data_line11 = self.graphWidget.plot(self.x1, self.y1, pen=pen2)
        pen3 = pg.mkPen(color=(0, 255, 0))
        self.data_line111 = self.graphWidget.plot(self.x1, self.yy1, pen=pen3)
        self.thermal_display_widget.plot_signal2.connect(self.update_plot2)

        layoutDown02 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')  # Set background color of plot
        self.graphWidget.setMaximumWidth(350)  # Add width of plot
        self.graphWidget.setMaximumHeight(200)  # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response-3", color="b", size="15pt")
        layoutDown02.addWidget(self.graphWidget)
        self.x2 = list(range(100))  # 100 time points

        self.y2 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yy2 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen4 = pg.mkPen(color=(255, 0, 0))
        self.data_line22 = self.graphWidget.plot(self.x2, self.y2, pen=pen4)
        pen5 = pg.mkPen(color=(0, 255, 0))
        self.data_line222 = self.graphWidget.plot(self.x2, self.yy2, pen=pen5)
        self.thermal_display_widget.plot_signal3.connect(self.update_plot3)

        layoutDown03 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')  # Set background color of plot
        self.graphWidget.setMaximumWidth(350)  # Add width of plot
        self.graphWidget.setMaximumHeight(200)  # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response-4", color="b", size="15pt")
        layoutDown03.addWidget(self.graphWidget)
        self.x3 = list(range(100))  # 100 time points
        self.y3 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yy3 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen6 = pg.mkPen(color=(255, 0, 0))
        self.data_line33 = self.graphWidget.plot(self.x3, self.y3, pen=pen6)
        pen7 = pg.mkPen(color=(0, 255, 0))
        self.data_line333 = self.graphWidget.plot(self.x3, self.yy3, pen=pen7)
        self.thermal_display_widget.plot_signal4.connect(self.update_plot4)

        layoutDown04 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')  # Set background color of plot
        # self.graphWidget.setYRange(0, 50)        # set Y-axis range of plot
        self.graphWidget.setMaximumWidth(350)  # Add width of plot
        self.graphWidget.setMaximumHeight(200)  # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response-5", color="b", size="15pt")
        layoutDown04.addWidget(self.graphWidget)
        self.x4 = list(range(100))  # 100 time points
        self.y4 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yy4 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen8 = pg.mkPen(color=(255, 0, 0))
        self.data_line44 = self.graphWidget.plot(self.x4, self.y4, pen=pen8)
        pen9 = pg.mkPen(color=(0, 255, 0))
        self.data_line444 = self.graphWidget.plot(self.x4, self.yy4, pen=pen9)
        self.thermal_display_widget.plot_signal5.connect(self.update_plot5)

        layoutDown05 = QtWidgets.QHBoxLayout()
        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setBackground('w')  # Set background color of plot
        self.graphWidget.setMaximumWidth(350)  # Add width of plot
        self.graphWidget.setMaximumHeight(200)  # Add height of plot
        # Add Axis Labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.graphWidget.setLabel("left", "Temperature (°C)", **styles)
        self.graphWidget.setLabel("bottom", "No. of observations", **styles)
        # show the grids  on the graph
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("PID Response-6", color="b", size="15pt")
        layoutDown05.addWidget(self.graphWidget)
        self.x5 = list(range(100))  # 100 time points
        self.y5 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.yy5 = [randint(0, 1) for _ in range(100)]  # 100 data points
        self.graphWidget.setBackground('w')
        pen10 = pg.mkPen(color=(255, 0, 0))
        self.data_line55 = self.graphWidget.plot(self.x5, self.y5, pen=pen10)
        pen11 = pg.mkPen(color=(0, 255, 0))
        self.data_line555 = self.graphWidget.plot(self.x5, self.yy5, pen=pen11)
        self.thermal_display_widget.plot_signal6.connect(self.update_plot6)

        layoutDown1 = QtWidgets.QVBoxLayout()
        self.run_button = QtWidgets.QPushButton('Start')
        layoutDown1.addWidget(self.run_button)
        self.run_button.clicked.connect(
            self.thermal_video_stream.start_capture)
        self.stop_button = QtWidgets.QPushButton('Stop')
        layoutDown1.addWidget(self.stop_button)
        self.stop_button.clicked.connect(
            self.thermal_video_stream.stop_capture)
        self.sav_img_button = QtWidgets.QPushButton('Save Img')
        layoutDown1.addWidget(self.sav_img_button)
        self.sav_img_button.clicked.connect(
            self.thermal_display_widget.sav_img)

        self.First_three_TECS = QtWidgets.QPushButton('PLot1,2,3')
        layoutDown1.addWidget(self.First_three_TECS)
        self.First_three_TECS.clicked.connect(SHOW_PLT1)

        layoutDown3 = QtWidgets.QVBoxLayout()
        self.save_temperature_button = QtWidgets.QPushButton('Save Temperature')
        layoutDown3.addWidget(self.save_temperature_button)
        self.save_temperature_button.clicked.connect(
            self.thermal_display_widget.save_temperature)
        self.self_cal_button = QtWidgets.QPushButton('Shutter Cal')
        layoutDown3.addWidget(self.self_cal_button)
        self.self_cal_button.clicked.connect(self.thermal_video_stream.selfCal)
        self.edit_par_button = QtWidgets.QPushButton('Meas...')
        layoutDown3.addWidget(self.edit_par_button)
        self.edit_par_button.clicked.connect(self.editPara)
        self.END_three_TECS = QtWidgets.QPushButton('PLot4,5,6')
        layoutDown3.addWidget(self.END_three_TECS)
        self.END_three_TECS.clicked.connect(SHOW_PLT2)

        layoutDown5 = QtWidgets.QVBoxLayout()

        self.pseudoimage22 = QtWidgets.QPushButton('CTGP-II offset')
        layoutDown5.addWidget(self.pseudoimage22)
        self.pseudoimage22.clicked.connect(self.im_offsety22)

        self.exit_button = QtWidgets.QPushButton('EXIT NOW!')
        layoutDown5.addWidget(self.exit_button)
        self.exit_button.clicked.connect(exitNow)

        if QtWidgets.QDesktopWidget().screenGeometry(-1).height() < 1200:
            layoutVert = QtWidgets.QHBoxLayout()
            layoutVert.addLayout(layoutDown1)
            layoutVert.addLayout(layoutDown3)
            layoutVert.addLayout(layoutDown5)

            layoutHORI1 = QtWidgets.QHBoxLayout()
            layoutHORI1.addLayout(layoutDown0)
            layoutHORI1.addLayout(layoutDown01)
            layoutHORI1.addLayout(layoutDown02)

            layoutHORI2 = QtWidgets.QHBoxLayout()
            layoutHORI2.addLayout(layoutDown03)
            layoutHORI2.addLayout(layoutDown04)
            layoutHORI2.addLayout(layoutDown05)

            layoutVert1 = QtWidgets.QVBoxLayout()
            layoutVert1.addLayout(layoutHORI1)
            layoutVert1.addLayout(layoutHORI2)
            layoutVert1.addLayout(layoutVert)

            layoutHori = QtWidgets.QHBoxLayout()
            layoutHori.addLayout(layoutVert1)
            layoutHori.addWidget(self.thermal_display_widget)
            self.setLayout(layoutHori)
        else:
            layoutVert = QtWidgets.QVBoxLayout()
            layoutVert.addWidget(self.thermal_display_widget)
            layoutVert.addLayout(layoutDown1)
            layoutVert.addLayout(layoutDown3)
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

def SHOW_PLT1():
    global P
    P = 1

def SHOW_PLT2():
    global P
    P = 2

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
        # Emiss = c_float()
        # refltmp = c_float()
        # airtemp = c_float()
        # Humi = c_float()
        # Distance = c_ushort()
        # Fix = c_float()
        # XthermDll.GetFixParam(
        #     byref(Emiss.value), byref(refltmp.value), byref(airtemp.value), byref(Humi.value),
        #     byref(Distance.value), byref(Fix.value))

        Emiss = float(0.99)
        refltmp = float(0)
        airtemp = float(18.5)
        Humi = float(0.53)
        Distance = int(1)
        Fix = float(0.1)
        XthermDll.UpdateFixParam(
            c_float(Emiss), c_float(refltmp), c_float(airtemp),
            c_float(Humi), c_ushort(Distance), c_float(Fix))

        # print(
        #     'FixParamFromDll:Emiss:{:.2f},refltmp:{:.2f},airtemp:{:.2f},Humi:{:.2f},Distance:{:d},Fix:{:.2f}'.
        #     format(Emiss.value, refltmp.value, airtemp.value, Humi.value,
        #            Distance.value, Fix.value))

        layoutForm = QtWidgets.QFormLayout()

        self.EmissCntl = QtWidgets.QLineEdit('{:.2f}'.format(Emiss))
        layoutForm.addRow('Emiss', self.EmissCntl)

        self.refltmpCntl = QtWidgets.QLineEdit('{:.2f}'.format(refltmp))
        layoutForm.addRow('refltmp', self.refltmpCntl)

        self.airtempCntl = QtWidgets.QLineEdit('{:.2f}'.format(airtemp))
        layoutForm.addRow('airtemp', self.airtempCntl)

        self.HumiCntl = QtWidgets.QLineEdit('{:.2f}'.format(Humi))
        layoutForm.addRow('Humi', self.HumiCntl)

        self.DistanceCntl = QtWidgets.QLineEdit('{:d}'.format(Distance))
        layoutForm.addRow('Distance', self.DistanceCntl)

        self.FixCntl = QtWidgets.QLineEdit('{:.2f}'.format(Fix))
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
    main_window.setWindowTitle("Real-time ATC")
    main_widget = MainWidget()
    main_window.setCentralWidget(main_widget)
    main_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

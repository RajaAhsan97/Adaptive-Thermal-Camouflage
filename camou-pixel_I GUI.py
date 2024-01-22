"""
    Camou-pixel-I GUI
    This code is for HT-301, T3S & T2L downloaded from EEVblog

    For T2L (replace the resotion 292*384 by 196*256)
"""


"""Example Program for XTherm T3s Camera using OpenCV"""
from ctypes import *    ## ['ctypes' is a function library for PYTHON. It provides C compatible datatype & support calling functions in DLL  (see this for MATLAB)]
import sys
import os
import datetime, time
import cv2
import numpy as np
from scipy import ndimage

from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5 import QtGui

#####################################################################################################################################
is_64bits = sys.maxsize > 2**32
print('Loading binary')
XthermDll = cdll.LoadLibrary(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Project1.dll'))   # Xtherm.dll
print('Ok')
#####################################################################################################################################
    
if not hasattr(QtGui.QImage, 'Format_Grayscale16'):
    force_use_8bits = True
    print('The version of PyQt5 ({}) used is too old (should > 5.13)\nQtGui.QImage does not support 16bit grayscale, forcing to use 8bit display'.format(QtCore.PYQT_VERSION_STR))
else:
    force_use_8bits = False

# defaults
seq_cap_num = 12
SCuseCal = 1
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
                    data = data.view(np.uint16).reshape(196, 256) #292, 384
                    data[0:192, :] = self.calFunc(data[0:192, :]) #data[0:288, :]
                    self.image_data_signal.emit(data)
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
            self.camera = SearchForXThermT3s()
        else:
            self.camera = cv2.VideoCapture(camera_port)  # ,cv2.CAP_DSHOW)

        if not self.camera:
            sys.exit("Xtherm camera not found")
        ######################
        self.calOffset = np.zeros((192, 256), dtype=np.int16) # (288, 384)
        self.calSlope = np.ones((192, 256), dtype=np.float32) # (288, 384)
        #######################
        self.camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # CV_CAP_PROP_CONVERT_RGB
        # CV_CAP_PROP_ZOOM use raw mode
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8004)
        # CV_CAP_PROP_ZOOM calibrate
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8000)
        # CV_CAP_PROP_ZOOM temperature mode?
        self.camera.set(cv2.CAP_PROP_ZOOM, 0x8020)
        XthermDll.DataInit(256, 192) # 384, 288
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

        #XthermDll.UpdateParam(tempMode, rawFrame.ctypes.data_as(POINTER(c_byte)))

    def start_capture(self):
        self.capture_thread.running_signal.emit(1)

    def stop_capture(self):
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

    ###############################3
    def applyCal(self, img):
        img[0:192, :] = (np.multiply(img[0:192, :], self.calSlope) + self.calOffset).astype(np.uint16) # img[0:288, :]
        return img

    def calZeroize(self):
        self.calOffset = np.zeros((192, 256), dtype=np.int16) # (288, 384)
        self.calSlope = np.ones((192, 256), dtype=np.float32)


class ThermalDisplayWidget(QtWidgets.QWidget):
    def __init__(self, resy=512 + 30, resx=384, infobarY=30, parent=None, rec_status_signal=None):
        # (self, resy=636 + 60, resx=768, infobarY=60, parent=None, rec_status_signal=None):
        super().__init__(parent)
        self.image = QtGui.QImage()
        self.video_writer = None
        self.video_frames = 0
        self.set_size(resy, resx, infobarY)
        self.rec_status_signal = rec_status_signal
        self.flip_x = False
        self.flip_y = False

    def set_size(self, resy=512 + 30, resx=384, infobarY=30):
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

    def image_data_slot(self, rawFrame):
        global drawHiLo, use_clahe, clahe, colormap, use_NLMDenoising, use_deStrip, deStripk, deStripIter, use_equHist, use_bf, bf_sigmaColor, use_sharpening, sharpening_kernel
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
        XthermDll.GetTmpData(1, rawFrame.ctypes.data_as(POINTER(c_byte)),
                             byref(maxtmp), byref(maxx), byref(maxy),
                             byref(mintmp), byref(minx), byref(miny),
                             byref(centertmp),
                             tmparr.ctypes.data_as(POINTER(c_float)),
                             0)  # full frame temperature not necessary
        # there is not transformed temperature data in 400 deg range, use pixel values instead
        if tempMode:
            maxtmp.value = rawFrame[192, 4] # [288, 4]
            mintmp.value = rawFrame[192, 7] # [288, 7]
            centertmp.value = (rawFrame[143, 191] + rawFrame[143, 192] +
                               rawFrame[144, 191] + rawFrame[144, 192]) / 4
        fpatmp = c_float()
        coretmp = c_float()
        fpaavg = c_int()
        orgavg = c_int()
        XthermDll.GetDevData(byref(fpatmp), byref(coretmp), byref(fpaavg), byref(orgavg))

        # begin the processing of the image
        outputFrame = rawFrame[0:192, :] # [0:192, :]
        outputFrame = np.transpose(outputFrame)


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

        '''
            *** Script to grab average temperature of Hexagon camou-pixel and its surrounding - written by RMA
        '''
        temp_data = alltmp.reshape(192,256)
        grayscale = temp_data
        
        m,n = [(ss-1.)/2. for ss in (50,50)]
        y,x = np.ogrid[-m:m+1,-n:n+1]
        h = np.exp( -(x*x + y*y) / (2.*1*1) )
        h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
        sumh = h.sum()
        if sumh != 0:
            h /= sumh

        filtered = cv2.filter2D(grayscale,-1,h)
        
        #cv2.imshow('Identity', filtered)
        #Edgecanny = Canny_detector(filtered)
        Kx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], np.float32)
        Ky = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]], np.float32)

        Ix = ndimage.convolve(filtered, Kx)
        Iy = ndimage.convolve(filtered, Ky)
            
        G = np.hypot(Ix, Iy)
        G = G / G.max() * 255
        theta = np.arctan2(Iy, Ix)

        GG = cv2.threshold(G, 100, 255, cv2.THRESH_BINARY)[1]
        X = ndimage.binary_fill_holes(GG).astype(float)
        X.astype(int)
        Bw = X
        
        Bw[:,1:128] = 0     # set pixels value to 0 on left side
        Bw[:,-1-93:-1] = 0  # set pixels value to 0 on right side
        Bw[1:58, :] = 0     # set pixels value to 0 on top side
        Bw[-1-97: -1, :] = 0  # set pixels value to 0 on bottom side

        #print("Bw length")
        #print(Bw)
        #cv2.imshow('Bw',Bw)
        
        y = np.where(Bw == 1)
        
        row = len(y[0])
        col = row

        temp = np.empty(row, dtype=float)

        for i in range(0,row-1):
             #print(i)
             x = y[0][i]
             z = y[1][i]
             temp[i] = temp_data[x,z]
             ##print("detected edges")
 
        if len(temp) != 0:
            pixel_temp = 1
            #print("t")
            temp_max = max(temp)
            temp_avg = sum(temp) / len(temp)
            #print('px t a')
            #print(temp_avg)
        else:
            pixel_temp = 0

        background = Bw
        background[45:98 , 117:181] = 1

        background_img = background
        for i in range(0,row-1):
             x = y[0][i]
             z = y[1][i]
             background_img[x,z] = 0

        yy = np.where(background_img == 1)
        row = len(yy[0])

        col = row
        
        bg_temp = np.empty(row, dtype=float)
        for i in range(0,row-1):
             xx = yy[0][i]
             zz = yy[1][i]
             bg_temp[i] = temp_data[xx,zz]

        if len(bg_temp) != 0:
            bg_temp_max = max(bg_temp)
            bg_temp_avg = sum(bg_temp) / len(bg_temp)
            #print('by ta')
            #print(bg_temp_avg)

        
 

        '''
            End of script for grabbing average temperatire of camou-pixel and surrounding
        '''
        
        # display image on GUI
        outputFrame = cv2.normalize(
            outputFrame,
            dst=None,
            alpha=0,
            beta=65535,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_16U)
##        print('OUTPUTFRAME')
##        print(outputFrame)

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
            #print('background')
            #print(bg_temp_avg)
            if pixel_temp == 0:
                temp_avg = 0.00
                #print('0')
            else:
                temp_avg = temp_avg
                #print(temp_avg)

                
            self.bufferedImg = np.zeros(
                (self.resy, self.resx), dtype=np.uint16)
            cv2.putText(
                self.bufferedImg,
                'FPAT:{:.2f} CORT:{:.2f} FPAA:{:5d} ORGA:{:5d} P_Tavg:{:.3f}'.format(
                    fpatmp.value, coretmp.value, fpaavg.value, orgavg.value, temp_avg) +
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
                'MAXT:{:.2f} {:3d},{:3d} MINT:{:.2f} {:3d},{:3d} CNTT:{:.2f} B_Tavg:{:.2f} '
                .format(maxtmp.value, maxx.value, maxy.value, mintmp.value, 
                        minx.value, miny.value, centertmp.value, bg_temp_avg),
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
    def matlab_style_gauss2D(shape=(50,50),sigma=1):
        m,n = [(ss-1.)/2. for ss in shape]
        y,x = np.ogrid[-m:m+1,-n:n+1]
        h = np.exp( -(x*x + y*y) / (2.*sigma*sigma) )
        h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
        sumh = h.sum()
        if sumh != 0:
            h /= sumh
        return h

    def Canny_detector(img, weak_th = None, strong_th = None):
        # conversion of image to grayscale
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
       
        # Noise reduction step
        img = cv2.GaussianBlur(img, (5, 5), 2)
       
        # Calculating the gradients
        gx = cv2.Sobel(np.float32(img), cv2.CV_64F, 1, 0, 3)
        gy = cv2.Sobel(np.float32(img), cv2.CV_64F, 0, 1, 3)
      
        # Conversion of Cartesian coordinates to polar 
        mag, ang = cv2.cartToPolar(gx, gy, angleInDegrees = True)
       
        # setting the minimum and maximum thresholds 
        # for double thresholding
        mag_max = np.max(mag)
        if not weak_th:weak_th = mag_max * 0.1
        if not strong_th:strong_th = mag_max * 0.5
      
        # getting the dimensions of the input image  
        height, width = img.shape
       
        # Looping through every pixel of the grayscale 
        # image
        for i_x in range(width):
            for i_y in range(height):
                grad_ang = ang[i_y, i_x]
                grad_ang = abs(grad_ang-180) if abs(grad_ang)>180 else abs(grad_ang)
               
                # selecting the neighbours of the target pixel
                # according to the gradient direction
                # In the x axis direction
                if grad_ang<= 22.5:
                    neighb_1_x, neighb_1_y = i_x-1, i_y
                    neighb_2_x, neighb_2_y = i_x + 1, i_y
              
                # top right (diagonal-1) direction
                elif grad_ang>22.5 and grad_ang<=(22.5 + 45):
                    neighb_1_x, neighb_1_y = i_x-1, i_y-1
                    neighb_2_x, neighb_2_y = i_x + 1, i_y + 1
              
                # In y-axis direction
                elif grad_ang>(22.5 + 45) and grad_ang<=(22.5 + 90):
                    neighb_1_x, neighb_1_y = i_x, i_y-1
                    neighb_2_x, neighb_2_y = i_x, i_y + 1
              
                # top left (diagonal-2) direction
                elif grad_ang>(22.5 + 90) and grad_ang<=(22.5 + 135):
                    neighb_1_x, neighb_1_y = i_x-1, i_y + 1
                    neighb_2_x, neighb_2_y = i_x + 1, i_y-1
              
                # Now it restarts the cycle
                elif grad_ang>(22.5 + 135) and grad_ang<=(22.5 + 180):
                    neighb_1_x, neighb_1_y = i_x-1, i_y
                    neighb_2_x, neighb_2_y = i_x + 1, i_y
               
                # Non-maximum suppression step
                if width>neighb_1_x>= 0 and height>neighb_1_y>= 0:
                    if mag[i_y, i_x]<mag[neighb_1_y, neighb_1_x]:
                        mag[i_y, i_x]= 0
                        continue
   
                if width>neighb_2_x>= 0 and height>neighb_2_y>= 0:
                    if mag[i_y, i_x]<mag[neighb_2_y, neighb_2_x]:
                        mag[i_y, i_x]= 0
   
        weak_ids = np.zeros_like(img)
        strong_ids = np.zeros_like(img)              
        ids = np.zeros_like(img)
       
        # double thresholding step
        for i_x in range(width):
            for i_y in range(height):
              
                grad_mag = mag[i_y, i_x]
              
                if grad_mag<weak_th:
                    mag[i_y, i_x]= 0
                elif strong_th>grad_mag>= weak_th:
                    ids[i_y, i_x]= 1
                else:
                    ids[i_y, i_x]= 2
       
       
        # finally returning the magnitude of
        # gradients of edges
        return mag

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


class MainWidget(QtWidgets.QWidget):
    rec_status_signal = QtCore.pyqtSignal(str)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.thermal_display_widget = ThermalDisplayWidget(rec_status_signal=self.rec_status_signal)
        self.thermal_video_stream = ThermalVideoStream()

        image_data_slot = self.thermal_display_widget.image_data_slot
        self.thermal_video_stream.image_data_signal.connect(image_data_slot)
        
        self.rec_status_signal.connect(self.update_recording_label)

        layoutDown1 = QtWidgets.QHBoxLayout()
        self.run_button = QtWidgets.QPushButton('Start')
        layoutDown1.addWidget(self.run_button)
        self.run_button.clicked.connect(
            self.thermal_video_stream.start_capture)
        self.stop_button = QtWidgets.QPushButton('Stop')
        layoutDown1.addWidget(self.stop_button)
        self.stop_button.clicked.connect(
            self.thermal_video_stream.stop_capture)
        self.flip_x_button = QtWidgets.QPushButton('Flip X')
        layoutDown1.addWidget(self.flip_x_button)
        self.flip_x_button.clicked.connect(self.thermal_display_widget.flip_x_toggle)
        self.flip_y_button = QtWidgets.QPushButton('Flip Y')
        layoutDown1.addWidget(self.flip_y_button)
        self.flip_y_button.clicked.connect(self.thermal_display_widget.flip_y_toggle)

        self.ok = QtWidgets.QPushButton('Okay')
        layoutDown1.addWidget(self.ok)

        layoutDown2 = QtWidgets.QHBoxLayout()
        self.dump_rawdata_button = QtWidgets.QPushButton('Dump Raw CSV')
        layoutDown2.addWidget(self.dump_rawdata_button)
        self.dump_rawdata_button.clicked.connect(
            self.thermal_display_widget.dump_rawdata)
        self.dump_rawdata_png_button = QtWidgets.QPushButton('Dump Raw image')
        layoutDown2.addWidget(self.dump_rawdata_png_button)
        self.dump_rawdata_png_button.clicked.connect(
            self.thermal_display_widget.dump_rawdata_png)
        self.seq_cap_button = QtWidgets.QPushButton('Burst Seq Cap')
        layoutDown2.addWidget(self.seq_cap_button)
        self.seq_cap_button.clicked.connect(self.thermal_video_stream.seq_cap)
        self.sav_img_button = QtWidgets.QPushButton('Save Img')
        layoutDown2.addWidget(self.sav_img_button)
        self.sav_img_button.clicked.connect(
            self.thermal_display_widget.sav_img)

        layoutDown3 = QtWidgets.QHBoxLayout()
        self.save_temperature_button = QtWidgets.QPushButton('Save Temperature')
        layoutDown3.addWidget(self.save_temperature_button)
        self.save_temperature_button.clicked.connect(
            self.thermal_display_widget.save_temperature)
        self.self_cal_button = QtWidgets.QPushButton('Shutter Cal')
        layoutDown3.addWidget(self.self_cal_button)
        self.self_cal_button.clicked.connect(self.thermal_video_stream.selfCal)
        self.calDialog = QtWidgets.QPushButton('Cali...')
        layoutDown3.addWidget(self.calDialog)
        self.calDialog.clicked.connect(self.startCalDialog)
        self.edit_par_button = QtWidgets.QPushButton('Meas...')
        layoutDown3.addWidget(self.edit_par_button)
        self.edit_par_button.clicked.connect(self.editPara)
        
        layoutDown4 = QtWidgets.QHBoxLayout()
        self.rec_status_label = QtWidgets.QLabel('Not Recording')
        layoutDown4.addWidget(self.rec_status_label)
        self.rec_button = QtWidgets.QPushButton('Rec Run/Stop')
        layoutDown4.addWidget(self.rec_button)
        self.rec_button.clicked.connect(self.thermal_display_widget.toggle_recording)
        

        layoutDown5 = QtWidgets.QHBoxLayout()
        self.exit_button = QtWidgets.QPushButton('EXIT NOW!')
        layoutDown5.addWidget(self.exit_button)
        self.exit_button.clicked.connect(exitNow)
        self.set_sz_button = QtWidgets.QPushButton('Set Size')
        layoutDown5.addWidget(self.set_sz_button)
        self.set_sz_button.clicked.connect(
            self.thermal_display_widget.toggle_size)
        self.settings_button = QtWidgets.QPushButton('Proc...')
        layoutDown5.addWidget(self.settings_button)
        self.settings_button.clicked.connect(startSettings)

        if QtWidgets.QDesktopWidget().screenGeometry(-1).height() < 1200:
            layoutVert = QtWidgets.QVBoxLayout()
            layoutVert.addLayout(layoutDown1)
            layoutVert.addLayout(layoutDown2)
            layoutVert.addLayout(layoutDown3)
            layoutVert.addLayout(layoutDown4)
            layoutVert.addLayout(layoutDown5)
            layoutHori = QtWidgets.QHBoxLayout()
            layoutHori.addWidget(self.thermal_display_widget)
            layoutHori.addLayout(layoutVert)
            self.setLayout(layoutHori)
        else:
            layoutVert = QtWidgets.QVBoxLayout()
            layoutVert.addWidget(self.thermal_display_widget)
            layoutVert.addLayout(layoutDown1)
            layoutVert.addLayout(layoutDown2)
            layoutVert.addLayout(layoutDown3)
            layoutVert.addLayout(layoutDown4)
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
    main_widget = MainWidget()
    main_window.setCentralWidget(main_widget)
    main_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

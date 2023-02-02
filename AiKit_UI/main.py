#! /usr/bin/env pyhton3
# encoding:utf-8
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSlot, Qt, QCoreApplication
from PyQt5.QtGui import QEnterEvent, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication, QInputDialog, QWidget, QMessageBox
from pymycobot.mycobot import MyCobot
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.ultraArm import ultraArm

from log import logfile
from pyqtFile.AiKit import Ui_AiKit_UI as AiKit_window


class AiKit_APP(AiKit_window, QMainWindow, QWidget):
    def __init__(self):
        super(AiKit_APP, self).__init__()
        self.setupUi(self)
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # self.M5color.init()
        self.myCobot = None
        self.loger = logfile.MyLogging().logger
        self.path = os.path.split(os.path.abspath(__file__))
        self.port_list = []
        self.loger = logfile.MyLogging().logger
        self._init_main_window()
        self._close_max_min_icon()
        self._initDrag()  # Set the mouse tracking judgment trigger default value
        self.setMouseTracking(True)  # Set widget mouse tracking
        self.widget.installEventFilter(self)  # Initialize event filter
        self.move(450, 10)
        self.radioButton_A.setChecked(True)
        self._init_variable()
        self._init_status()

        self._init_language()

        self.cap = cv2.VideoCapture()  # video stream
        self.min_btn.clicked.connect(self.min_clicked)  # minimize
        # self.max_btn.clicked.connect(self.max_clicked)
        self.close_btn.clicked.connect(self.close_clicked)  # close
        self.comboBox_function.activated.connect(self.combox_func_checked)  # switch algorithm
        self.comboBox_port.highlighted.connect(self.get_serial_port_list)  # get serial port
        self.comboBox_port.activated.connect(self.get_serial_port_list)  # get serial port
        self.comboBox_device.currentTextChanged.connect(self.buad_choose)  # Device drop-down box
        self.connect_btn.clicked.connect(self.connect_checked)  # connect button
        self.open_camera_btn.clicked.connect(self.camera_checked)  # open/close camera
        self.yolov5_cut_btn.clicked.connect(self.cut_yolov5_img)
        self.auto_btn.clicked.connect(self.auto_mode)  # automation
        self.discern_btn.clicked.connect(self.discern_func)  # discern
        self.crawl_btn.clicked.connect(self.crawl_func)  # crawl
        self.place_btn.clicked.connect(self.place_func)  # place
        self.to_origin_btn.clicked.connect(self.to_origin_func)  # to_origin
        self.offset_save_btn.clicked.connect(self.insert_offsets)  # update offsets
        self.open_file_btn.clicked.connect(self.open_file)  # open file
        self.add_img_btn.clicked.connect(self.add_img)  # add image
        self.exit_add_btn.clicked.connect(self.exit_add)  # exit add image
        self.image_coord_btn.clicked.connect(self.get_img_coord)  # get the image coords
        self.current_coord_btn.clicked.connect(self.get_current_coord_btnClick)  # get the robot coords
        self.language_btn.clicked.connect(self.set_language)  # set language
        self.get_serial_port_list()
        self.combox_func_checked()
        self.offset_change()
        self.btn_status()
        self.device_coord()
        self.cut_yolov5_img_status()
        self._init_tooltip()

    # Initialize variables
    def _init_variable(self):
        self.pump_y = 0
        self.pump_x = 0
        # device
        self.M5 = ['myPalletizer 260 for M5', 'myCobot 280 for M5', 'ultraArm P340']  # M5 robot
        self.Pi = ['myCobot 280 for Pi', 'mechArm 270 for Pi']  # Pi robot

        # angles to move
        self.move_angles = [
            [-29.0, 5.88, -4.92, -76.28],  # point to grab
            [17.4, -10.1, -87.27, 5.8],  # point to grab
        ]
        # origin coords
        self.home_coords = [166.4, -21.8, 219, 0.96]

        # coords to move
        self.move_coords = [
            [132.6, -155.6, 211.8, -20.9],  # D Sorting area
            [232.5, -134.1, 197.7, -45.26],  # C Sorting area
            [111.6, 159, 221.5, -120],  # A Sorting area
            [-15.9, 164.6, 217.5, -119.35],  # B Sorting area
        ]

        # The internal parameter matrix of the camera
        self.camera_matrix = np.array([
            [781.33379113, 0., 347.53500524],
            [0., 783.79074192, 246.67627253],
            [0., 0., 1.]])

        # Distortion coefficient of the camera
        self.dist_coeffs = np.array(([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03, 6.70503562e-03,
                                       2.57018000e+00]]))

        self.color = 0
        # parameters to calculate camera clipping parameters 计算相机裁剪参数的参数
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord 设置真实坐标的缓存
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            # "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
            "yellow": [np.array([22, 93, 0]), np.array([45, 255, 245])],
            "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
            "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }
        # Used to calculate the coordinates between the cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # Grab the coordinates of the center point relative to mycobot
        self.camera_x, self.camera_y = 165, 5
        # The coordinates of the cube relative to mycobot
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Initialize the background subtractor
        self.mog = cv2.bgsegm.createBackgroundSubtractorMOG()
        # yolov5 model file path
        self.modelWeights = self.path[0] + '/yolov5File/yolov5s.onnx'

        self._init_ = 20
        self.init_num = 0
        self.nparams = 0
        self.num = 0
        self.real_sx = self.real_sy = 0

        # Constants.
        self.INPUT_WIDTH = 640  # 640
        self.INPUT_HEIGHT = 640  # 640
        self.SCORE_THRESHOLD = 0.5
        self.NMS_THRESHOLD = 0.45
        self.CONFIDENCE_THRESHOLD = 0.45

        # Text parameters.
        self.FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
        self.FONT_SCALE = 0.7
        self.THICKNESS = 1

        # Colors.
        self.BLACK = (0, 0, 0)
        self.BLUE = (255, 178, 50)
        self.YELLOW = (0, 255, 255)

    # initialization status
    def _init_status(self):
        self.camera_status = False  # camera open state
        self.discern_status = False  # Whether to enable recognition
        self.crawl_status = False  # Whether to enable crawling
        self.place_status = False  # Whether to open the placement
        self.cut_status = False  # Whether to open and cut pictures
        self.auto_mode_status = False  # Whether to enable automatic operation
        self.img_coord_status = False  # Whether to enable the display of the X and Y coordinates of the object
        self.current_coord_status = False  # Whether to enable displaying the real-time location of the robot
        self.is_pick = False  # Whether the object has been grasped
        self.yolov5_is_not_pick = True
        self.is_yolov5_cut_btn_clicked = False
        self.yolov5_count = 0
        self.open_camera_func = 1  # Opening mode of the camera, 1 is the open button, 2 is the add button
        with open(self.path[0] + f'/offset/language.txt', "r", encoding="utf-8") as f:
            lange = f.read()
        self.language = int(lange)  # Control language, 1 is English, 2 is Chinese
        if self.language == 1:
            self.btn_color(self.language_btn, 'green')
        else:
            self.btn_color(self.language_btn, 'blue')
        self.is_language_btn_click = False

    # Initialize window borders
    def _init_main_window(self):
        # Set the form to be borderless
        self.setWindowFlags(Qt.FramelessWindowHint)
        # Set the background to be transparent
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Set software icon
        w = self.logo_lab.width()
        h = self.logo_lab.height()
        self.pix = QPixmap(self.path[0] + '/res/logo.png')  # the path to the icon
        self.logo_lab.setPixmap(self.pix)
        self.logo_lab.setScaledContents(True)

    # Close, minimize button display text
    def _close_max_min_icon(self):
        # self.close_btn.setFont(QFont("Webdings"))
        # self.max_btn.setFont(QFont("Webdings"))
        # self.min_btn.setFont(QFont("Webdings"))
        self.close_btn.setText('x')
        # self.max_btn.setText('1')
        self.min_btn.setText('-')

    def _init_tooltip(self):
        if self.language == 1:
            self.func_lab_6.setToolTip(
                'Adjust the suction position of the end, add X forward,'
                ' subtract X backward, add Y to the left, and subtract Y to the right.')
        else:
            self.func_lab_6.setToolTip(
                '调整末端吸取位置，向前X加，向后X减，向左Y加，向右Y减。')

    @pyqtSlot()
    def min_clicked(self):
        # minimize
        self.showMinimized()

    @pyqtSlot()
    def max_clicked(self):
        # Maximize and restore (not used)
        if self.isMaximized():
            self.showNormal()
            self.max_btn.setText('1')  # toggle magnify button icon
            self.max_btn.setToolTip("<html><head/><body><p>maximize</p></body></html>")
        else:
            self.showMaximized()
            self.max_btn.setText('2')
            self.max_btn.setToolTip("<html><head/><body><p>recover</p></body></html>")

    @pyqtSlot()
    def close_clicked(self):
        # turn off an app
        if self.camera_status:
            self.close_camera()
        self.close()
        QCoreApplication.instance().quit

    def _initDrag(self):
        # Set the mouse tracking judgment trigger default value
        self._move_drag = False

    def eventFilter(self, obj, event):
        # Event filter, used to solve the problem of reverting to the standard mouse style after the mouse enters other controls
        if isinstance(event, QEnterEvent):
            self.setCursor(Qt.ArrowCursor)
        return super(AiKit_APP, self).eventFilter(obj, event)  # Note that MyWindow is the name of the class
        # return QWidget.eventFilter(self, obj, event)  # You can also use this, but pay attention to modifying the window type

    def mousePressEvent(self, event):
        # Rewrite mouse click event
        if (event.button() == Qt.LeftButton) and (event.y() < self.widget.height()):
            # Click the left mouse button on the title bar area
            self._move_drag = True
            self.move_DragPosition = event.globalPos() - self.pos()
            event.accept()

    def mouseMoveEvent(self, QMouseEvent):
        try:
            if Qt.LeftButton and self._move_drag:
                # title bar drag and drop window position
                self.move(QMouseEvent.globalPos() - self.move_DragPosition)
                QMouseEvent.accept()
        except Exception as e:
            self.loger.info(e)

    def mouseReleaseEvent(self, QMouseEvent):
        # After the mouse is released, each trigger resets
        self._move_drag = False

    def has_mycobot(self):
        """Check whether it is connected on mycobot"""
        if not self.myCobot:
            self.loger.info("Mycobot is not connected yet! ! ! Please connect to myCobot first! ! !")
            return False
        return True

    def get_serial_port_list(self):
        """Get the current serial port and map it to the serial port drop-down box"""
        plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
        ]
        if not plist:
            if self.comboBox_port.currentText() == 'NO Port':
                return
            self.comboBox_port.clear()
            self.comboBox_port.addItem('NO Port')
            self.connect_btn.setEnabled(False)
            self.connect_btn.setStyleSheet("background-color: rgb(185, 195, 199);\n"
                                           "color: rgb(255, 255, 255);\n"
                                           "border-radius: 10px;\n"
                                           "border: 2px groove gray;\n"
                                           "border-style: outset;")
            self.port_list = []
            return
        else:
            if self.port_list != plist:
                self.port_list = plist
                self.comboBox_port.clear()
                self.connect_btn.setEnabled(True)
                self.connect_btn.setStyleSheet("background-color: rgb(39, 174, 96);\n"
                                               "color: rgb(255, 255, 255);\n"
                                               "border-radius: 10px;\n"
                                               "border: 2px groove gray;\n"
                                               "border-style: outset;")
                for p in plist:
                    self.comboBox_port.addItem(p)

    def buad_choose(self):
        try:
            """Switch the baud rate according to the device and initialize the corresponding variable"""
            # self.btn_status(True)
            value = self.comboBox_device.currentText()
            if value in self.Pi:
                # self.comboBox_buad.clear()
                # self.comboBox_buad.addItem('1000000')
                self.comboBox_buad.setCurrentIndex(0)
            else:
                # self.comboBox_buad.clear()
                # self.comboBox_buad.addItem('115200')
                self.comboBox_buad.setCurrentIndex(1)

            self.offset_change()  # Get the corresponding offset of the device
            self.device_coord()  # Initialize the point of the corresponding device
        except Exception as e:
            self.loger.error(str(e))

    def device_coord(self):
        """Get points according to the device"""
        value = self.comboBox_device.currentText()
        if value == 'myCobot 280 for Pi' or value == 'myCobot 280 for M5':
            # yolov5 model file path
            self.modelWeights = self.path[0] + "/yolov5File/yolov5s.onnx"
            # y-axis offset
            self.pump_y = -55
            # x-axis offset
            self.pump_x = 15
            self.move_angles = [
                [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
                [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
            ]
            self.move_coords = [
                [132.2, -136.9, 200.8, -178.24, -3.72, -107.17],  # D Sorting area
                [238.8, -124.1, 204.3, -169.69, -5.52, -96.52],  # C Sorting area
                [115.8, 177.3, 210.6, 178.06, -0.92, -6.11],  # A Sorting area
                [-6.9, 173.2, 201.5, 179.93, 0.63, 33.83],  # B Sorting area
            ]
            self.home_coords = [145.0, -65.5, 280.1, 178.99, 7.67, -179.9]
        elif value == 'myPalletizer 260 for M5':
            self.pump_y = -45
            self.pump_x = -30
            self.move_angles = [
                [-22.0, 0, 0, 7.28],  # point to grab
                [17.4, -10.1, -87.27, 5.8],  # point to grab
            ]

            self.home_coords = [166.4, -21.8, 219, 0.96]

            self.move_coords = [
                [132.6, -155.6, 211.8, -20.9],  # D Sorting area
                [232.5, -134.1, 197.7, -45.26],  # C Sorting area
                [111.6, 159, 221.5, -120],  # A Sorting area
                [-15.9, 164.6, 217.5, -119.35],  # B Sorting area
            ]
        elif value == 'mechArm 270 for Pi':
            self.pump_y = -55
            self.pump_x = 15
            self.move_angles = [
                [-33.31, 2.02, -10.72, -0.08, 95, -54.84],  # point to grab
                [0, 0, 0, 0, 90, 0],  # init the point
            ]

            self.move_coords = [
                [96.5, -101.9, 185.6, 155.25, 19.14, 75.88],  # D
                [180.9, -99.3, 184.6, 124.4, 30.9, 80.58],  # C
                [77.4, 122.1, 179.2, 151.66, 17.94, 178.24],  # A
                [2.2, 128.5, 171.6, 163.27, 10.58, -147.25]  # B
            ]
            self.home_coords = [81.8, -52.3, 186.7, 174.48, 4.08, 92.41]
        elif value == 'ultraArm P340':
            self.pump_y = -30
            # x-axis offset
            self.pump_x = -45
            # 移动角度
            self.move_angles = [
                [25.55, 0.0, 15.24, 0],
                [0.0, 14.32, 0.0, 0],  # point to grab
            ]

            # 移动坐标
            self.move_coords = [
                [141.53, 148.67, 43.73, 0],  # D Sorting area
                [248.52, 152.35, 53.45, 0],  # C Sorting area
                [269.02, -161.65, 51.42, 0],  # A Sorting area
                [146.8, -159.53, 50.44, 0],  # B Sorting area
            ]
            self.home_coords = [267.15, 0.0, 125.96]

    def connect_mycobot(self):
        """Connect the arm"""
        self.comboBox_port.setEnabled(False)
        self.comboBox_buad.setEnabled(False)
        self.comboBox_device.setEnabled(False)
        device = self.comboBox_device.currentText()
        port = self.comboBox_port.currentText()
        baud = self.comboBox_buad.currentText()
        baud = int(baud)

        try:
            if device == 'myPalletizer 260 for M5':
                self.myCobot = MyPalletizer(port, baud, timeout=0.2)
            elif device == 'ultraArm P340':
                self.myCobot = ultraArm(port, baud, timeout=0.2)
                self.stop_wait(0.1)
                zero = threading.Thread(target=self.go_zero)
                zero.start()
                if self.language == 1:
                    self.prompts('Zero calibration is in progress, please wait patiently......')
                else:
                    self.prompts('正在进行回零校正，请耐心等待......')
                self.btn_status(False)
                self.connect_btn.setEnabled(False)
            else:
                self.myCobot = MyCobot(port, baud, timeout=0.2)
            self.stop_wait(0.5)
            self.loger.info("connection succeeded !")
            if device != 'ultraArm P340':
                self.btn_status(True)
            if self.language == 1:
                self.connect_btn.setText('DISCONNECT')
            else:
                self.connect_btn.setText('断开')
            self.btn_color(self.connect_btn, 'red')

        except Exception as e:
            err_log = """\
                \rConnection failed !!!
                \r=================================================
                {}
                \r=================================================
            """.format(
                e
            )
            self.myCobot = None
            self.comboBox_port.setEnabled(True)
            self.comboBox_buad.setEnabled(True)
            self.comboBox_device.setEnabled(True)
            self.btn_status(False)
            if self.language == 1:
                self.connect_btn.setText('CONNECT')
            else:
                self.connect_btn.setText('连接')
            self.btn_color(self.connect_btn, 'green')
            self.loger.error(err_log)

    def disconnect_mycobot(self):
        if not self.has_mycobot():
            return

        try:
            del self.myCobot
            self.myCobot = None
            self.loger.info("Disconnected successfully !")
            self.comboBox_port.setEnabled(True)
            self.comboBox_buad.setEnabled(True)
            self.comboBox_device.setEnabled(True)
            if self.language == 1:
                self.connect_btn.setText('CONNECT')
            else:
                self.connect_btn.setText('连接')
            self.btn_color(self.connect_btn, 'green')
            self.btn_color(self.discern_btn, 'blue')
            self.btn_status(False)
            self.auto_mode_status = False
            self.discern_status = False
            self.crawl_status = False
            self.place_status = False
            self.img_coord_status = False
            self.current_coord_status = False
            self.is_pick = False
        except AttributeError:
            self.loger.info("Not yet connected to mycobot！！！")

    def connect_checked(self):
        """State toggle for the connect button"""
        if self.language == 1:
            txt = 'CONNECT'
        else:
            txt = '连接'
        if self.connect_btn.text() == txt:
            self.connect_mycobot()
        else:
            self.disconnect_mycobot()

    def btn_status(self, status=False):
        """Some button state settings"""
        btn_blue = [self.to_origin_btn, self.crawl_btn, self.place_btn]
        btn_green = [self.auto_btn, self.current_coord_btn, self.image_coord_btn]
        if status:
            for b in btn_blue:
                b.setEnabled(True)  # clickable
                b.setStyleSheet("background-color: rgb(41, 128, 185);\n"
                                "color: rgb(255, 255, 255);\n"
                                "border-radius: 10px;\n"
                                "border: 2px groove gray;\n"
                                "border-style: outset;")
            for b in btn_green:
                b.setEnabled(True)
                b.setStyleSheet("background-color: rgb(39, 174, 96);\n"
                                "color: rgb(255, 255, 255);\n"
                                "border-radius: 10px;\n"
                                "border: 2px groove gray;\n"
                                "border-style: outset;")
        else:
            for b in btn_blue:
                b.setEnabled(False)  # not clickable
                b.setStyleSheet("background-color: rgb(185, 195, 199);\n"
                                "color: rgb(255, 255, 255);\n"
                                "border-radius: 10px;\n"
                                "border: 2px groove gray;\n"
                                "border-style: outset;")
            for b in btn_green:
                b.setEnabled(False)
                b.setStyleSheet("background-color: rgb(185, 195, 199);\n"
                                "color: rgb(255, 255, 255);\n"
                                "border-radius: 10px;\n"
                                "border: 2px groove gray;\n"
                                "border-style: outset;")

    def open_camera(self):
        """Turn on the camera"""
        try:
            if self.language == 1:
                self.prompts('Opening camera, please wait....')
            else:
                self.prompts('正在打开相机，请稍后....')
            QApplication.processEvents()
            self.camera_status = True
            self.camera_edit.setEnabled(False)
            flag = self.cap.open(int(self.camera_edit.text()))  # Get the serial number of the camera to open
            if not flag:  # Flag indicates whether the camera is successfully opened
                if self.language == 1:
                    self.prompts(
                        'The camera failed to open, please check whether the serial number is correct or the camera is connected.')
                else:
                    self.prompts('相机打开失败，请检查序号是否正确或摄像头已连接.')
                self.loger.error('Failed to open camera')
                self.close_camera()
                return
            self.prompts_lab.clear()
            if self.language == 1:
                self.add_img_btn.setText('Cut')
                self.open_camera_btn.setText('Close')
            else:
                self.add_img_btn.setText('剪切')
                self.open_camera_btn.setText('关闭')
            self.btn_color(self.open_camera_btn, 'red')
        except Exception as e:
            self.loger.error('Unable to open camera' + str(e))
            self.camera_status = False
            self.camera_edit.setEnabled(True)
            if self.language == 1:
                self.open_camera_btn.setText('Open')
            else:
                self.open_camera_btn.setText('打开')
            self.btn_color(self.open_camera_btn, 'green')

    def close_camera(self):
        """turn off the camera"""
        try:
            self.camera_status = False
            if self.has_mycobot():
                self.auto_mode_status = True
                self.auto_mode()
            self.cap.release()  # free video stream
            self.camera_edit.setEnabled(True)
            self.show_camera_lab.clear()  # Clear the video display area
            if self.language == 1:
                self.open_camera_btn.setText('Open')
            else:
                self.open_camera_btn.setText('打开')
            self.btn_color(self.open_camera_btn, 'green')
            self._init_variable()
            self.buad_choose()
            self.prompts_lab.clear()
        except Exception as e:
            self.loger.error('camera off exception' + str(e))

    def camera_checked(self):
        """Bind camera switch"""
        if self.language == 1:
            txt = 'Open'
        else:
            txt = '打开'
        if self.open_camera_btn.text() == txt:
            self.open_camera_func = 1
            self.show_camera()
        else:
            if self.language == 1:
                txt = 'Cut'
            else:
                txt = '剪切'
            if self.add_img_btn.text() == txt:
                if self.language == 1:
                    self.add_img_btn.setText('Add')
                else:
                    self.add_img_btn.setText('添加')
            self.close_camera()

    def show_camera(self):
        """matching algorithm for identification"""
        try:
            if not self.camera_status:  #
                self.open_camera()
            if not self.camera_status:
                return
            # Define Keypoints image storage/reading path
            num = sum_x = sum_y = 0
            is_release = False
            while self.camera_status:
                func = self.comboBox_function.currentText()
                if func == 'Color recognition' or func == '颜色识别':
                    # read camera
                    _, frame = self.cap.read()
                    # deal img
                    frame = self.transform_frame(frame)
                    QApplication.processEvents()
                    if self._init_ > 0:
                        self._init_ -= 1
                        if self.camera_status:
                            # The video color is converted back to RGB, so that it is the realistic color
                            show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            # Convert the read video data into QImage format
                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                     QtGui.QImage.Format_RGB888)
                            # Display the QImage in the Label that displays the video
                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                            continue

                    # calculate the parameters of camera clipping
                    QApplication.processEvents()
                    if self.init_num < 20:
                        if self.get_calculate_params(frame) is None:
                            if self.camera_status:
                                show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                         QtGui.QImage.Format_RGB888)
                                self.show_camera_lab.setPixmap(
                                    QtGui.QPixmap.fromImage(showImage))
                            continue
                        else:
                            x1, x2, y1, y2 = self.get_calculate_params(frame)
                            self.draw_marker(frame, x1, y1)
                            self.draw_marker(frame, x2, y2)
                            self.sum_x1 += x1
                            self.sum_x2 += x2
                            self.sum_y1 += y1
                            self.sum_y2 += y2
                            self.init_num += 1
                            continue
                    elif self.init_num == 20:
                        self.set_cut_params(
                            (self.sum_x1) / 20.0,
                            (self.sum_y1) / 20.0,
                            (self.sum_x2) / 20.0,
                            (self.sum_y2) / 20.0,
                        )
                        self.sum_x1 = self.sum_x2 = self.sum_y1 = self.sum_y2 = 0
                        self.init_num += 1
                        continue

                    # # calculate params of the coords between cube and mycobot
                    QApplication.processEvents()
                    if self.nparams < 10:
                        if self.get_calculate_params(frame) is None:
                            if self.camera_status:
                                show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                         QtGui.QImage.Format_RGB888)
                                self.show_camera_lab.setPixmap(
                                    QtGui.QPixmap.fromImage(showImage))
                            continue
                        else:
                            x1, x2, y1, y2 = self.get_calculate_params(frame)
                            self.draw_marker(frame, x1, y1)
                            self.draw_marker(frame, x2, y2)
                            self.sum_x1 += x1
                            self.sum_x2 += x2
                            self.sum_y1 += y1
                            self.sum_y2 += y2
                            self.nparams += 1
                            continue
                    elif self.nparams == 10:
                        self.nparams += 1
                        # calculate and set params of calculating real coord between cube and mycobot
                        self.set_params(
                            (self.sum_x1 + self.sum_x2) / 20.0,
                            (self.sum_y1 + self.sum_y2) / 20.0,
                            abs(self.sum_x1 - self.sum_x2) / 10.0 +
                            abs(self.sum_y1 - self.sum_y2) / 10.0
                        )
                        self.loger.info("Color recognition ok")
                        continue

                    # get detect result
                    QApplication.processEvents()
                    detect_result = None
                    if self.discern_status:
                        detect_result = self.color_detect(frame)
                    if detect_result is None:
                        if self.camera_status:
                            show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                     QtGui.QImage.Format_RGB888)
                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                        continue
                    else:
                        x, y = detect_result
                        # calculate real coord between cube and mycobot
                        self.real_x, self.real_y = self.get_position(x, y)
                        if self.num == 20:
                            if self.crawl_status or self.place_status:
                                self.decide_move(self.real_sx / 20.0, self.real_sy / 20.0, self.color)
                                self.num = self.real_sx = self.real_sy = 0
                        else:
                            self.num += 1
                            self.real_sy += self.real_y
                            self.real_sx += self.real_x

                    if self.camera_status:
                        show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                 QtGui.QImage.Format_RGB888)
                        self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                elif func == 'Keypoints' or func == '特征点识别':
                    try:
                        res_queue = [[], [], [], []]
                        res_queue[0] = self.parse_folder('res/D')
                        res_queue[1] = self.parse_folder('res/C')
                        res_queue[2] = self.parse_folder('res/A')
                        res_queue[3] = self.parse_folder('res/B')
                        QApplication.processEvents()
                        # read camera
                        _, frame = self.cap.read()
                        # deal img
                        frame = self.transform_frame(frame)

                        if self.init_num < 20:
                            if self.get_calculate_params(frame) is None:
                                if self.camera_status:
                                    show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                             QtGui.QImage.Format_RGB888)
                                    self.show_camera_lab.setPixmap(
                                        QtGui.QPixmap.fromImage(showImage))
                                continue
                            else:
                                x1, x2, y1, y2 = self.get_calculate_params(frame)
                                self.draw_marker(frame, x1, y1)
                                self.draw_marker(frame, x2, y2)
                                self.sum_x1 += x1
                                self.sum_x2 += x2
                                self.sum_y1 += y1
                                self.sum_y2 += y2
                                self.init_num += 1
                                continue
                        elif self.init_num == 20:
                            self.set_cut_params(
                                (self.sum_x1) / 20.0,
                                (self.sum_y1) / 20.0,
                                (self.sum_x2) / 20.0,
                                (self.sum_y2) / 20.0,
                            )
                            self.sum_x1 = self.sum_x2 = self.sum_y1 = self.sum_y2 = 0
                            self.init_num += 1
                            continue
                        # calculate params of the coords between cube and mycobot
                        if self.nparams < 10:
                            if self.get_calculate_params(frame) is None:
                                if self.camera_status:
                                    show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                             QtGui.QImage.Format_RGB888)
                                    self.show_camera_lab.setPixmap(
                                        QtGui.QPixmap.fromImage(showImage))
                                continue
                            else:
                                x1, x2, y1, y2 = self.get_calculate_params(frame)
                                self.draw_marker(frame, x1, y1)
                                self.draw_marker(frame, x2, y2)
                                self.sum_x1 += x1
                                self.sum_x2 += x2
                                self.sum_y1 += y1
                                self.sum_y2 += y2
                                self.nparams += 1
                                self.loger.info("Keypoints ok")
                                continue
                        elif self.nparams == 10:
                            self.nparams += 1
                            # calculate and set params of calculating real coord between cube and mycobot
                            self.set_params((self.sum_x1 + self.sum_x2) / 20.0,
                                            (self.sum_y1 + self.sum_y2) / 20.0,
                                            abs(self.sum_x1 - self.sum_x2) / 10.0 +
                                            abs(self.sum_y1 - self.sum_y2) / 10.0)
                            self.loger.info("ok")
                            continue
                        # get detect result
                        detect_result = None
                        for i, v in enumerate(res_queue):
                            if self.discern_status:
                                detect_result = self.obj_detect(frame, v)
                            if detect_result is None:
                                if self.camera_status:
                                    show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                             QtGui.QImage.Format_RGB888)
                                    self.show_camera_lab.setPixmap(
                                        QtGui.QPixmap.fromImage(showImage))
                                continue
                            else:
                                x, y = detect_result
                                # calculate real coord between cube and mycobot
                                self.real_x, self.real_y = self.get_position(x, y)
                                if self.num == 5:
                                    # self.color = i
                                    # self.pub_marker(self.real_sx / 5.0 / 1000.0,
                                    #                   self.real_sy / 5.0 / 1000.0)
                                    if self.crawl_status:
                                        self.decide_move(self.real_sx / 5.0, self.real_sy / 5.0,
                                                         self.color)
                                        self.num = self.real_sx = self.real_sy = 0
                                else:
                                    self.num += 1
                                    self.real_sy += self.real_y
                                    self.real_sx += self.real_x
                        if self.camera_status:
                            show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                     QtGui.QImage.Format_RGB888)
                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                    except Exception as e:
                        self.loger.error('Abnormal image recognition：' + str(e))
                elif func == 'QR code recognition' or func == '二维码识别':
                    try:
                        QApplication.processEvents()
                        success, img = self.cap.read()
                        if not success:
                            break
                        if self.discern_status:
                            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                            corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
                                gray, self.aruco_dict, parameters=self.aruco_params
                            )
                            if len(corners) > 0:
                                if ids is not None:
                                    ret = cv2.aruco.estimatePoseSingleMarkers(
                                        corners, 0.03, self.camera_matrix, self.dist_coeffs
                                    )
                                    # rvec:rotation offset,tvec:translation deviator
                                    (rvec, tvec) = (ret[0], ret[1])
                                    (rvec - tvec).any()
                                    xyz = tvec[0, 0, :]
                                    # calculate the coordinates of the aruco relative to the pump
                                    xyz = [round(xyz[0] * 1000 + self.pump_y+int(self.yoffset_edit.text()), 2), round(xyz[1] * 1000 + self.pump_x+int(self.xoffset_edit.text()), 2),
                                           round(xyz[2] * 1000, 2)]

                                    # cv.putText(img, 'coords' + str(xyz), (0, 64), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)
                                    for i in range(rvec.shape[0]):
                                        # draw the aruco on img
                                        cv2.aruco.drawDetectedMarkers(img, corners)
                                        if num < 40:
                                            sum_x += xyz[1]
                                            sum_y += xyz[0]
                                            num += 1
                                        elif num == 40:
                                            if self.crawl_status:
                                                self.decide_move(sum_x / 40.0, sum_y / 40.0, 0)
                                                num = sum_x = sum_y = 0
                        if self.camera_status:
                            show = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                     QtGui.QImage.Format_RGB888)
                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                    except Exception as e:
                        self.loger.error('abnormal' + str(e))
                elif func == 'yolov5':
                    try:
                        if self.yolov5_count != 0 and is_release:
                            self.open_camera()
                            self.comboBox_function.setEnabled(True)
                            self.open_camera_btn.setEnabled(True)
                            self.connect_btn.setEnabled(True)
                            is_release = False
                        # yolov5 img path
                        path_img = self.path[0] + r'\res\yolov5_detect.png'
                        # print(path_img)
                        QApplication.processEvents()
                        # read camera
                        ret, frame = self.cap.read()
                        # deal img
                        # frame = self.transform_frame(frame)

                        show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                 QtGui.QImage.Format_RGB888)
                        self.show_camera_lab.setPixmap(
                            QtGui.QPixmap.fromImage(showImage))
                        if self.language == 1:
                            self.prompts('Please click the Cut button to capture the picture of the whiteboard part of the QR code.')
                        else:
                            self.prompts('请点击Cut按钮截取二维码白板部分的图片，按Enter确认。')
                        if self.is_yolov5_cut_btn_clicked:
                            roi = cv2.selectROI(windowName="Cut Image",
                                                img=frame,
                                                showCrosshair=False,
                                                fromCenter=False)
                            cv2.moveWindow("Cut Image", 798, 220)
                            x, y, w, h = roi
                            # print('1111111111111111111111111111111111111111111111', roi)
                            if roi != (0, 0, 0, 0):
                                crop = frame[y:y + h, x:x + w]
                                cv2.imwrite(path_img, crop)
                                self.cap.release()
                                is_release = True
                                self.comboBox_function.setEnabled(False)
                                self.open_camera_btn.setEnabled(False)
                                self.connect_btn.setEnabled(False)
                                cv2.destroyAllWindows()

                            frame = frame[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]
                            show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                     QtGui.QImage.Format_RGB888)
                            width = showImage.width()
                            height = showImage.height()
                            if width / 640 >= height / 480:
                                ratio = width / 640
                            else:
                                ratio = height / 480
                            new_width = width / ratio
                            new_height = height / ratio
                            showImage = showImage.scaled(int(new_width), int(new_height), Qt.KeepAspectRatio)
                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                            if self.language == 1:
                                self.prompts(
                                    'Recognition, grabbing and placing are now possible.')
                            else:
                                self.prompts('现在可以进行识别、抓取和放置了。')
                            while self.yolov5_is_not_pick:
                                try:
                                    QApplication.processEvents()
                                    # print('imgpath:', path_img)
                                    frame = cv2.imread(path_img)
                                    # print('imgpath:', path_img)
                                    # frame = self.transform_frame(frame)
                                    if self._init_ > 0:
                                        self._init_ -= 1
                                        continue
                                    if self.init_num < 20:
                                        if self.get_calculate_params(frame) is None:
                                            if self.camera_status:
                                                show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                                showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                                                         show.shape[1] * 3,
                                                                         QtGui.QImage.Format_RGB888)
                                                self.show_camera_lab.setPixmap(
                                                    QtGui.QPixmap.fromImage(showImage))
                                            continue
                                        else:
                                            x1, x2, y1, y2 = self.get_calculate_params(frame)
                                            self.draw_marker(frame, x1, y1)
                                            self.draw_marker(frame, x2, y2)
                                            self.sum_x1 += x1
                                            self.sum_x2 += x2
                                            self.sum_y1 += y1
                                            self.sum_y2 += y2
                                            self.init_num += 1
                                            continue
                                    elif self.init_num == 20:
                                        self.set_cut_params(
                                            (self.sum_x1) / 20.0,
                                            (self.sum_y1) / 20.0,
                                            (self.sum_x2) / 20.0,
                                            (self.sum_y2) / 20.0,
                                        )
                                        self.sum_x1 = self.sum_x2 = self.sum_y1 = self.sum_y2 = 0
                                        self.init_num += 1
                                        continue
                                    # calculate params of the coords between cube and mycobot
                                    if self.nparams < 10:
                                        if self.get_calculate_params(frame) is None:
                                            if self.camera_status:
                                                show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                                showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                                                         show.shape[1] * 3,
                                                                         QtGui.QImage.Format_RGB888)
                                                self.show_camera_lab.setPixmap(
                                                    QtGui.QPixmap.fromImage(showImage))
                                            continue
                                        else:
                                            x1, x2, y1, y2 = self.get_calculate_params(frame)
                                            self.draw_marker(frame, x1, y1)
                                            self.draw_marker(frame, x2, y2)
                                            self.sum_x1 += x1
                                            self.sum_x2 += x2
                                            self.sum_y1 += y1
                                            self.sum_y2 += y2
                                            self.nparams += 1
                                            self.loger.info("yolov5 ok")
                                            continue
                                    elif self.nparams == 10:
                                        self.nparams += 1
                                        # calculate and set params of calculating real coord between cube and mycobot
                                        self.set_params((self.sum_x1 + self.sum_x2) / 20.0,
                                                        (self.sum_y1 + self.sum_y2) / 20.0,
                                                        abs(self.sum_x1 - self.sum_x2) / 10.0 +
                                                        abs(self.sum_y1 - self.sum_y2) / 10.0)
                                        self.loger.info("ok")
                                        continue
                                    # get detect result
                                    detect_result = None
                                    if self.discern_status:
                                        detect_result = self.post_process(frame)
                                    if detect_result:
                                    #     if self.camera_status:
                                    #         show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    #         showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                    #                                  show.shape[1] * 3,
                                    #                                  QtGui.QImage.Format_RGB888)
                                    #         self.show_camera_lab.setPixmap(
                                    #             QtGui.QPixmap.fromImage(showImage))
                                    #     continue
                                    # else:
                                        x, y, input_img = detect_result
                                        # calculate real coord between cube and mycobot
                                        self.real_x, self.real_y = self.get_position(x, y)
                                        # print('real_x_y', self.real_x,self.real_y)
                                        show = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
                                        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                                                 show.shape[1] * 3,
                                                                 QtGui.QImage.Format_RGB888)
                                        self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                                        QApplication.processEvents()
                                        if self.crawl_status:
                                            self.decide_move(self.real_x, self.real_y,
                                                             self.color)
                                            show = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
                                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
                                                                     show.shape[1] * 3,
                                                                     QtGui.QImage.Format_RGB888)
                                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                                            QApplication.processEvents()

                                            self.num = self.real_sx = self.real_sy = 0
                                except Exception as e:
                                    self.loger.error('yolov5 Exception:' + str(e))
                    except Exception as e:
                        self.loger.error('yolov5 Exception:' + str(e))
                else:
                    try:
                        QApplication.processEvents()
                        # read camera
                        _, frame = self.cap.read()
                        # deal img
                        frame = self.transform_frame(frame)
                        if self._init_ > 0:
                            self._init_ -= 1
                            if self.camera_status:
                                show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                         QtGui.QImage.Format_RGB888)
                                self.show_camera_lab.setPixmap(
                                    QtGui.QPixmap.fromImage(showImage))
                            continue

                        # calculate the parameters of camera clipping
                        if self.init_num < 20:
                            if self.get_calculate_params(frame) is None:
                                if self.camera_status:
                                    show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                             QtGui.QImage.Format_RGB888)
                                    self.show_camera_lab.setPixmap(
                                        QtGui.QPixmap.fromImage(showImage))
                                continue
                            else:
                                x1, x2, y1, y2 = self.get_calculate_params(frame)
                                self.draw_marker(frame, x1, y1)
                                self.draw_marker(frame, x2, y2)
                                self.sum_x1 += x1
                                self.sum_x2 += x2
                                self.sum_y1 += y1
                                self.sum_y2 += y2
                                self.init_num += 1
                                continue
                        elif self.init_num == 20:
                            self.set_cut_params(
                                (self.sum_x1) / 20.0,
                                (self.sum_y1) / 20.0,
                                (self.sum_x2) / 20.0,
                                (self.sum_y2) / 20.0,
                            )
                            self.sum_x1 = self.sum_x2 = self.sum_y1 = self.sum_y2 = 0
                            self.init_num += 1
                            continue

                        # calculate params of the coords between cube and mycobot
                        if self.nparams < 10:
                            if self.get_calculate_params(frame) is None:
                                if self.camera_status:
                                    show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                             QtGui.QImage.Format_RGB888)
                                    self.show_camera_lab.setPixmap(
                                        QtGui.QPixmap.fromImage(showImage))
                                continue
                            else:
                                x1, x2, y1, y2 = self.get_calculate_params(frame)
                                self.draw_marker(frame, x1, y1)
                                self.draw_marker(frame, x2, y2)
                                self.sum_x1 += x1
                                self.sum_x2 += x2
                                self.sum_y1 += y1
                                self.sum_y2 += y2
                                self.nparams += 1
                                continue
                        elif self.nparams == 10:
                            self.nparams += 1
                            # calculate and set params of calculating real coord between cube and mycobot
                            self.set_params(
                                (self.sum_x1 + self.sum_x2) / 20.0,
                                (self.sum_y1 + self.sum_y2) / 20.0,
                                abs(self.sum_x1 - self.sum_x2) / 10.0 +
                                abs(self.sum_y1 - self.sum_y2) / 10.0
                            )
                            continue

                        detect_result = None
                        if self.discern_status:
                            detect_result = self.shape_detect(frame)
                        if detect_result is None:
                            if self.camera_status:
                                show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                         QtGui.QImage.Format_RGB888)
                                self.show_camera_lab.setPixmap(
                                    QtGui.QPixmap.fromImage(showImage))
                            continue
                        else:
                            x, y = detect_result
                            # calculate real coord between cube and mycobot
                            self.real_x, self.real_y = self.get_position(x, y)
                            if self.num == 20:
                                if self.crawl_status:
                                    self.decide_move(self.real_sx / 20.0, self.real_sy / 20.0, self.color)
                                    self.num = self.real_sx = self.real_sy = 0

                            else:
                                self.num += 1
                                self.real_sy += self.real_y
                                self.real_sx += self.real_x

                        if self.camera_status:
                            show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                     QtGui.QImage.Format_RGB888)
                            self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                    except Exception as e:
                        self.loger.error('Abnormal shape recognition' + str(e))
        except Exception as e:
            self.loger.error(str(e))

    def discern_func(self):
        """Turn recognition on/off"""
        try:
            if self.discern_status:
                self.discern_status = False
                self.btn_color(self.discern_btn, 'blue')
                if self.auto_mode_status:
                    self.auto_mode_status = False
                    self.btn_color(self.auto_btn, 'green')
            else:
                self.discern_status = True
                self.btn_color(self.discern_btn, 'red')


        except Exception as e:
            self.loger.error('identify anomalies' + str(e))

    def crawl_func(self):
        """Turn crawling on/off"""
        if self.crawl_status:
            self.crawl_status = False
            self.btn_color(self.crawl_btn, 'blue')
            if self.auto_mode_status:
                self.auto_mode_status = False
                self.btn_color(self.auto_btn, 'green')
        else:
            self.crawl_status = True
            self.is_pick = True
            self.btn_color(self.crawl_btn, 'red')

    def place_func(self):
        """Turn on/off placement"""
        if self.place_status:
            self.place_status = False
            # self.is_pick = False
            self.btn_color(self.place_btn, 'blue')
            if self.auto_mode_status:
                self.auto_mode_status = False
                self.btn_color(self.auto_btn, 'green')
        else:
            self.place_status = True
            # self.is_pick = True
            self.btn_color(self.place_btn, 'red')

    def to_origin_func(self):
        try:
            """back to initial position"""
            self.is_pick = False
            self.pump_off()
            if self.comboBox_device.currentText() == 'ultraArm P340':
                self.myCobot.set_angles(self.move_angles[0], 30)
            else:
                self.myCobot.send_angles(self.move_angles[0], 30)
            self.stop_wait(3)
        except Exception as e:
            self.loger.error(str(e))

    def stop_wait(self, t):
        """Refresh the software screen in real time during the robot movement"""
        if t * 10 <= 1:
            t = 1
        else:
            t = int(t * 10)

        for i in range(1, t + 1):
            QApplication.processEvents()
            time.sleep(0.1)

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # self ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)

                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)

    # set parameters to calculate the coords between cube and mycobot
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0 / ratio

    # calculate the coords between cube and mycobot
    def get_position(self, x, y):
        pot_x = ((y - self.c_y) * self.ratio + self.camera_x)
        pot_y = ((x - self.c_x) * self.ratio + self.camera_y)
        if self.img_coord_status:
            self.img_coord_lab.clear()
            self.img_coord_lab.setText(f'X:{int(pot_x)}  Y:{int(pot_y)}')
        else:
            self.img_coord_lab.clear()
        return pot_x, pot_y

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        """Judging whether it is recognized normally"""
        try:
            # enlarge the image by 1.5 times
            fx = 1.5
            fy = 1.5
            frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                               interpolation=cv2.INTER_CUBIC)
            if self.x1 != self.x2:
                # the cutting ratio here is adjusted according to the actual situation
                frame = frame[int(self.y2 * 0.78):int(self.y1 * 1.1),
                        int(self.x1 * 0.84):int(self.x2 * 1.08)]
            return frame
        except Exception as e:
            # self.loger.error('Interception failed' + str(e))
            pass

    # detect cube color
    def color_detect(self, img):
        """color recognition"""
        # set the arrangement of color'HSV
        x = y = 0
        for mycolor, item in self.HSV.items():
            redLower = np.array(item[0])
            redUpper = np.array(item[1])

            # transfrom the img to model of gray
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # wipe off all color expect color in range
            mask = cv2.inRange(hsv, item[0], item[1])

            # a etching operation on a picture to remove edge roughness
            # 对图片进行蚀刻操作以去除边缘粗糙度
            erosion = cv2.erode(mask, np.ones((1, 1), np.uint8), iterations=2)

            # the image for expansion operation, its role is to deepen the color depth in the picture
            dilation = cv2.dilate(erosion, np.ones(
                (1, 1), np.uint8), iterations=2)

            # adds pixels to the image
            target = cv2.bitwise_and(img, img, mask=dilation)

            # the filtered image is transformed into a binary image and placed in binary
            # 将过滤后的图像转换为二值图像并放入二值
            ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)

            # get the contour coordinates of the image, where contours is the coordinate value, here only the contour is detected
            contours, hierarchy = cv2.findContours(
                dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # do something about misidentification
                boxes = [
                    box
                    for box in [cv2.boundingRect(c) for c in contours]
                    if min(img.shape[0], img.shape[1]) / 10
                       < min(box[2], box[3])
                       < min(img.shape[0], img.shape[1]) / 1
                ]
                if boxes:
                    for box in boxes:
                        x, y, w, h = box
                    # find the largest object that fits the requirements
                    c = max(contours, key=cv2.contourArea)
                    # get the lower left and upper right points of the positioning object
                    x, y, w, h = cv2.boundingRect(c)
                    # locate the target by drawing rectangle
                    cv2.rectangle(img, (x, y), (x + w, y + h), (153, 153, 0), 2)
                    # calculate the rectangle center
                    x, y = (x * 2 + w) / 2, (y * 2 + h) / 2
                    # calculate the real coordinates of mycobot relative to the target

                    if mycolor == "yellow":

                        self.color = 3
                        break

                    elif mycolor == "red":
                        self.color = 0
                        break

                    elif mycolor == "cyan":
                        self.color = 2
                        break

                    elif mycolor == "blue":
                        self.color = 2
                        break
                    elif mycolor == "green":
                        self.color = 1
                        break

        # Judging whether it is recognized normally
        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None

        # detect object

    def obj_detect(self, img, goal):
        """Keypoints"""
        i = 0
        MIN_MATCH_COUNT = 5
        sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp = []
        des = []

        for i in goal:
            QApplication.processEvents()
            kp0, des0 = sift.detectAndCompute(i, None)
            kp.append(kp0)
            des.append(des0)
        # kp1, des1 = sift.detectAndCompute(goal, None)
        kp2, des2 = sift.detectAndCompute(img, None)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)  # or pass empty dictionary
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        x, y = 0, 0
        try:
            for i in range(len(des)):
                QApplication.processEvents()
                matches = flann.knnMatch(des[i], des2, k=2)
                # store all the good matches as per Lowe's ratio test.  根据Lowe比率测试存储所有良好匹配项。
                good = []
                for m, n in matches:
                    if m.distance < 0.7 * n.distance:
                        good.append(m)

                # When there are enough robust matching point pairs
                if len(good) > MIN_MATCH_COUNT:
                    # extract corresponding point pairs from matching
                    # query index of small objects, training index of scenarios
                    src_pts = np.float32([kp[i][m.queryIdx].pt
                                          for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt
                                          for m in good]).reshape(-1, 1, 2)

                    # Using matching points to find homography matrix in cv2.ransac
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,
                                                 5.0)
                    matchesMask = mask.ravel().tolist()
                    # Calculate the distortion of image, that is the corresponding position in frame
                    h, w, d = goal[i].shape
                    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                                      [w - 1, 0]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, M)
                    ccoord = (dst[0][0] + dst[1][0] + dst[2][0] +
                              dst[3][0]) / 4.0

                    x = (dst[0][0][0] + dst[1][0][0] + dst[2][0][0] +
                         dst[3][0][0]) / 4.0
                    y = (dst[0][0][1] + dst[1][0][1] + dst[2][0][1] +
                         dst[3][0][1]) / 4.0

                    # bound box
                    img = cv2.polylines(img, [np.int32(dst)], True, 244, 3,
                                        cv2.LINE_AA)
                    # cv2.polylines(mixture, [np.int32(dst)], True, (0, 255, 0), 2, cv2.LINE_AA)
        except Exception as e:
            pass
        if x + y > 0:
            return x, y
        else:
            return None

    def shape_detect(self, img):
        """shape recognition"""
        x = 0
        y = 0

        Alpha = 65.6
        Gamma = -8191.5
        cal = cv2.addWeighted(img, Alpha, img, 0, Gamma)
        # 转换为灰度图片
        gray = cv2.cvtColor(cal, cv2.COLOR_BGR2GRAY)

        # a etching operation on a picture to remove edge roughness
        erosion = cv2.erode(gray, np.ones((2, 2), np.uint8), iterations=2)

        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation = cv2.dilate(erosion, np.ones(
            (1, 1), np.uint8), iterations=2)

        # 设定灰度图的阈值 175, 255
        _, threshold = cv2.threshold(dilation, 175, 255, cv2.THRESH_BINARY)
        # 边缘检测
        edges = cv2.Canny(threshold, 50, 100)
        # 检测物体边框
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            for cnt in contours:
                # if 6000>cv2.contourArea(cnt) and cv2.contourArea(cnt)>4500:
                if cv2.contourArea(cnt) > 5500:
                    objectType = None
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    objCor = len(approx)

                    boxes = [
                        box
                        for box in [cv2.boundingRect(c) for c in contours]
                        if min(img.shape[0], img.shape[1]) / 10
                           < min(box[2], box[3])
                           < min(img.shape[0], img.shape[1]) / 1
                    ]
                    if boxes:
                        for box in boxes:
                            x, y, w, h = box
                        # find the largest object that fits the requirements
                        c = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(img, [box], 0, (153, 153, 0), 2)
                        x = int(rect[0][0])
                        y = int(rect[0][1])

                    if objCor == 3:
                        objectType = ["Triangle", "三角形"]
                        self.color = 3
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)

                    elif objCor == 4:
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        _W = math.sqrt(math.pow((box[0][0] - box[1][0]), 2) + math.pow((box[0][1] - box[1][1]), 2))
                        _H = math.sqrt(math.pow((box[0][0] - box[3][0]), 2) + math.pow((box[0][1] - box[3][1]), 2))
                        aspRatio = _W / float(_H)
                        if 0.98 < aspRatio < 1.03:
                            objectType = ["Square", "正方形"]
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color = 1
                        else:
                            objectType = ["Rectangle", "长方形"]
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color = 2
                    elif objCor >= 5:
                        objectType = ["Circle", "圆形"]
                        self.color = 0
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                    else:
                        pass
                    if self.language == 1:
                        self.prompts(f"shape is {objectType[0]}")
                    else:
                        self.prompts(f"形状为{objectType[1]}")

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None

    def decide_move(self, x, y, color):
        device = self.comboBox_device.currentText()
        if device == 'yolov5':
            print(x,y)
            self.cache_x = self.cache_y = 0
            _moved = threading.Thread(target=self.moved(x, y))
            _moved.start()
            return
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # Adjust the suction position of the suction pump, increase y, move to the left;
            # decrease y, move to the right; increase x, move forward; decrease x, move backward
            if self.comboBox_function.currentText() == 'QR code recognition' or self.comboBox_function.currentText() == '二维码识别':
                if device == 'myPalletizer 260 for M5':
                    _moved = threading.Thread(target=self.moved(x + 28, y + 98))
                    _moved.start()
                elif device == 'mechArm 270 for Pi':
                    _moved = threading.Thread(target=self.moved(x + 38, y + 138))
                    _moved.start()
                elif device == 'myCobot 280 for Pi' or device == 'myCobot 280 for M5':
                    _moved = threading.Thread(target=self.moved(x - 5, y + 145))
                    _moved.start()
                elif device == 'ultraArm P340':
                    _moved = threading.Thread(target=self.moved(x + 50, y + 60))
                    _moved.start()
            else:
                _moved = threading.Thread(target=self.moved(x, y))
                _moved.start()

    # Grasping motion
    def moved(self, x, y):
        try:
            print('x',x)
            print('y',y)
            self.is_crawl = True
            while self.is_pick:
                QApplication.processEvents()
                # send Angle to move mycobot
                device = self.comboBox_device.currentText()
                if self.is_crawl:
                    if self.crawl_status:
                        self.is_crawl = False
                        if device == 'ultraArm P340':
                            self.myCobot.set_angles(self.move_angles[1], 20)
                        else:
                            self.myCobot.send_angles(self.move_angles[1], 20)
                        self.stop_wait(3)
                        func = self.comboBox_function.currentText()
                        # send coordinates to move mycobot
                        if func == 'QR code recognition' or func == '二维码识别':
                            if device == 'myPalletizer 260 for M5':
                                self.myCobot.send_coords([self.home_coords[0] + x, self.home_coords[1] + y, 103, 0], 20,
                                                         0)
                                self.stop_wait(2.5)
                                self.myCobot.send_coords([self.home_coords[0] + x, self.home_coords[1] + y, 62, 0], 20,
                                                         0)
                                self.stop_wait(2.5)
                            elif device == 'mechArm 270 for Pi':
                                self.myCobot.send_coords(
                                    [self.home_coords[0] + x, self.home_coords[1] + y, 150, 172.36, 5.36, 125.58], 30,
                                    0)
                                self.stop_wait(3.5)
                                self.myCobot.send_coords(
                                    [self.home_coords[0] + x, self.home_coords[1] + y, 93, 172.36, 5.36, 125.58], 30, 0)
                                self.stop_wait(3.5)
                                self.myCobot.send_coords(
                                    [self.home_coords[0] + x, self.home_coords[1] + y, 65, 172.36, 5.36, 125.58], 30, 0)
                                self.stop_wait(3.5)
                            elif device == 'myCobot 280 for Pi' or device == 'myCobot 280 for M5':
                                self.myCobot.send_coords(
                                    [self.home_coords[0] + x, self.home_coords[1] + y, 108, 178.99, -3.78, -62.9], 25,
                                    0)
                                self.stop_wait(2)
                                self.myCobot.send_coords(
                                    [self.home_coords[0] + x, self.home_coords[1] + y, 70, 178.99, -3.78, -62.9], 25, 0)
                                self.stop_wait(3.5)
                            elif device == 'ultraArm P340':
                                self.myCobot.set_coords([self.home_coords[0] + x, self.home_coords[1] - y, 65.51, 0],
                                                        50)
                                time.sleep(2)
                                self.myCobot.set_coords([self.home_coords[0] + x, self.home_coords[1] - y, -55, 0], 50)
                                time.sleep(3)

                        else:
                            if func == 'shape recognition' or func == 'Keypoints' or func == '形状识别' or func == '特征点识别' or func == 'yolov5':
                                if device == 'myPalletizer 260 for M5':
                                    self.myCobot.send_coords([x, y, 103, 0], 20, 0)
                                    self.stop_wait(2.5)
                                    self.myCobot.send_coords([x, y, 62, 0], 20, 0)
                                    self.stop_wait(1.5)
                                elif device == 'mechArm 270 for Pi':
                                    self.myCobot.send_coords([x, y, 110, -176.1, 2.4, -125.1], 25,
                                                             0)  # usb :rx,ry,rz -173.3, -5.48, -57.9
                                    self.stop_wait(3)
                                    self.myCobot.send_coords([x, y, 70, -176.1, 2.4, -125.1], 25, 0)
                                    self.stop_wait(3)
                                elif device == 'myCobot 280 for Pi' or device == 'myCobot 280 for M5':
                                    self.myCobot.send_coords([x, y, 170.6, 179.87, -3.78, -62.75], 25,
                                                             0)  # usb :rx,ry,rz -173.3, -5.48, -57.9
                                    self.stop_wait(3)
                                    self.myCobot.send_coords([x, y, 65.5, 179.87, -3.78, -62.75], 25, 0)
                                    self.stop_wait(4)
                                elif device == 'ultraArm P340':
                                    self.myCobot.set_coords([x, -y, 65.51, 0], 50)
                                    time.sleep(1.5)
                                    self.myCobot.set_coords([x, -y, -55, 0], 50)
                                    time.sleep(2)
                            else:
                                if device == 'myPalletizer 260 for M5':
                                    self.myCobot.send_coords([x, y, 160, 0], 20, 0)
                                    self.stop_wait(1.5)
                                    self.myCobot.send_coords([x, y, 103, 0], 20, 0)
                                    self.stop_wait(1.5)
                                elif device == 'mechArm 270 for Pi':
                                    self.myCobot.send_coords([x, y, 150, -176.1, 2.4, -125.1], 25,
                                                             0)  # usb :rx,ry,rz -173.3, -5.48, -57.9
                                    self.stop_wait(3)
                                    self.myCobot.send_coords([x, y, 108, -176.1, 2.4, -125.1], 25, 0)
                                    self.stop_wait(3)
                                elif device == 'myCobot 280 for Pi' or device == 'myCobot 280 for M5':
                                    self.myCobot.send_coords([x, y, 170.6, 179.87, -3.78, -62.75], 25,
                                                             0)  # usb :rx,ry,rz -173.3, -5.48, -57.9
                                    self.stop_wait(3)
                                    self.myCobot.send_coords([x, y, 103, 179.87, -3.78, -62.75], 25, 0)
                                    self.stop_wait(4)
                                elif device == 'ultraArm P340':
                                    self.myCobot.set_coords([x, -y, 65.51, 0], 50)
                                    time.sleep(1.5)
                                    self.myCobot.set_coords([x, -y, -18, 0], 50)
                                    time.sleep(2)

                        # open pump
                        self.pump_on()
                        self.stop_wait(2)
                        if device == 'myPalletizer 260 for M5':
                            self.myCobot.send_angle(2, 0, 20)
                            self.stop_wait(0.3)
                            self.myCobot.send_angle(3, -20, 20)
                            self.stop_wait(2)
                        elif device == 'ultraArm P340':
                            self.myCobot.set_angles([0, 0, 0, 0], 50)
                        else:
                            tmp = []
                            while True:
                                if not tmp:
                                    tmp = self.myCobot.get_angles()
                                else:
                                    break
                            if device == 'mechArm 270 for Pi':
                                self.myCobot.send_angles([tmp[0], 17.22, -32.51, tmp[3], 97, tmp[5]], 30)
                                self.stop_wait(3.5)
                            elif device == 'myCobot 280 for Pi' or device == 'myCobot 280 for M5':
                                self.myCobot.send_angles([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]], 25)
                                self.stop_wait(3)

                        if not self.auto_mode_status:
                            self.crawl_status = False
                            self.discern_status = False
                            self.btn_color(self.crawl_btn, 'blue')
                            self.btn_color(self.discern_btn, 'blue')
                if self.place_status:
                    self.is_pick = False
                    self.yolov5_is_not_pick = False
                    self.is_yolov5_cut_btn_clicked = False
                    self.yolov5_count += 1
                    if self.radioButton_A.isChecked():
                        color = 2
                    elif self.radioButton_B.isChecked():
                        color = 3
                    elif self.radioButton_C.isChecked():
                        color = 1
                    else:
                        color = 0
                    if device == 'ultraArm P340':
                        self.myCobot.set_coords(self.move_coords[color], 40)
                    else:
                        self.myCobot.send_coords(self.move_coords[color], 40, 0)
                    # self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
                    #                 [1]/1000.0, self.move_coords[color][2]/1000.0)
                    self.stop_wait(4)

                    # close pump

                    self.pump_off()

                    self.stop_wait(4)
                    if device == 'ultraArm P340':
                        self.myCobot.set_angles(self.move_angles[0], 25)
                    else:
                        self.myCobot.send_angles(self.move_angles[0], 25)
                    if not self.auto_mode_status:
                        self.place_status = False
                        self.btn_color(self.place_btn, 'blue')
                    # self._init_ = 20
                    # self.init_num = 0
                    # self.nparams = 0
                    self.num = 0
                    self.real_sx = self.real_sy = 0
            if self.auto_mode_status:
                self.is_pick = True
        except Exception as e:
            self.loger.info(e)

    def pump_on(self):
        """Start the suction pump"""
        if self.comboBox_device.currentText() in self.M5:
            if self.comboBox_device.currentText() == 'ultraArm P340':
                self.myCobot.set_gpio_state(0)
            else:
                self.myCobot.set_basic_output(2, 0)
                self.myCobot.set_basic_output(5, 0)
        else:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)

    def pump_off(self):
        """stop suction pump m5"""
        if self.comboBox_device.currentText() in self.M5:
            if self.comboBox_device.currentText() == 'ultraArm P340':
                self.myCobot.set_gpio_state(1)
            else:
                self.myCobot.set_basic_output(2, 1)
                self.myCobot.set_basic_output(5, 1)
        else:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)

    # The path to save the image folder
    def parse_folder(self, folder):
        """Retrieve Folder Pictures"""
        try:
            restore = []
            path = rf'{self.path[0]}/{folder}'

            for i, j, k in os.walk(path):
                for l in k:
                    restore.append(cv2.imread(path + '/{}'.format(l)))
            return restore
        except Exception as e:
            self.loger.error(str(e))

    def add_img(self):
        """Add the picture recognized by Keypoints"""
        try:
            if self.language == 1:
                self.open_camera_btn.setText('Close')
            else:
                self.open_camera_btn.setText('关闭')
            self.btn_color(self.open_camera_btn, 'red')
            class_name = "res"
            if (os.path.exists("res")):
                pass
            else:
                os.mkdir(class_name)
            index = 'takephoto'
            btn_text = self.add_img_btn.text()
            if btn_text == 'Add' or btn_text == '添加':
                if self.language == 1:
                    self.add_img_btn.setText('Cut')
                    self.prompts('Put the image you want to recognize into the camera area, and click the Cut button.')
                else:
                    self.add_img_btn.setText('剪切')
                    self.prompts('将要识别的图像放入相机区域，然后单击剪切按钮。')
                self.open_camera_func = 2
            elif btn_text == 'Cut' or btn_text == '剪切':
                self.cut_status = True
                if self.open_camera_func == 2:
                    return
            if not self.camera_status:
                self.open_camera()
            while self.camera_status:
                while self.camera_status:
                    QApplication.processEvents()
                    _, frame = self.cap.read()
                    if self.camera_status:
                        show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                 QtGui.QImage.Format_RGB888)
                        self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                    if self.cut_status:
                        cv2.imwrite(
                            "%s/%s.jpeg" % (class_name, index),
                            cv2.resize(frame, (600, 480), interpolation=cv2.INTER_AREA))
                        break
                if self.cut_status:
                    path = self.path[0]
                    path_red = path + '/res/D'
                    for i, j, k in os.walk(path_red):
                        file_len_red = len(k)

                    path_gray = path + '/res/B'
                    for i, j, k in os.walk(path_gray):
                        file_len_gray = len(k)

                    path_green = path + '/res/C'
                    for i, j, k in os.walk(path_green):
                        file_len_green = len(k)

                    path_blue = path + '/res/A'
                    for i, j, k in os.walk(path_blue):
                        file_len_blue = len(k)
                    if self.language == 1:
                        self.prompts("Please intercept the part to be recognized, and then press the enter key")
                    else:
                        self.prompts("请截取需要识别的部分，然后按回车键")
                    frame = cv2.imread(r"res/takephoto.jpeg")
                    cut = cv2.imread(r"res/takephoto.jpeg")

                    show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                             QtGui.QImage.Format_RGB888)
                    self.show_camera_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))

                    # Select ROIs
                    try:
                        roi = cv2.selectROI(windowName="Cut Image",
                                            img=frame,
                                            showCrosshair=False,
                                            fromCenter=False)
                        cv2.moveWindow("Cut Image", 798, 220)
                        frame = frame[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]
                        show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0], show.shape[1] * 3,
                                                 QtGui.QImage.Format_RGB888)
                        width = showImage.width()
                        height = showImage.height()
                        if width / 311 >= height / 151:
                            ratio = width / 311
                        else:
                            ratio = height / 151
                        new_width = width / ratio
                        new_height = height / ratio
                        showImage = showImage.scaled(new_width, new_height, Qt.KeepAspectRatio)
                        self.show_cutimg_lab.setPixmap(QtGui.QPixmap.fromImage(showImage))
                    except Exception as e:
                        self.loger.info(e)

                    x, y, w, h = roi
                    items = ["A", "B", "C", "D"]
                    if self.language == 1:
                        value, ok = QInputDialog.getItem(self, "Prompt", f"Please select a storage area:", items, 0,
                                                         True)
                    else:
                        value, ok = QInputDialog.getItem(self, "提示", f"请选择存储区域:", items, 0, True)

                    if not ok:
                        cv2.destroyAllWindows()
                        self.cut_status = False
                        continue
                    # Display ROI and save image
                    saved = False
                    if roi != (0, 0, 0, 0):

                        crop = cut[y:y + h, x:x + w]
                        # cv2.imshow('crop', crop)
                        # Select the D area folder
                        if value == 'D':
                            cv2.imwrite(path + '/res/D/goal{}.jpeg'.format(str(file_len_red + 1)), crop)
                            saved = True
                        # Select the B area folder
                        elif value == 'B':
                            cv2.imwrite(path + '/res/B/goal{}.jpeg'.format(str(file_len_gray + 1)), crop)
                            saved = True
                        # Select the C area folder
                        elif value == 'C':
                            cv2.imwrite(path + '/res/C/goal{}.jpeg'.format(str(file_len_green + 1)), crop)
                            saved = True
                        # Select the A area folder
                        elif value == 'A':
                            cv2.imwrite(path + '/res/A/goal{}.jpeg'.format(str(file_len_blue + 1)), crop)
                            saved = True
                    if saved:
                        if self.language == 1:
                            self.prompts('saved')
                        else:
                            self.prompts('已保存')
                        self.cut_status = False
                        # cv2.waitKey(0)
                        cv2.destroyAllWindows()

        except Exception as e:
            self.loger.error('Abnormal image interception：' + str(e))
            self.exit_add()

    def exit_add(self):
        if self.add_img_btn.text() == 'Cut' or self.add_img_btn.text() == '剪切':
            if self.language == 1:
                self.add_img_btn.setText('Add')
            else:
                self.add_img_btn.setText('添加')
        self.close_camera()
        self.prompts_lab.clear()
        self.cut_status = False

    def prompts(self, msg=None):
        """show prompts"""
        self.prompts_lab.clear()
        if msg is not None:
            if self.language == 1:
                self.prompts_lab.setText('Prmpt:\n' + msg)
            else:
                self.prompts_lab.setText('提示:\n' + msg)

    def combox_func_checked(self):
        self.algorithm_lab.setText(self.comboBox_function.currentText())
        self.prompts_lab.clear()
        self.offset_change()
        device = self.comboBox_device.currentText()
        if device == 'myCobot 280 for Pi' or device == 'myCobot 280 for M5':
            if self.comboBox_function.currentText() == 'yolov5':
                IS_CV_4 = cv2.__version__[0] == '4'
                if IS_CV_4:
                    self.net = cv2.dnn.readNet(self.modelWeights)
                    '''加载类别名'''
                    classesFile = self.path[0] + "/yolov5File/coco.names"
                    self.classes = None
                    with open(classesFile, 'rt') as f:
                        self.classes = f.read().rstrip('\n').split('\n')
                    self.cut_yolov5_img_status(True)
                else:
                    self.prompts('Load yolov5 model need the version of opencv is 4.')
                    self.comboBox_function.setCurrentIndex(0)
                    self.cut_yolov5_img_status()
            else:
                self.cut_yolov5_img_status()

        self.yolov5_count = 0

    def auto_mode(self):
        """automated operation"""
        btn = [self.discern_btn, self.crawl_btn, self.place_btn]
        if self.auto_mode_status:
            self.auto_mode_status = False
            self.discern_status = False
            self.crawl_status = False
            self.place_status = False
            self.is_pick = False
            self.btn_color(self.auto_btn, 'green')
            for b in btn:
                self.btn_color(b, 'blue')
        else:
            self.auto_mode_status = True
            self.discern_status = True
            self.crawl_status = True
            self.place_status = True
            self.is_pick = True
            self.btn_color(self.auto_btn, 'red')
            for b in btn:
                self.btn_color(b, 'red')

    def offset_change(self):
        try:
            """Get the offset according to the device"""

            func = self.comboBox_function.currentText()
            if func == 'QR code recognition' or func == '二维码识别':
                with open(self.path[0] + f'/offset/{self.comboBox_device.currentText()}_encode.txt', "r",
                          encoding="utf-8") as f:
                    offset = f.read().splitlines()
            else:
                with open(self.path[0] + f'/offset/{self.comboBox_device.currentText()}.txt', "r",
                          encoding="utf-8") as f:
                    offset = f.read().splitlines()
                self.camera_x, self.camera_y = int(eval(offset[0])[0]), int(eval(offset[0])[1])
            self.xoffset_edit.clear()
            self.yoffset_edit.clear()
            self.xoffset_edit.insert(f'{eval(offset[0])[0]}')
            self.yoffset_edit.insert(f'{eval(offset[0])[1]}')
        except Exception as e:
            self.loger.error(str(e))

    def insert_offsets(self):
        """write offset"""
        try:
            func = self.comboBox_function.currentText()
            x = self.xoffset_edit.text()
            y = self.yoffset_edit.text()
            if x is not None and (x.startswith('-') and x[1:] or x).isdigit() and y is not None and (
                    y.startswith('-') and y[1:] or y).isdigit():
                offset = [x, y]
                if func == 'QR code recognition' or func == '二维码识别':
                    with open(rf'{self.path[0]}/offset/{self.comboBox_device.currentText()}_encode.txt', "w",
                              encoding="utf-8") as file:
                        file.write(str(offset))
                else:
                    with open(rf'{self.path[0]}/offset/{self.comboBox_device.currentText()}.txt', "w",
                              encoding="utf-8") as file:
                        file.write(str(offset))
                    self.camera_x = int(x)
                    self.camera_y = int(y)
                if self.language == 1:
                    msg_box = QMessageBox(QMessageBox.Information, 'prompt', 'Successfully saved！')
                else:
                    msg_box = QMessageBox(QMessageBox.Information, '提示', '保存成功！')
                msg_box.exec_()
            else:
                if self.language == 1:
                    msg_box = QMessageBox(QMessageBox.Warning, 'Warning', 'The offset setting can only enter numbers！')
                else:
                    msg_box = QMessageBox(QMessageBox.Warning, '警告', '偏移量只允许输入整数！')
                msg_box.exec_()
        except Exception as e:
            self.loger.info(str(e))

    def open_file(self):
        """Open the folder where the file is located"""
        try:
            value = self.comboBox_device.currentText()
            if value in self.M5:
                os.startfile(self.path[0])
            else:
                os.system('xdg-open ' + self.path[0])
            # self.file_window = fileWindow()
            # self.file_window.show()
        except Exception as e:
            self.loger.info(str(e))

    def get_img_coord(self):
        if self.img_coord_status:
            self.img_coord_status = False
            self.btn_color(self.image_coord_btn, 'green')
            self.img_coord_lab.clear()
        else:
            self.img_coord_status = True
            self.btn_color(self.image_coord_btn, 'red')

    def get_current_coord_btnClick(self):
        if not self.has_mycobot():
            return
        if self.current_coord_status:
            self.current_coord_status = False
            self.btn_color(self.current_coord_btn, 'green')
            self.cuttent_coord_lab.clear()
        else:
            self.current_coord_status = True
            self.btn_color(self.current_coord_btn, 'red')
            get_coord = threading.Thread(target=self.get_current_coord)
            get_coord.start()

    def get_current_coord(self):
        while self.current_coord_status:
            QApplication.processEvents()
            try:
                coord = self.myCobot.get_coords()
            except:
                coord = []
                pass
            if coord != [] and coord is not None:
                if len(coord) == 6:
                    coord = f'X:{coord[0]} Y:{coord[1]} Z:{coord[2]} Rx:{coord[3]} Ry:{coord[4]} Rz:{coord[5]}'
                else:
                    coord = f'X:{coord[0]} Y:{coord[1]} Z:{coord[2]} Θ:{coord[3]}'
                self.cuttent_coord_lab.clear()
                if self.current_coord_status:
                    self.cuttent_coord_lab.setText(str(coord))
            # else:
            #     self.cuttent_coord_lab.clear()
            #     if self.current_coord_status:
            #         if self.language == 1:
            #             self.cuttent_coord_lab.setText('Fetch failed')
            #         else:
            #             self.cuttent_coord_lab.setText('获取失败')
            self.stop_wait(0.2)

    def btn_color(self, btn, color):
        if color == 'red':
            btn.setStyleSheet("background-color: rgb(231, 76, 60);\n"
                              "color: rgb(255, 255, 255);\n"
                              "border-radius: 10px;\n"
                              "border: 2px groove gray;\n"
                              "border-style: outset;")
        elif color == 'green':
            btn.setStyleSheet("background-color: rgb(39, 174, 96);\n"
                              "color: rgb(255, 255, 255);\n"
                              "border-radius: 10px;\n"
                              "border: 2px groove gray;\n"
                              "border-style: outset;")
        elif color == 'blue':
            btn.setStyleSheet("background-color: rgb(41, 128, 185);\n"
                              "color: rgb(255, 255, 255);\n"
                              "border-radius: 10px;\n"
                              "border: 2px groove gray;\n"
                              "border-style: outset;")

    def set_language(self):
        self.is_language_btn_click = True
        if self.language == 1:
            self.language = 2
            self.btn_color(self.language_btn, 'blue')
        else:
            self.language = 1
            self.btn_color(self.language_btn, 'green')
        self._init_language()
        self.combox_func_checked()
        self.is_language_btn_click = False
        with open(rf'{self.path[0]}/offset/language.txt', "w",
                  encoding="utf-8") as file:
            file.write(str(self.language))
        self._init_tooltip()

    def _init_language(self):
        _translate = QtCore.QCoreApplication.translate
        if self.language == 1:
            if not self.is_language_btn_click:
                return
            self.camara_show.setText(_translate("AiKit_UI", "Camera"))
            if self.open_camera_btn.text() == '打开':
                self.open_camera_btn.setText(_translate("AiKit_UI", "Open"))
            else:
                self.open_camera_btn.setText(_translate("AiKit_UI", "Close"))
            self.connect_lab.setText(_translate("AiKit_UI", "Connection"))
            if self.connect_btn.text() == '连接':
                self.connect_btn.setText(_translate("AiKit_UI", "CONNECT"))
            else:
                self.connect_btn.setText(_translate("AiKit_UI", "DISCONNECT"))
            self.device_lab.setText(_translate("AiKit_UI", "Device"))
            self.baud_lab.setText(_translate("AiKit_UI", "Baud"))
            self.port_lab.setText(_translate("AiKit_UI", "Serial Port"))
            self.func_lab.setText(_translate("AiKit_UI", "Control"))
            self.auto_btn.setText(_translate("AiKit_UI", "Auto Mode"))
            self.to_origin_btn.setText(_translate("AiKit_UI", "Go"))
            self.func_lab_2.setText(_translate("AiKit_UI", "Homing"))
            self.func_lab_4.setText(_translate("AiKit_UI", "Recognition"))
            self.discern_btn.setText(_translate("AiKit_UI", "Run"))
            self.func_lab_5.setText(_translate("AiKit_UI", "Algorithm:"))
            self.crawl_btn.setText(_translate("AiKit_UI", "Run"))
            self.func_lab_7.setText(_translate("AiKit_UI", "Pick"))
            self.func_lab_8.setText(_translate("AiKit_UI", "Place"))
            self.place_btn.setText(_translate("AiKit_UI", "Run"))
            self.radioButton_A.setText(_translate("AiKit_UI", "BinA"))
            self.radioButton_B.setText(_translate("AiKit_UI", "BinB"))
            self.radioButton_C.setText(_translate("AiKit_UI", "BinC"))
            self.radioButton_D.setText(_translate("AiKit_UI", "BinD"))
            self.open_file_btn.setText(_translate("AiKit_UI", "Open File"))
            self.func_lab_9.setText(_translate("AiKit_UI", "Algorithm"))
            self.func_lab_10.setText(_translate("AiKit_UI", "Select"))
            self.comboBox_function.setItemText(0, _translate("AiKit_UI", "Color recognition"))
            self.comboBox_function.setItemText(1, _translate("AiKit_UI", "shape recognition"))
            self.comboBox_function.setItemText(2, _translate("AiKit_UI", "QR code recognition"))
            self.comboBox_function.setItemText(3, _translate("AiKit_UI", "Keypoints"))
            self.func_lab_11.setText(_translate("AiKit_UI", "Add New Pictures"))
            if self.add_img_btn.text() == '添加':
                self.add_img_btn.setText(_translate("AiKit_UI", "Add"))
            else:
                self.add_img_btn.setText(_translate("AiKit_UI", "Cut"))
            self.exit_add_btn.setText(_translate("AiKit_UI", "Exit"))
            self.func_lab_6.setText(_translate("AiKit_UI", "XY Offsets"))
            self.offset_save_btn.setText(_translate("AiKit_UI", "Save"))
            self.func_lab_12.setText(_translate("AiKit_UI", "X offset:"))
            self.func_lab_13.setText(_translate("AiKit_UI", " Y offset:"))
            self.connect_lab_3.setText(_translate("AiKit_UI", "Display"))
            self.current_coord_btn.setText(_translate("AiKit_UI", "  current coordinates"))
            self.image_coord_btn.setText(_translate("AiKit_UI", "  image coordinates"))
            self.language_btn.setText(_translate("AiKit_UI", "简体中文"))
            self.yolov5_cut_btn.setText(_translate("AiKit_UI", "Cut"))
        else:
            self.camara_show.setText(_translate("AiKit_UI", "相机"))
            if self.is_language_btn_click:
                if self.open_camera_btn.text() == 'Open':
                    self.open_camera_btn.setText(_translate("AiKit_UI", "打开"))
                else:
                    self.open_camera_btn.setText(_translate("AiKit_UI", "关闭"))
            else:
                self.open_camera_btn.setText(_translate("AiKit_UI", "打开"))
            self.connect_lab.setText(_translate("AiKit_UI", "连接"))
            if self.is_language_btn_click:
                if self.connect_btn.text() == 'CONNECT':
                    self.connect_btn.setText(_translate("AiKit_UI", "连接"))
                else:
                    self.connect_btn.setText(_translate("AiKit_UI", "断开"))
            else:
                self.connect_btn.setText(_translate("AiKit_UI", "连接"))
            self.device_lab.setText(_translate("AiKit_UI", "设备"))
            self.baud_lab.setText(_translate("AiKit_UI", "波特率"))
            self.port_lab.setText(_translate("AiKit_UI", "串口"))
            self.func_lab.setText(_translate("AiKit_UI", "控制"))
            self.auto_btn.setText(_translate("AiKit_UI", "全自动"))
            self.to_origin_btn.setText(_translate("AiKit_UI", "运行"))
            self.func_lab_2.setText(_translate("AiKit_UI", "初始点"))
            self.func_lab_4.setText(_translate("AiKit_UI", "识别"))
            self.discern_btn.setText(_translate("AiKit_UI", "运行"))
            self.func_lab_5.setText(_translate("AiKit_UI", "算法:"))
            self.crawl_btn.setText(_translate("AiKit_UI", "运行"))
            self.func_lab_7.setText(_translate("AiKit_UI", "抓取"))
            self.func_lab_8.setText(_translate("AiKit_UI", "放置"))
            self.place_btn.setText(_translate("AiKit_UI", "运行"))
            self.open_file_btn.setText(_translate("AiKit_UI", "打开文件"))
            self.func_lab_9.setText(_translate("AiKit_UI", "算法"))
            self.func_lab_10.setText(_translate("AiKit_UI", "选择"))
            self.comboBox_function.setItemText(0, _translate("AiKit_UI", "颜色识别"))
            self.comboBox_function.setItemText(1, _translate("AiKit_UI", "形状识别"))
            self.comboBox_function.setItemText(2, _translate("AiKit_UI", "二维码识别"))
            self.comboBox_function.setItemText(3, _translate("AiKit_UI", "特征点识别"))
            self.func_lab_11.setText(_translate("AiKit_UI", "添加新图片"))
            if self.is_language_btn_click:
                if self.add_img_btn.text() == 'Add':
                    self.add_img_btn.setText(_translate("AiKit_UI", "添加"))
                else:
                    self.add_img_btn.setText(_translate("AiKit_UI", "剪切"))
            else:
                self.add_img_btn.setText(_translate("AiKit_UI", "添加"))
            self.exit_add_btn.setText(_translate("AiKit_UI", "退出"))
            self.func_lab_6.setText(_translate("AiKit_UI", "XY 坐标偏移"))
            self.offset_save_btn.setText(_translate("AiKit_UI", "保存"))
            self.func_lab_12.setText(_translate("AiKit_UI", "X 偏移量:"))
            self.func_lab_13.setText(_translate("AiKit_UI", " Y 偏移量:"))
            self.connect_lab_3.setText(_translate("AiKit_UI", "坐标显示"))
            self.current_coord_btn.setText(_translate("AiKit_UI", "  实时坐标"))
            self.image_coord_btn.setText(_translate("AiKit_UI", "  定位坐标"))
            self.language_btn.setText(_translate("AiKit_UI", "English"))
            self.yolov5_cut_btn.setText(_translate("AiKit_UI", "剪切"))

    def go_zero(self):
        self.myCobot.go_zero()
        if self.language == 1:
            self.prompts('Zero calibration completed')
        else:
            self.prompts('回零校正已完成')
        self.btn_status(True)
        self.connect_btn.setEnabled(True)

        '''绘制类别'''

    def draw_label(self, img, label, x, y):
        text_size = cv2.getTextSize(label, self.FONT_FACE, self.FONT_SCALE, self.THICKNESS)
        dim, baseline = text_size[0], text_size[1]
        cv2.rectangle(img, (x, y), (x + dim[0], y + dim[1] + baseline), (0, 0, 0), cv2.FILLED)
        cv2.putText(img, label, (x, y + dim[1]), self.FONT_FACE, self.FONT_SCALE, self.YELLOW, self.THICKNESS)

    # detect object
    def post_process(self, input_image):
        class_ids = []
        confidences = []
        boxes = []
        blob = cv2.dnn.blobFromImage(input_image, 1 / 255, (self.INPUT_HEIGHT, self.INPUT_WIDTH), [0, 0, 0], 1,
                                     crop=False)
        # Sets the input to the network.
        self.net.setInput(blob)
        # Run the forward pass to get output of the output layers.
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())

        rows = outputs[0].shape[1]
        image_height, image_width = input_image.shape[:2]

        x_factor = image_width / self.INPUT_WIDTH
        y_factor = image_height / self.INPUT_HEIGHT
        # 像素中心点
        cx = 0
        cy = 0
        # 循环检测
        try:
            for r in range(rows):
                row = outputs[0][0][r]
                confidence = row[4]
                if confidence > self.CONFIDENCE_THRESHOLD:
                    classes_scores = row[5:]
                    class_id = np.argmax(classes_scores)
                    if (classes_scores[class_id] > self.SCORE_THRESHOLD):
                        confidences.append(confidence)
                        class_ids.append(class_id)
                        cx, cy, w, h = row[0], row[1], row[2], row[3]
                        left = int((cx - w / 2) * x_factor)
                        top = int((cy - h / 2) * y_factor)
                        width = int(w * x_factor)
                        height = int(h * y_factor)
                        box = np.array([left, top, width, height])
                        boxes.append(box)

                        '''非极大值抑制来获取一个标准框'''
                        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)

                        for i in indices:
                            box = boxes[i]
                            left = box[0]
                            top = box[1]
                            width = box[2]
                            height = box[3]

                            # 描绘标准框
                            cv2.rectangle(input_image, (left, top), (left + width, top + height), self.BLUE,
                                          3 * self.THICKNESS)

                            # 像素中心点
                            cx = left + (width) // 2
                            cy = top + (height) // 2

                            cv2.circle(input_image, (cx, cy), 5, self.BLUE, 10)

                            # 检测到的类别
                            label = "{}:{:.2f}".format(self.classes[class_ids[i]], confidences[i])
                            # 绘制类real_sx, real_sy, detect.color)

                            self.draw_label(input_image, label, left, top)

                # cv2.imshow("nput_frame",input_image)
        # return input_image
        except Exception as e:
            print(e)
            exit(0)

        if cx + cy > 0:
            return cx, cy, input_image
        else:
            return None

    def cut_yolov5_img(self):
        if not self.is_yolov5_cut_btn_clicked:
            self.is_yolov5_cut_btn_clicked = True
            self.yolov5_is_not_pick = True

    def cut_yolov5_img_status(self, status=False):
        if status:
            # 设置透明度的值，0.0到1.0，最小值0是透明，1是不透明
            op = QtWidgets.QGraphicsOpacityEffect()
            op.setOpacity(1)
            self.yolov5_cut_btn.setGraphicsEffect(op)
            self.yolov5_cut_btn.setEnabled(True)
            self.add_img_btn.setEnabled(False)
            self.exit_add_btn.setEnabled(False)
        else:
            # 设置透明度的值，0.0到1.0，最小值0是透明，1是不透明
            op = QtWidgets.QGraphicsOpacityEffect()
            op.setOpacity(0)
            self.yolov5_cut_btn.setGraphicsEffect(op)
            self.yolov5_cut_btn.setEnabled(False)
            self.add_img_btn.setEnabled(True)
            self.exit_add_btn.setEnabled(True)


# File loading window displayed
# class fileWindow(file_window,QMainWindow,QWidget):
#     def __init__(self):
#         try:
#             super(fileWindow,self).__init__()
#             self.setupUi(self)
#             self.setWindowTitle('File')
#             self.save_file_btn.clicked.connect(self.save)
#             self.cancel_btn.clicked.connect(self.cancel)
#             self.path = os.path.split(os.path.abspath(__file__))
#             with open(self.path[0] + f'/main.py', "r", encoding="utf-8") as f:
#                 offset = f.read()
#             self.change_file_lineEdit.setPlainText(offset)
#         except Exception as e:
#             print(str(e))
#
#     def save(self):
#         try:
#             with open(rf'{self.path[0]}/main.py', "w", encoding="utf-8") as file:
#                 file.write(self.change_file_lineEdit.toPlainText())
#             msg_box = QMessageBox(QMessageBox.Information, 'prompt', 'Successfully saved！')
#             msg_box.exec_()
#             self.cancel()
#         except Exception as e:
#             print(str(e))
#
#     def cancel(self):
#         self.close()

# class GetCoordThread(QThread):
#     trigger = pyqtSignal(str)
#     def __init__(self):
#         super(GetCoordThread,self).__init__()

# def start(self):
#     status = True
#     while status:
#         pass


if __name__ == '__main__':
    app = QApplication(sys.argv)
    AiKit_window = AiKit_APP()
    AiKit_window.show()
    sys.exit(app.exec_())

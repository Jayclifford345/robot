#!/usr/bin/env python3
import cv2
import numpy as np
import time
import os,sys
import platform

from pymycobot.mycobot import MyCobot
from pymycobot import MyCobotSocket
import threading
from threading import Lock
from opcua import Server
lock = Lock()


IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"
# Adaptive seeed

# Monitoring Robot Class
class Monitor_Robot():
    NUM_JOINTS = 6  # Class constant for number of joints

    def __init__(self, mc):
        self.mc = mc
        self.server = Server()
        self.server.set_endpoint("opc.tcp://0.0.0.0:4840/robot/server/")
        
        # Setup our own namespace
        uri = "http://robot.monitor"
        idx = self.server.register_namespace(uri)

        # Create a new object in the server
        robot_obj = self.server.nodes.objects.add_object(idx, "RobotMonitor")
        
        # Add speed and moving variables to the RobotMonitor object
        self.speed_var = robot_obj.add_variable(idx, "speed", 0)
        self.moving_var = robot_obj.add_variable(idx, "moving", 0)
        self.speed_var.set_writable()
        self.moving_var.set_writable()
        
        # Create joint objects and their respective variables
        self.joint_objects = {}
        for i in range(self.NUM_JOINTS):
            joint_obj = robot_obj.add_object(idx, f"joint{i+1}")
            joint_data = {
                'angle': joint_obj.add_variable(idx, "angle", 0),
                'coord': joint_obj.add_variable(idx, "coord", 0.0),
                'radian': joint_obj.add_variable(idx, "radian", 0.0),
                'error': joint_obj.add_variable(idx, "error", 0),
                'servo_speed': joint_obj.add_variable(idx, "servo_speed", 0),
                'servo_voltage': joint_obj.add_variable(idx, "servo_voltage", 0.0),
                'servo_temp': joint_obj.add_variable(idx, "servo_temp", 0.0),
            }
            for var in joint_data.values():
                var.set_writable()
            self.joint_objects[f"joint{i+1}"] = joint_data

        # Start the server
        self.start_server()

            
    def start_server(self):
        self.server.start()
        print("Server started at {}".format(self.server.endpoint))
    
    def stop_server(self):
        self.server.stop()

    def monitor(self):
        data = {
                    'speed': None,
                    'angles': None,
                    'coords': None,
                    'radians': None,
                    'error': None,
                    'moving': None,
                    'servo_speeds': None,
                    'servo_voltage': None,
                    'servo_temp': None,
                }
        while True:
            if lock.acquire(blocking=False):
                try:
                    # Update the data dictionary
                    data['speed'] = self.mc.get_speed()
                    data['angles'] = self.mc.get_angles()
                    data['coords'] = self.mc.get_coords()
                    data['radians'] = self.mc.get_radians()
                    data['error'] = self.mc.read_next_error()
                    data['moving'] = self.mc.is_moving()
                    data['servo_speeds'] = self.mc.get_servo_speeds()
                    data['servo_voltage'] = self.mc.get_servo_voltages()
                    data['servo_temp'] = self.mc.get_servo_temps()

                    #print(data)

                except Exception as e:
                    # Log the error for debugging
                    print(f"Error: {e}")
                finally:
                    # Always release the lock when done
                    lock.release()
                
                try:
                    # Update speed and moving at RobotMonitor level
                    self.speed_var.set_value(data['speed'])
                    self.moving_var.set_value(data['moving'])
                    for i in range(self.NUM_JOINTS):
                        joint_name = f"joint{i+1}"
                        self.joint_objects[joint_name]['angle'].set_value(data['angles'][i])
                        self.joint_objects[joint_name]['coord'].set_value(data['coords'][i])
                        self.joint_objects[joint_name]['radian'].set_value(data['radians'][i])
                        self.joint_objects[joint_name]['error'].set_value(data['error'][i])
                        self.joint_objects[joint_name]['servo_speed'].set_value(data['servo_speeds'][i])
                        self.joint_objects[joint_name]['servo_voltage'].set_value(data['servo_voltage'][i])
                        self.joint_objects[joint_name]['servo_temp'].set_value(data['servo_temp'][i])
                except Exception as e:
                    # Log the error for debugging
                    print(f"Error: {e}")

                # Sleep for the next iteration
                time.sleep(2)
    

def execute_with_retry(lock, action, max_retries=5, retry_delay=0.5):
    """
    Try to acquire the lock and execute the action. If the lock cannot be acquired,
    it will retry for max_retries times, waiting retry_delay seconds between attempts.

    :param lock: The lock object.
    :param action: The callable to execute when the lock is acquired.
    :param max_retries: The maximum number of retries if the lock is not acquired.
    :param retry_delay: The delay between retries.
    """
    for _ in range(max_retries):
        if lock.acquire():
            try:
                action()
                break
            finally:
                lock.release()
        else:
            time.sleep(retry_delay)

class Object_detect():

    def __init__(self, camera_x = 145, camera_y = 7):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare mecharm 270
        self.mc = None
        self.monitor = None
        self.block = "A"

        # 移动角度
        self.move_angles = [
            [0, 0, 0, 0, 90, 0],  # init the point
            [-33.31, 2.02, -10.72, -0.08, 95, -54.84],  # point to grab
        ]

        # 移动坐标
        self.move_coords = [
            [96.5, -101.9, 185.6, 155.25, 19.14, 75.88], # D Sorting area
            [180.9, -99.3, 184.6, 124.4, 30.9, 80.58], # C Sorting area
            [77.4, 122.1, 179.2, 151.66, 17.94, 178.24], # A Sorting area
            [168.3, 130.4, 164.2, 156.56, 49.61, -169.69], # B Sorting area
        ]

        self.loading_bay = {
         "above_bin" :   [-3.7, 162.6, 168.7, -172.36, 11.31, -89.51], #above bin
         "a" :   [-34.3, 179.3, 92.4, -173.67, -0.5, -88.08], #in bin (box 1)
         "transition" : [18.2, 141.0, 209.4, -162.17, 29.67, -55.98], #before moving to clear
         "b" :  [10.7, 181.2, 92.0, -172.08, -1.53, -101.09], #in bin (box 2)
         "scan" :[153.3, 2.5, 111.3, -176.27, 0.31, -113.13], # place box in scan area
         "clear": [50.1, -71.5, 209.3, -140.53, 43.85, 173.33] # move robot from scan area
        }
        
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
        self.robot_jes = os.popen("ls /dev/ttyTHS1").readline()[:-1]
        self.raspi = False
        if "dev" in self.robot_m5:
            self.Pin = [2, 5]
        elif "dev" in self.robot_wio:
            # self.Pin = [20, 21]
            self.Pin = [2, 5]

            # for i in self.move_coords:
            #     i[2] -= 20
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            GPIO.output(20, 1)
            GPIO.output(21, 1)
            self.raspi = True
        if self.raspi:
            self.gpio_status(False)

        # choose place to set cube 选择放置立方体的地方
        self.color = 0
        # parameters to calculate camera clipping parameters 计算相机裁剪参数的参数
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord 设置真实坐标的缓存
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            # "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
            # "yellow": [np.array([22, 93, 0]), np.array([45, 255, 245])],
            "yellow": [np.array([26, 43, 46]), np.array([34, 255, 255])],
            "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
            "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }
       
        # use to calculate coord between cube and mecharm 270
        # 用于计算立方体和 mycobot 之间的坐标
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mecharm270
        # 抓取中心点相对于 mycobot 的坐标
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mecharm270
        # 立方体相对于 mycobot 的坐标
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        # 像素与实际值的比值
        self.ratio = 0
        
        # Get ArUco marker dict that can be detected.
        # 获取可以检测到的 ArUco 标记字典。
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params. 获取 ArUco 标记参数
        self.aruco_params = cv2.aruco.DetectorParameters_create()


    # pump_control pi
    def gpio_status(self, flag):
        if flag:
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)
    
    

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mecharm270
        print(color)

        action = lambda: self.mc.send_coords(self.move_coords[color], 60, 0)
        execute_with_retry(lock, action)
        
        time.sleep(2)

        # send coordinates to move mycobot
        action = lambda: self.mc.send_coords([x, y, 150, -176.1, 2.4, -125.1], 60, 0) # usb :rx,ry,rz -173.3, -5.48, -57.9
        execute_with_retry(lock, action)
    
        time.sleep(2)
        
        # self.mc.send_coords([x, y, 150, 179.87, -3.78, -62.75], 25, 0)
        # time.sleep(3)

        # self.mc.send_coords([x, y, 103, 179.87, -3.78, -62.75], 25, 0)

        action = lambda: self.mc.send_coords([x, y, 108, -176.1, 2.4, -125.1], 60, 0) # -178.77, -2.69, 40.15     pi
        execute_with_retry(lock, action)

        time.sleep(2.5)

        # open pump
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_on()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(True)
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp:
                with lock: 
                    tmp = self.mc.get_angles()    
            else:
                break
        time.sleep(0.5)
        
        print(tmp)
        action = lambda: self.mc.send_angles([tmp[0], 17.22, -32.51, tmp[3], 97, tmp[5]],60) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        execute_with_retry(lock, action)
    
        time.sleep(2)

        action = lambda: self.mc.send_coords(self.move_coords[color], 60, 0)
        execute_with_retry(lock, action)
        # self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
        #                 [1]/1000.0, self.move_coords[color][2]/1000.0)
        time.sleep(2)
       
        # close pump
 
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_off()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(False)
        time.sleep(2)

        action = lambda: self.mc.send_angles(self.move_angles[1], 75)
        execute_with_retry(lock, action)
        time.sleep(2)

    # decide whether grab cube 决定是否抓取立方体
    def decide_move(self, x, y, color):
        print(x, y, self.cache_x, self.cache_y, color)
        # detect the cube status move or run 检测立方体状态移动或运行
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm

            self.cache_x, self.cache_y = x, y
            return False
        else:

            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x, y, color)
            return True

    def load_bay(self):
        
        action = lambda: self.mc.send_coords(self.loading_bay["above_bin"], 60)
        execute_with_retry(lock, action)
        time.sleep(2)
        if self.block=="A":
            action = lambda: self.mc.send_coords(self.loading_bay["a"], 60)
            execute_with_retry(lock, action)
            self.block="B"
        elif self.block=="B":
            action = lambda: self.mc.send_coords(self.loading_bay["b"], 60)
            execute_with_retry(lock, action)
            self.block="A"
        time.sleep(2)
        self.gpio_status(True)
        time.sleep(2)
        action = lambda: self.mc.send_coords(self.loading_bay["above_bin"], 30)
        execute_with_retry(lock, action)
        time.sleep(2)
        action = lambda: self.mc.send_coords(self.loading_bay["transition"], 40)
        execute_with_retry(lock, action)
        time.sleep(1)
        action = lambda: self.mc.send_coords(self.loading_bay["scan"], 50)
        execute_with_retry(lock, action)
        time.sleep(3)
        self.gpio_status(False)
        time.sleep(2)
        action = lambda: self.mc.send_coords(self.loading_bay["clear"], 60)
        execute_with_retry(lock, action)
        time.sleep(2)

       

    # init mecharm270
    def run(self):
        if "dev" in self.robot_wio :
            self.mc = MyCobot(self.robot_wio, 115200) 
        elif "dev" in self.robot_m5:
            self.mc = MyCobot(self.robot_m5, 115200) 
        elif "dev" in self.robot_raspi:
          self.mc = MyCobot(self.robot_raspi, 1000000)
          #self.mc = MyCobotSocket('10.42.0.1', 9000)
        if not self.raspi:
            self.pub_pump(False, self.Pin)
        action = lambda: self.mc.send_angles([-33.31, 2.02, -10.72, -0.08, 95, -54.84], 70)
        execute_with_retry(lock, action)

        self.monitor = Monitor_Robot(self.mc)

        # Start the monitoring thread
        monitor_thread = threading.Thread(target=self.monitor.monitor)
        monitor_thread.start()
        
        time.sleep(3)

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img 在 img 上绘制矩形
        cv2.rectangle( 
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    # get points of two aruco 获得两个 aruco 的点位
    def get_calculate_params(self, img):
        # Convert the image to a gray image 将图像转换为灰度图像
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
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
                # print('point_11:', point_11)
                # print('point_21:', point_21)
                # print('point_31:', point_31)
                # print('point_41:', point_41)
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)
                
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters 设置相机裁剪参数 
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)

    # set parameters to calculate the coords between cube and mecharm270
    # 设置参数以计算立方体和 mycobot 之间的坐标
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    # calculate the coords between cube and mecharm270
    # 计算立方体和 mycobot 之间的坐标
    def get_position(self, x, y):
        return ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """
    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2*0.78):int(self.y1*1.1),
                          int(self.x1*0.86):int(self.x2*1.08)]
        return frame

    # detect cube color
    def color_detect(self, img):
        # set the arrangement of color'HSV
        x = y = 0
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for mycolor, item in self.HSV.items():
            # redLower = np.array(item[0])
            # redUpper = np.array(item[1])

            # transfrom the img to model of gray 将图像转换为灰度模型
            # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # wipe off all color expect color in range 擦掉所有颜色期望范围内的颜色
            mask = cv2.inRange(hsv, item[0], item[1])

            # a etching operation on a picture to remove edge roughness
            # 对图片进行蚀刻操作以去除边缘粗糙度
            erosion = cv2.erode(mask, np.ones((1, 1), np.uint8), iterations=2)

            # the image for expansion operation, its role is to deepen the color depth in the picture
            # 用于扩展操作的图像，其作用是加深图片中的颜色深度
            dilation = cv2.dilate(erosion, np.ones(
                (1, 1), np.uint8), iterations=2)

            # adds pixels to the image 向图像添加像素
            target = cv2.bitwise_and(img, img, mask=dilation)

            # the filtered image is transformed into a binary image and placed in binary
            # 将过滤后的图像转换为二值图像并放入二值
            ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)

            # get the contour coordinates of the image, where contours is the coordinate value, here only the contour is detected
            # 获取图像的轮廓坐标，其中contours为坐标值，这里只检测轮廓
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
                    # find the largest object that fits the requirements 找到符合要求的最大对象
                    c = max(contours, key=cv2.contourArea)
                    # get the lower left and upper right points of the positioning object
                    # 获取定位对象的左下和右上点
                    x, y, w, h = cv2.boundingRect(c)
                    # locate the target by drawing rectangle 通过绘制矩形来定位目标
                    cv2.rectangle(img, (x, y), (x+w, y+h), (153, 153, 0), 2)
                    # calculate the rectangle center 计算矩形中心
                    x, y = (x*2+w)/2, (y*2+h)/2
                    # calculate the real coordinates of mecharm270 relative to the target
                    #  计算 mycobot 相对于目标的真实坐标
                    
                    if mycolor  == "yellow":
                        
                        self.color = 3
                        break

                    elif mycolor == "red":
                        self.color = 0
                        break

                    elif mycolor == "cyan":
                        self.color = 2
                        break

                    elif mycolor == "blue":
                        self.color =2
                        break
                    elif mycolor == "green":
                        self.color = 1
                        break
        
        # 判断是否正常识别
        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None

def initialize_detection_area(detect, cap):
    _init_ = 20  
    init_num = 0
    nparams = 0
    while cv2.waitKey(1) < 0:
       # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)
        if _init_ > 0:
            _init_ -= 1
            continue

        # calculate the parameters of camera clipping 计算相机裁剪的参数
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
               # cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mecharm270 计算立方体和 mycobot 之间坐标的参数
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
               # cv2.imshow("figure", frame)
                print("No frame")
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                print(nparams)
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mecharm270
            # 计算和设置计算立方体和mycobot之间真实坐标的参数
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print("ok")
            break
                # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()



if __name__ == "__main__":

    # open the camera
    if platform.system() == "Windows":
        cap_num = 1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        
        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = 0
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        
        if not cap.isOpened():
            cap.open()
            
    # init a class of Object_detect
    detect = Object_detect()
    # init mecharm270
    detect.run()

    initialize_detection_area(detect, cap)
   
        
    while True:
        # Load a block into the detection area
        print("Loading...", flush=True)
        detect.load_bay()
        time.sleep(2)  # Ensure there's enough time for block to be placed before detection starts
        
        # Initialize or reset necessary variables for detection loop
        no_result = 0
        num = real_sx = real_sy = 0
        
        # Detection and moving loop
        while cv2.waitKey(1) < 0:
            _, frame = cap.read()
            frame = detect.transform_frame(frame)
            if frame is None:  # Check if frame is captured correctly
                print("Failed to capture frame")
        

            # get detect result 获取检测结果
            print("Detecting...", flush=True)
            detect_result = detect.color_detect(frame)
            if detect_result is None:
            #  cv2.imshow("figure", frame)
                if no_result < 10:
                    no_result += 1
                    print(f"no result: {no_result}", flush=True)
                    time.sleep(0.1)
                else:
                    print(f"Can't find block returning to loading", flush=True)
                    break
            else:
                x, y = detect_result
                # calculate real coord between cube and mecharm270 计算立方体和 mycobot 之间的真实坐标
                real_x, real_y = detect.get_position(x, y)
                # print('real_x',round(real_x, 3),round(real_y, 3))
                if num == 20:
                    print("Detection found decided to move...", flush=True)
                    result = detect.decide_move(real_sx/20.0, real_sy/20.0, detect.color)
                    if result == True:
                        print("Moving...", flush=True)
                        num = real_sx = real_sy = 0
                    else:
                        print("Detection failed, retrying...", flush=True)
                        

                else:
                    print("Calculating Detection...", flush=True)
                    num += 1
                    real_sy += real_y
                    real_sx += real_x

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                sys.exit()


      #  cv2.imshow("figure", frame)





# -*- coding: utf-8 -*-
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
#from _future_ import print_function
import time
import cv2 as cv
import numpy as np
import math
import time
import RPi.GPIO as GPIO
import sys
PIN = 19
PIN2 = 21

import threading

n = 0

def follow():
    global n
    
    mode=GPIO.getmode()
    GPIO.cleanup()
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.setup(11, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    p11 = GPIO.PWM(11,30)
    p13 = GPIO.PWM(13,30)
    p11.start(0)
    p13.start(0)
    GPIO.setup(7, GPIO.OUT)
    GPIO.setup(15, GPIO.OUT)
    p7 = GPIO.PWM(7, 30)
    p15 = GPIO.PWM(15, 30)

    GPIO.output(11, GPIO.LOW)  
    GPIO.output(13, GPIO.LOW)
    GPIO.output(7, GPIO.LOW)  
    GPIO.output(15, GPIO.LOW)
    while True:
        pin_status = GPIO.input(PIN)
        pin_status2 = GPIO.input(PIN2)
        print(pin_status, " ", pin_status2)
        
        if (n==1):
            print("Знак Stop")
            p7.start(0)
            p15.start(0)
            rawCapture.truncate(0)
            cv.waitKey(100000)
            
        else:
            p15.start(25)
            p7.start(15)
            
            if (pin_status == 0) and (pin_status2 == 1):
                p7.start(15)
                p15.start(0)
              #  print("right")

            elif (pin_status2 == 0) and (pin_status == 1):
                p7.start(0)
                p15.start(25)
             #   print("left")
            
            elif (pin_status == 0) and (pin_status2 == 0):
                p15.start(25)
                p7.start(15)
                
         #    else:
         #       p7.start(60)
         #       p15.start(85)

        time.sleep(0.02)

        
t = threading.Thread(target=follow)
t.daemon = True
t.start()
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.awb_mode = 'fluorescent'

# allow the camera to warmup
time.sleep(0.1)


 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
 
    # show the frame
    #cv.imshow("Frame", img)
    key = cv.waitKey(1) & 0xFF
 
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
    	break


    # Загружаем картинку из файла
   # img = cv.imread("4.jpg", cv.IMREAD_COLOR)
   # img = cv.resize(img, (500, 600))
    templ = cv.imread("/home/pi/Downloads/stopshab.jpg")
    templ = cv.resize(templ, (400, 400))
    image_window = "Source Image"
    result_window = "Result window"
    max_Trackbar = 5
    use_mask = False
    mask = None
    match_method = 1

   # cv.imshow("Processing", img)
  #  cv.waitKey()

    # Преобразуем картинку из формата RGB в формат HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Нужно отбросить все цвета, кроме синего:
    #  Задаем диапазон допустимых цветов:
   # h_range1 = (0, 0)
    h_range2 = (170, 180)     # синий
    s_range = (100, 255)        # яркий, насыщенный
    v_range = (50, 255)

    # Бинаризуем картинку по указанному цвету
    lower1  = (h_range2[0], s_range[0], v_range[0])
    higher1 = (h_range2[1], s_range[1], v_range[1])
    red_orange = cv.inRange(hsv, lower1, higher1)

    lower2  = (h_range2[0], s_range[0], v_range[0])
    higher2 = (h_range2[1], s_range[1], v_range[1])
    red_purple = cv.inRange(hsv, lower2, higher2)

    singleColor = cv.bitwise_or(red_orange, red_purple)

    

    n = 0
    

    # Утолщаем тонкие элементы границы
    filtered = cv.morphologyEx(
        singleColor,
        cv.MORPH_DILATE,
        cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    )

    # Выделяем контур
    #edge = cv.Canny(filtered, 50, 150)

    # Выбираем достаточно крупные контуры
    contours, hierarchy = cv.findContours(filtered, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    good_contours = []
    for i in range(len(contours)):
        bounds = cv.boundingRect(contours[i])
        max_bound = max(bounds[2], bounds[3])
        if (max_bound > 10):
            good_contours.append(contours[i])

    img_outline = img.copy()
    cv.drawContours(img_outline, good_contours, -1, (0, 0, 255), 2, 0, None, 1)
    #cv.imshow("Frame", img_outline)
  #  cv.waitKey(1)

    boxed = []
    for contour in good_contours:
        peri = cv.arcLength(contour, True)
        # Ищем углы
        approx = cv.approxPolyDP(contour, 0.07 * peri, True)
     #   print(approx)

        #------ Сортировка по углу
        bounds = cv.boundingRect(contour)
        center = (bounds[0] + 0.5 * bounds[2], bounds[1] + 0.5 * bounds[3])
        def angleToCenter(point):
            point = point[0]
            return math.atan2(point[1] - center[1], point[0] - center[0])
        approx = sorted(approx, key = angleToCenter)
        #------

        if len(approx) == 4:

            approx = np.array([item for sublist in approx for item in sublist], np.float32)
            #box = np.array([[399, 0], [399, 399], [0, 399], [0, 0]], np.float32)
            box = np.array([[0, 0], [399, 0], [399, 399], [0, 399]], np.float32)

            # Преобразование картинки (перевод в квадрат по 4 точкам)
            transform = cv.getPerspectiveTransform(approx, box)
            boxed_i = cv.warpPerspective(img, transform, (400, 400))

            # Добавляем в массив найденных знаков
            boxed.append(boxed_i)
  #  cv.waitKey(1)
    i = 0
  

    for sign in boxed:
        # Здесь распознавание по шаблону
        #cv.imshow(str(i), sign)
        i = i + 1

      #  status = GPIO.input(22)
      #  print(status)

        result = cv.matchTemplate(sign, templ, cv.TM_SQDIFF_NORMED)

        cv.imshow(result_window, result)
   #     print(result[0][0])
        if (result < 0.2):
            n = 1



        
    #GPIO.cleanup()

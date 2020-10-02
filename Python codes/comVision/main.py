# -*- coding: utf-8 -*-

#

# @TODO: compute stereo depth
# @TODO: a function. Input an image, return its depth info
# @TODO: collect samples,and train SVM model
# @TODO: figure out a tracking solution
# @TODO: figure out a hardware control solution

# %% import modules
import time
import port_carema
import threading
import binascii
import pickle as pkl
import random
import datetime
import numpy as np
import os.path
import sys
import keyboard
import logging
import cv2
from util import *
from serial import SerialException, portNotOpenError
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEDisconnectError, BTLEInternalError, BTLEException
from serial.tools import list_ports
import matplotlib.pyplot as plt


# %% define functions


def check_connection():
    """Check whether connection with BLE is normal. Reconnect it if disconnected"""
    global connection
    connected = False
    changed = False
    count = 0
    while not connected:
        log_info = time.ctime()[4:-5] + ":  disconnected,reconnecting."
        logger.info(log_info)
        if count > 10:
            log_info = time.ctime()[4:-5] + ":  connect JDY_16 fail."
            logger.info(log_info)
            break
        connected = True
        try:
            connection.writeCharacteristic(handle, bytes.fromhex('aa0001'))
        except BTLEDisconnectError or BTLEException:
            changed = True
            log_info = time.ctime()[4:-5] + ":  BLE not connected."
            logger.info(log_info)
            connected = False
            connection = Peripheral(JDY_MAC)
            time.sleep(1)
            count += 1
        except BTLEInternalError:
            changed = True
            log_info = time.ctime()[4:-5] + ":  BLE not connected."
            logger.info(log_info)
            connected = False
            connection.connect(JDY_MAC)
            time.sleep(1)
            count += 1
    time.sleep(0.05)
    if changed:
        log_info = time.ctime()[4:-5] + ":  connected. Reset car."
        logger.info(log_info)
        reset_car()


def call_back_func(event):
    """
    Call back function of key board events
    It captures user's action and send corresponding commands to JDY-16
    :param event: a key event
    """
    if event.event_type is 'down':
        if event.name is 'a':
            change_direction('left')
        elif event.name is 'd':
            change_direction('right')
        elif event.name is 'w':
            head_forwards()
        elif event.name is 's':
            head_backwards()
        elif event.name is 'f':
            stop()
        elif event.name is 'e':
            change_speed(states['speed'] + 1)
        elif event.name is 'q':
            change_speed(states['speed'] - 1)
        elif event.name is 'u':
            change_servos_pos(direction_vertical=1)
        elif event.name is 'i':
            change_servos_pos(direction_vertical=-1)
        elif event.name is 'j':
            change_servos_pos(direction_horizontal=1)
        elif event.name is 'k':
            change_servos_pos(direction_horizontal=-1)
        elif event.name is 'esc':
            keyboard.unhook_all()
        elif event.name is 'n':
            ports_list[0].take_a_photo()
            cv2.imshow('a', ports_list[0].image_np)
            cv2.waitKey(5000)
            cv2.destroyWindow('a')
        elif event.name is 'm':
            ports_list[1].take_a_photo()
            log_info = time.ctime() + ". Left camera took a photo. File's name: " + ports_list[1].file_name
            logger.info()
            cv2.imshow('b', ports_list[0].image_np)
            cv2.waitKey(5000)
            cv2.destroyWindow('b')


def head_forwards():
    """head forwards"""
    check_connection()
    time.sleep(0.1)
    log_info = time.ctime()[4:-5] + ":  head forwards."
    logger.info(log_info)
    connection.writeCharacteristic(handle, codebook['forwards'])
    last_states['direction'] = states['direction']
    states['direction'] = 'forwards'


def head_backwards():
    """head backwards"""
    check_connection()
    time.sleep(0.1)
    log_info = time.ctime()[4:-5] + ":  head backwards."
    logger.info(log_info)
    connection.writeCharacteristic(handle, codebook['backwards'])
    last_states['direction'] = states['direction']
    states['direction'] = 'backwards'


def change_direction(direction, degree=0):
    """
    Change current moving direction
    :param direction: 'left' or 'right'.
    :param degree: how many degrees the direction should be changed
    :return: return -1 if no change was made, otherwise return 1
    """
    check_connection()
    time.sleep(0.1)
    if direction is 'left':
        direction_str = '05'
    elif direction is 'right':
        direction_str = '06'
    else:
        return -1
    degree = int(degree)
    if degree is 0:
        if direction is 'left':
            time.sleep(0.02)
            connection.writeCharacteristic(handle, codebook['left'])
            last_states['direction'] = states['direction']
            log_info = time.ctime()[4:-5] + ":  turn left."
            logger.info(log_info)
            states['direction'] = 'left'
        elif direction is 'right':
            time.sleep(0.02)
            connection.writeCharacteristic(handle, codebook['right'])
            last_states['direction'] = states['direction']
            log_info = time.ctime()[4:-5] + ":  turn right."
            logger.info(log_info)
            states['direction'] = 'right'
    # no need to change direction when degree is greater than 0
    elif 0 < degree <= 15:
        degree = hex(degree).replace('0x', '')
        command = bytes.fromhex(direction_str + '0' + str(degree) + '05')
        connection.writeCharacteristic(handle, command)
        log_info = time.ctime()[4:-5] + ":  turn " + direction_str + ", degree: " + str(degree)
        logger.info(log_info)
    elif 15 <= degree <= 180:
        degree = hex(degree).replace('0x', '')
        command = bytes.fromhex(direction_str + str(degree) + '05')
        connection.writeCharacteristic(handle, command)
        log_info = time.ctime()[4:-5] + ":  turn " + direction_str + ", degree: " + str(degree)
        logger.info(log_info)


def change_servos_pos(degree_up=0, degree_down=0, direction_vertical=None, direction_horizontal=None):
    """
    Change the positions of two servos
    :param degree_up: how many degrees of the servo above are expected move
    :param degree_down: how many degrees of the servo below are expected move
    :param direction_vertical:
    only makes sense when degree_up is zero.
    move one step upwards when it's 'up',
    move one step downwards when it's 'down'
    :param direction_horizontal:
    only makes sense when degree_down is zero.
    move one step left when it's 'left',
    move one step right when it's 'right'
    """
    check_connection()
    time.sleep(0.1)
    if degree_down is 0 and degree_up is 0:
        if direction_horizontal == -1:
            if states['servos_down'] == 1:
                command_horizontal = '99'
            else:
                command_horizontal = states['servos_down'] - 1
                command_horizontal = '0' + str(command_horizontal)
                last_states['servos_down'] = states['servos_down']
                states['servos_down'] -= 1
        elif direction_horizontal == 1:
            if states['servos_down'] == 9:
                command_horizontal = '99'
            else:
                command_horizontal = states['servos_down'] + 1
                command_horizontal = '0' + str(command_horizontal)
                last_states['servos_down'] = states['servos_down']
                states['servos_down'] += 1
        else:
            command_horizontal = '99'
        if direction_vertical == -1:
            if states['servos_up'] == 1:
                command_vertical = '99'
            else:
                command_vertical = states['servos_up'] - 1
                command_vertical = '0' + str(command_vertical)
                last_states['servos_up'] = states['servos_up']
                states['servos_up'] -= 1
        elif direction_vertical == 1:
            if states['servos_up'] == 5:
                command_vertical = '99'
            else:
                command_vertical = states['servos_up'] + 1
                command_vertical = '0' + str(command_vertical)
                last_states['servos_up'] = states['servos_up']
                states['servos_up'] += 1
        else:
            command_vertical = '99'
    else:
        if 0 < degree_up <= 5:
            command_vertical = '0' + str(degree_up)
            last_states['servos_up'] = states['servos_up']
            states['servos_up'] = degree_up
        else:
            command_vertical = '99'
        if 0 < degree_down <= 9:
            command_horizontal = '0' + str(degree_down)
            last_states['servos_down'] = states['servos_down']
            states['servos_down'] = degree_down
        else:
            command_horizontal = '99'
    command = '09' + command_vertical + command_horizontal
    # print(command)
    command = bytes.fromhex(command)
    time.sleep(0.05)  # necessary delay
    connection.writeCharacteristic(handle, command)
    log_info = time.ctime()[4:-5] + ": change servos's postures. " \
               + "servo above: " + str(states['servos_up']) + ", servo below: " \
               + str(states['servos_down'])
    logger.info(log_info)


def change_speed(speed=9):
    """
    Change the speed of model car. Full speed by default.
    If launch the function when model car stops, it'll travel for 0.5 sec.
    :param speed: ranged from 3 to 9. The bigger the speed is ,the faster model car travels
    :return: Return -1 if fails to change the speed, return 1 otherwise.
    """
    check_connection()
    time.sleep(0.05)
    if speed < 2 or speed > 10:
        print("speed not in range. Fail")
        return -1
    else:
        command = '070' + str(speed) + '07'
        command = bytes.fromhex(command)
        connection.writeCharacteristic(handle, command)
        last_states['speed'] = states['speed']
        states['speed'] = speed
        if states['direction'] is 'stop':
            time.sleep(0.25)
            stop()
        log_info = time.ctime()[4:-5] + " change speed to: " + states['speed']
        logger.info(log_info)
        return 1


def stop():
    """stop model car"""
    time.sleep(0.05)
    # check_connection()
    states['direction'] = 'stop'
    time.sleep(0.05)
    connection.writeCharacteristic(handle, codebook['stop'])
    log_info = time.ctime()[4:-5] + " stop."
    logger.info(log_info)


def reset_car():
    time.sleep(0.05)
    connection.writeCharacteristic(handle, codebook['stop'])
    states['direction'] = 'stop'
    time.sleep(0.05)
    connection.writeCharacteristic(handle, bytes.fromhex('070555'))
    states['speed'] = 5
    time.sleep(0.05)
    connection.writeCharacteristic(handle, bytes.fromhex('090206'))
    states['servos_up'] = 2
    states['servos_down'] = 6
    log_info = time.ctime()[4:-5] + " reset model car."
    logger.info(log_info)


# %% define a logger
"""more info, see https://www.cnblogs.com/CJOKER/p/8295272.html"""
global logger
logger = logging.getLogger()
logger.setLevel(logging.INFO)
rq = time.strftime('%Y%m%d%H%M', time.localtime(time.time()))
log_path = os.path.dirname(os.getcwd()) + '/Logs/'
if not os.path.exists(log_path):
    os.mkdir(log_path)
log_name = log_path + rq + '.log'
logfile = log_name
f = open(logfile, 'w')
f.close()
fh = logging.FileHandler(logfile, mode='w')
fh.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
fh.setFormatter(formatter)
logger.addHandler(fh)
log_info = time.ctime()[4:-5] + " logger registered."
logger.info(log_info)

# %% define constants and variables
global codebook
global JDY_MAC
global connection
global handles
global handle
global states
global last_states
global ports_list
global threads
JDY_MAC = "3C:A5:19:7A:DF:31"
codebook = {'forwards': '010101', 'backwards': '020202', 'left': '030303',
            'right': '040404', 'stop': '877777'}
for item in codebook:
    codebook[item] = bytes.fromhex(codebook[item])
init_servos_msg = bytes.fromhex('09 02 06')
states = {'direction': 'forwards', 'speed': 5, 'servos_up': 2, 'servos_down': 6}
last_states = states
log_info = time.ctime()[4:-5] + "constants and variables defined."
logger.info(log_info)

# %% create bluetooth connection, get services and characteristics
try:
    connection = Peripheral(deviceAddr=JDY_MAC)
    log_info = time.ctime()[4:-5] + " BLE connected."
    logger.info(log_info)
except BTLEDisconnectError:
    log_info = time.ctime()[4:-5] + " create BLE connection fail. exit"
    logger.info(log_info)
    sys.exit()
services = connection.getServices()
services = list(services)
characteristics = connection.getCharacteristics()
characteristics = list(characteristics)
descriptors = connection.getDescriptors()
handles = []
for characteristic in characteristics:
    # characteristic.handle not equal to characteristic.getHandle()
    handles.append(characteristic.getHandle())
handle = handles[-1]

# %% create serial connections
port_names = get_port_devices()
for i, item in enumerate(port_names):
    if 'USB' not in item:
        del port_names[i]
        break
ports_list = open_ports(port_names[0:len(port_names)])
log_info = time.ctime()[4:-5] + " open two ports success."
logger.info(log_info)

# %% camera calibration     unit:millimeters
intrinstic_matrix = [[558.9213, 0, 122.5113], [0, 556.5961, 143.6292], [0, 0, 1]]
intrinstic_matrix = np.float64(intrinstic_matrix)
focal_length = [558.9213, 556.5961]
principle_point = [1.225113372015190e+02, 1.436291548334767e+02]

# %% train svm model
"""
Reference: https://blog.csdn.net/qq_33662995/article/details/79356939
"""
# 1. load samples
root_directory = '/home/pi/Desktop/comVision'
os.chdir(root_directory)
os.chdir('./positive_samples')
positive_samples_names = os.listdir()
positive_img_size = (96, 160)
positive_samples = prepare_imgs(file_name_list=positive_samples_names, img_size=positive_img_size)
os.chdir(root_directory)
os.chdir("./negative_samples")
negative_samples_names = os.listdir()
negative_img_size = (64, 128)
negative_samples = prepare_imgs(file_name_list=negative_samples_names, img_size=negative_img_size)
negative_img_size_ = (128, 64)
positive_img_size_ = (160, 96)
# 2. train the model
labels = []
gradients_list = []
compute_hogs(positive_samples, gradients_list)
[labels.append(+1) for _ in range(len(positive_samples))]
compute_hogs(negative_samples, gradients_list)
[labels.append(-1) for _ in range(len(negative_samples))]
svm = svm_config()
svm.train(np.array(gradients_list), cv2.ml.ROW_SAMPLE, np.array(labels))
# 3. second run train. hard negative samples
os.chdir(root_directory)
os.chdir("hard_negative_samples")
full_negative_samples_names = os.listdir()
full_negative_samples = list(map(lambda x: cv2.imread(x), full_negative_samples_names))
hog = cv2.HOGDescriptor()
hog.setSVMDetector(get_svm_detector(svm))
hard_negative_samples = []
for i in range(len(full_negative_samples)):
    try:
        rects, wei = hog.detectMultiScale(full_negative_samples[i], winStride=(4, 4), padding=(8, 8), scale=1.05)
        for (x, y, w, h) in rects:
            hardExample = full_negative_samples[i][y:y + h, x:x + w]
            hard_negative_samples.append(cv2.resize(hardExample, (64, 128)))
    except:
        continue
compute_hogs(hard_negative_samples, gradients_list)
[labels.append(-1) for _ in range(len(hard_negative_samples))]
svm.train(np.array(gradients_list), cv2.ml.ROW_SAMPLE, np.array(labels))
# 4. save model
os.chdir(root_directory)
file_name = "hog_detector.bin"
if file_name not in os.listdir():
    hog.save(file_name)
log_info = time.ctime()[4:-5] + " train svm model success."

# %% main loop
stereo = cv2.StereoBM_create(numDisparities=96, blockSize=9)
while 1:
    threads = create_threads(ports_list)
    threads[0].start()
    threads[1].start()
    preprocess_image(port=ports_list[0], Logger=logger)
    positions = [None, None]

    for i, port in enumerate(ports_list):
        img = port.image_np
        rects, wei = hog.detectMultiScale(img, winStride=(4, 4), padding=(8, 8), scale=1.05)
        # choose the most likely rectangle
        max_wei = np.max(wei)
        index = None
        for j, w in enumerate(wei):
            if w == max_wei:
                index = j
                break
        (x, y, w, h) = rects[index]
        if max_wei > 0.5:
            positions.append([np.mean([x, x + w]), np.mean([y, y + h])])
    if len(positions) == 0:
        log_info = time.ctime()[4:-5] + " detect pedestrian fail, exit!"
        logger.info(log_info)
        sys.exit()
    elif len(positions) == 2:
        # get the average detected position
        positions = np.mean(positions, axis=0)
    disparsity = stereo.compute(ports_list[0].image_np, ports_list[1].image_np)

    # TODO: develop a solution to switch the car's driving state based on the detected position
# %% test

# move forward for 2 seconds, then stop for 1 second, then move backwards for 2 seconds
head_forwards()
time.sleep(2)
stop()
time.sleep(1)
head_backwards()
time.sleep(2)
stop()

# turn left for 90 degrees and then turn right for 90 degrees
change_direction('left', 90)
time.sleep(2)
change_direction('right', 90)

# change postures of the two servos

for i in range(9):
    for j in range(5):
        change_servos_pos(degree_up=j,degree_down=i)
        time.sleep(0.3)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 14 19:24:52 2020

@author: Rancho
"""
import datetime
import os
import sys
import time

import cv2 as cv
import serial
import serial.tools.list_ports
from serial import SerialException, portNotOpenError


class Port:
    port = None
    image_np = None
    raw_img_data = None
    temp = None
    file_name = None
    time_spent = 'aaa'
    photos_directory_name = 'photos/'

    def __init__(self):
        self.temp = ''
        self.port = 'None'

    # close the port
    def close(self):
        self.port.close()
        if not self.port.isOpen():
            print("close port success")
        else:
            print("close port fail")

    # open a port
    def set_port(self, port_object):
        try:
            self.port = serial.Serial(port_object)
        except SerialException:
            print("Set port fail,exit")
            sys.exit()
        except portNotOpenError:
            print("Port not open.exit")
            sys.exit()
        else:
            print("open the port success")

    # initialize the port, including baudrate, bytesize, etc.
    def initialize_port(self):
        self.port.baudrate = 115200
        self.port.bytesize = 8
        self.port.stopbits = 1
        self.port.parity = 'N'
        self.change_serial_baudrate_to_115200()
        self.change_compress_rate_to_36()
        self.reset_camera()
        print('port initialization success,port infos:')
        print('baudrate: ' + str(self.port.baudrate) + ' bytesize: ' + str(self.port.bytesize) + ' stopbits:' + str(
            self.port.stopbits) + ' parity: ' + str(self.port.parity) + '\n')

    # set camera's baudrate to 115200
    def change_serial_baudrate_to_115200(self):
        self.port.write(bytes.fromhex('56 00 31 06 04 02 00 08 0D A6'))
        time.sleep(0.05)
        temp = self.port.read_all()
        if len(temp) != 0:
            print('change serial baudrate to 115200 success, received info: ' + temp.hex() + '\n')
        else:
            print('change serial baudrate to 115200 fail' + '\n')

    # set camera's compression rate to 36
    def change_compress_rate_to_36(self):
        # self.port.write(bytes.fromhex('56 00 31 05 01 01 12 04 03'))4
        self.port.write(bytes.fromhex('56 00 31 05 01 01 12 04 FF'))
        time.sleep(0.05)
        temp = self.port.read_all()
        if len(temp) != 0:
            print("change compress rate to 36 success, received info: " + temp.hex() + '\n')
        else:
            print('change compress rate to four fail' + '\n')

    # initialize camera( including works above)
    def reset_camera(self):
        self.port.write(bytes.fromhex('56 00 26 00'))
        time.sleep(0.05)
        temp = self.port.read_all()
        while b'Init end\r\n' not in temp:
            temp += self.port.read_all()
            time.sleep(0.01)
        self.flush()
        print("Reset camera success")
        # 收到的信息为：
        # vVC0703 1.00
        # Ctrl infr exist
        # User-defined sensor
        # 525
        # Init end

    # clear the flush
    def flush(self):
        while self.port.in_waiting:
            temp = self.port.read_all()
            time.sleep(0.01)

    # print information of the opened port
    def print_port_info(self):
        print('baudrate: ' + str(self.port.baudrate))
        print('bytesize:' + str(self.port.bytesize))
        print('stopbits:' + str(self.port.stopbits))
        print('parity:' + str(self.port.parity) + '\n')

    # clear camera's cache(not necessary, may cause corrupted JPEG data sometimes)
    def clear_cache(self):
        self.port.write(bytes.fromhex('56 00 36 01 03'))
        time.sleep(0.05)
        temp = self.port.read_all()
        if len(temp) != 0:
            print('clear cache success, received info: ' + temp.hex() + '\n')
        else:
            print('clear cache fail' + '\n')

    # take a photo, restore image's information in self.image_np
    def take_a_photo(self):
        self.time_spent = time.time()
        self.raw_img_data = ''  # clear the former images
        self.clear_cache()  # it's a must
        self.port.write(bytes.fromhex('56 00 36 01 00'))
        # returns 76 00 36 00 00
        time.sleep(0.01)
        temp = self.port.read_all()
        while len(temp) != 5:
            temp = temp + self.port.read_all()
            time.sleep(0.01)
        print("Send the command to take a photo: success")

        self.port.write(bytes.fromhex('56 00 34 01 00'))
        # returns 76 00 34 00 04 00 00 33 40(可能地址位不同，但是一定为9位)
        time.sleep(0.01)
        raw_addr_and_len = self.port.read_all()
        while len(raw_addr_and_len) != 9:
            raw_addr_and_len += self.port.read_all()
            time.sleep(0.01)
        raw_addr_and_len = raw_addr_and_len.hex()
        addr = raw_addr_and_len[-4:-2].upper() + ' ' + raw_addr_and_len[-2:].upper()
        get_img_command = bytes.fromhex('56 00 32 0C 00 0A 00 00 00 00 00 00 ' + addr + ' 00 FF')
        self.flush()
        self.port.write(get_img_command)
        time.sleep(0.05)
        self.raw_img_data = self.port.read_all()
        while self.raw_img_data[-5:] != bytes.fromhex("76 00 32 00 00") or self.port.in_waiting:
            self.raw_img_data += self.port.read_all()
            time.sleep(0.1)
        self.decode_from_stream()
        self.time_spent = time.time() - self.time_spent
        print("time spent: %f" % self.time_spent)
        print(time.ctime())

    # decode an image from buffer, restore image info in self.image_np
    def decode_from_stream(self):
        self.raw_img_data = self.raw_img_data.decode("ISO8859-1")
        replaced_str = b'v\x002\x00\x00'.decode("ISO8859-1")  # ANSI编码
        self.raw_img_data = self.raw_img_data.replace(replaced_str, '').encode("ISO8859-1")
        time_now = str(datetime.datetime.now())
        time_now = time_now.replace(':', '_').replace(' ', '_')
        time_now = time_now.replace('.', '_').replace("2020-", '')
        time_now = time_now.replace(time_now[-7:], '')
        # self.file_name = self.photos_directory_name+time_now + ".jpg"
        self.file_name = time_now + ".jpg"
        with open(self.file_name, 'wb') as f:
            f.write(self.raw_img_data)
        self.image_np = cv.imread(self.file_name,0)
        cv.flip(self.image_np, 0, self.image_np)
        # self.image_np = cv.flip(self.image_np, 0)
        # try:
        #     cv.imshow('a', self.image_np)
        #     cv.waitKey(6000)
        #     cv.destroyWindow('a')
        # except:
        #     pass

    # display the image
    def show_image(self, window_name='window_name_not_set'):
        try:

            cv.imshow(window_name, self.image_np)
            cv.waitKey(6000)
            cv.destroyWindow(window_name)
        except cv.error:
            print("no photo taken")

    # delete the saved image from current working derectory
    def delete_image(self):
        os.remove(self.file_name)
        self.file_name = None

    # demo
    def demo(self):
        self.initialize_port()
        time.sleep(0.05)  # dalay is vital
        self.print_port_info()
        time.sleep(0.05)
        self.reset_camera()
        time.sleep(0.05)
        time.sleep(0.05)
        self.change_compress_rate_to_36()
        time.sleep(0.05)
        self.clear_cache()
        time.sleep(0.05)
        self.take_a_photo()
        time.sleep(0.05)
        self.show_image()

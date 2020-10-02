import port_carema
import time
import threading
import cv2
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEDisconnectError, BTLEInternalError, BTLEException
from serial.tools import list_ports
import numpy as np
import os

from main import hog


class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)


def get_port_devices():
    """
    get names of available ports
    :return: a list of the names of available ports
    """
    port_names_ = []
    serial_list = list_ports.comports()
    for serial_name in serial_list:
        port_names_.append(serial_name.device)
    return port_names_


def open_ports(port_names_):
    """
    Open the ports.
    :param port_names_: a list of the names of ports
    :return: a list of Port objects
    """
    ports_list_ = list()
    for i in range(len(port_names_)):
        port = port_carema.Port()
        port.set_port(port_names_[i])
        port.initialize_port()
        ports_list_.append(port)
    return ports_list_


def preprocess_image(port=None, img=None, alpha=0.2, Logger=None):
    """
    The camera on the left side has a larger view. Resize it so that two pictures' perspectives are
    almost the same (for nextstep 3-d reconstruction and stereo depth)
    :param img: image to be processed
    :param port: an instance of Port
    :param alpha: (to how much degree the image should be resized
    :param Logger: a Logger(if available)
    """
    if port is None and img is None:
        if Logger is not None:
            info_msg = 'No image captured, return -1'
            Logger.info(info_msg)
        return -1
    else:
        height = 240
        width = 320
        delta_height = int(alpha * height)
        delta_width = int(alpha * width)
        if port is not None:
            port.image_np = port.image_np[delta_height:(height - delta_height), delta_width:(width - delta_width)]
            port.image_np = cv2.resize(port.image_np, (320, 240))
        else:
            img = img[delta_height:(height - delta_height), delta_width:(width - delta_width)]
            img = cv2.resize(img, (320, 240))
            return img


def load_image(file_name):
    """
    flip an image horizontally
    :param file_name: the file name of image
    :return: an image
    """
    img = cv2.imread(file_name, 0)
    if img is None:
        print_info = "get image fail"
        print(print_info)
        return -1
    cv2.flip(img, 1, img)
    return img


def create_threads(port_lists_):
    """
    Create a thread for two port
    Notice that a thread can only be started for once
    Check whether a thread is alive by thread.is_alive()
    :param port_lists_:
    :return: two threads
    """
    port_1 = port_lists_[0]
    port_2 = port_lists_[1]
    threads_ = []
    thread_1 = threading.Thread(target=port_1.take_a_photo)
    thread_2 = threading.Thread(target=port_2.take_a_photo)
    threads_.append(thread_1)
    threads_.append(thread_2)
    return threads_


def prepare_imgs(file_name_list, img_size=None):
    """
    Prepare imgs for svm model
    :param file_name_list: list of imgs' file names
    :param img_size: targeted img size,width * height. a tuple
    :return: list of imgs(np.ndarrays)
    """
    imgs_list = []
    for file_name in file_name_list:
        imgs_list.append(cv2.imread(file_name))
    # print(imgs_list[0] is None)
    if img_size is not None:
        for i in range(len(imgs_list)):
            imgs_list[i] = cv2.resize(imgs_list[i], img_size)
    return imgs_list


def compute_hogs(imgs_list, gradients_list, w_size=(128, 64)):
    """
    compute hogs or list of images
    :param w_size: size of RIO
    :param imgs_list: the list of images
    :param gradients_list: the list of gradients of images
    """
    hog = cv2.HOGDescriptor()
    for img in imgs_list:
        # resize RIO
        if img.shape[1] >= w_size[1] and img.shape[0] >= w_size[0]:
            rio = img[(img.shape[0] - w_size[0]) // 2:(img.shape[0] - w_size[0]) // 2 + w_size[0],
                  (img.shape[1] - w_size[1]) // 2:(img.shape[1] - w_size[1]) // 2 + w_size[1]]
        gray_img = cv2.cvtColor(rio, cv2.COLOR_BGR2GRAY)
        gradients_list.append(hog.compute(gray_img))


def svm_config():
    """
    Create a svm classfier
    :return: a svm classifier
    """
    svm = cv2.ml.SVM_create()
    svm.setCoef0(0)
    svm.setCoef0(0.0)
    svm.setDegree(3)
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 1000, 1e-3)
    svm.setTermCriteria(criteria)
    svm.setGamma(0)
    svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setNu(0.5)
    svm.setP(0.1)  # for EPSILON_SVR, epsilon in loss function?
    svm.setC(0.01)  # From paper, soft classifier
    svm.setType(cv2.ml.SVM_EPS_SVR)  # C_SVC # EPSILON_SVR # may be also NU_SVR # do regression task
    svm.setC(0.01)
    return svm


def get_svm_detector(svm):
    """
    get a svm detector
    :param svm: a svm classifier
    :return: a svm detector
    """
    support_vectors = svm.getSupportVectors()
    rho, _, _ = svm.getDecisionFunction(0)
    support_vectors = np.transpose(support_vectors)
    return np.append(support_vectors, [[-rho]], 0)


def test_svm(root_directory):
    """
    test the svm model
    :param root_directory: root directory of project
    """
    os.chdir(root_directory)
    os.chdir("./imgs")
    img_names = os.listdir()
    imgs = list(map(lambda x: cv2.imread(x), img_names))
    imgs = imgs[35:200]
    for i, img in enumerate(imgs):
        rects, wei = hog.detectMultiScale(img, winStride=(4, 4), padding=(8, 8), scale=1.05)
        max_wei = np.max(wei)
        index = None
        for j, w in enumerate(wei):
            if w == max_wei:
                index = j
                break
        (x, y, w, h) = rects[index]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255))
        cv2.imwrite("../svm_test_" + str(i) + ".png", img)
        # cv2.imshow('a', img)
        # cv2.waitKey(200)


def calculate_depth(img1,img2,stereo):


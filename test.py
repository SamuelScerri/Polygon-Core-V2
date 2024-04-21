from engine import ghetty
import numpy
import base64
import cv2
from multiprocessing import Process

def process():
    print('Started Process')

    sr = cv2.dnn_superres.DnnSuperResImpl_create()
    sr.readModel('models/FSRCNN_x2.pb')
    sr.setModel('fsrcnn', 2)

    sr.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
    sr.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)

    img = cv2.imread('images/buffer.png')

    while True:
        sr.upsample(img)

def on_render():
    image = numpy.frombuffer(base64.b64decode(ghetty.EncodedImage()), numpy.uint8).reshape((90, 160, 4))
    rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

if __name__ == '__main__':
    upscaler = Process(target=process)
    upscaler.start()

    ghetty.Launch(renderCallback=on_render)
    upscaler.join()
from engine import ghetty
import numpy
import base64
import cv2
from multiprocessing import Process, Pipe
import asyncio

def process(connection):
    print('Started Process')

    sr = cv2.dnn_superres.DnnSuperResImpl_create()
    sr.readModel('models/FSRCNN_x4.pb')
    sr.setModel('fsrcnn', 4)

    sr.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    sr.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)

    while True:
        if connection.poll():
            data = connection.recv()

            image = numpy.frombuffer(base64.b64decode(data), numpy.uint8).reshape((90, 160, 4))
            rgb_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            sr.upsample(rgb_img)
            connection.send(data)

if __name__ == '__main__':
    parent_connection, child_connection = Pipe()

    upscaler = Process(target=process, args=(child_connection,))
    upscaler.start()

    def on_render():
        parent_connection.send(ghetty.EncodedImage())
        parent_connection.recv()

    ghetty.Launch(renderCallback=on_render)
    upscaler.join()
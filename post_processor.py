from engine import ghetty
import numpy
import base64
import cv2

#Load Model
sr = cv2.dnn_superres.DnnSuperResImpl_create()
ghetty_initialized = 0

def slice_image(img, rows, cols):
    height, width = img.shape[:2]

    slice_height = int(height / rows)
    slice_width = int(width / cols)

    slices = []

    for y in range(0, height, slice_height):
        for x in range(0, width, slice_width):
            slice = img[y:y+slice_height, x:x+slice_width]
            slices.append(slice)

    return slices

def initialize():
    print('Initializing')

    model_name = ''

    if ghetty.Upscaler() == 'fsrcnn':
        print('Detected FSRCNN With Scale:', ghetty.Scale())
        model_name += 'FSRCNN_x'
    elif ghetty.Upscaler() == 'lapsrn':
        print('Detected LapSRN With Scale:', ghetty.Scale())
        model_name += 'LapSRN_x'

    model_name += str(ghetty.Scale()) + '.pb'

    sr.readModel('models/cnn_models/' + model_name)
    sr.setModel(ghetty.Upscaler(), ghetty.Scale())

    #Set Backend To OpenCL (Enables Hardware Acceleration, Note To Self: Should Try Vulkan Backend But Requires Re-Compilation Of OpenCV)
    sr.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    sr.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)

#Callback To When The Engine Renders A Scene, This Returns An Encoded Image
def on_render():
    global ghetty_initialized

    if ghetty_initialized == 0:
        initialize()
        ghetty_initialized = 1

    #Convert To Numpy Array As OpenCV Requires That Format
    image = numpy.frombuffer(base64.b64decode(ghetty.EncodedImage()), dtype=numpy.uint8).reshape((ghetty.Height, ghetty.Width, 4))

    #Use Loaded Model To Upscale
    result = sr.upsample(cv2.cvtColor(image, cv2.COLOR_BGRA2BGR))

    #Flatten And Re-Encode The Image
    converted_image = cv2.cvtColor(result, cv2.COLOR_BGR2BGRA).flatten()
    return base64.b64encode(converted_image)

ghetty.Launch(renderCallback=on_render)

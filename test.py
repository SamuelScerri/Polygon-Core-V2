from engine import ghetty
import numpy
import base64
import cv2

if __name__ == '__main__':
    sr = cv2.dnn_superres.DnnSuperResImpl_create()
    sr.readModel('models/ESPCN_x4.pb')
    sr.setModel('espcn', 4)

    sr.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    sr.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)

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

    def on_render():
        image = numpy.frombuffer(base64.b64decode(ghetty.EncodedImage()), numpy.uint8).reshape((90, 160, 4))
        rgb_img = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        
        result = sr.upsample(rgb_img)

        cv2.imwrite('images/buffer.png', cv2.cvtColor(result, cv2.COLOR_BGR2RGB))

    ghetty.Launch(renderCallback=on_render)
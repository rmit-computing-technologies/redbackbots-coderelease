# import numpy as np
import cv2
import os
import time
import images_pred as ai


def estimate_ref_pose(blackboard):
    #time.sleep(1)
    # Read buffer from blackboard
    buf = blackboard.vision.image_buffer
    # Read buffer into numpy array
    arr = np.frombuffer(buf, dtype=np.uint8)
    # Reshape array into required shape
    im = arr.reshape(960, 1280, 2)
    
    # Covert into useful colours
    image = cv2.cvtColor(im, cv2.COLOR_YUV2BGR_YUYV)   # From YUV TO BGR

    # Call preprocessing and predicting module
    prediction = ai.image_angles(image)
    #gd.detect_gesture(image)


    #i = 0
    #while os.path.exists("images/imagefaces%s.jpg" % i):
    #    i += 1
    
    # cv2.imwrite("images/imagefaces%s.jpg" % i, image)
    return prediction
    # time.sleep(0.5)


import numpy as np
import cv2
import mediapipe as mp # mediapipe 0.10.*
import math
import onnxruntime
import time

"""
Todo: add compatability with whistle
"""
required_keypoints = [15, 13, 11, 16, 14, 12, 23]

sequence_length = 20

# Add +2 to size if using 30 LSTM angled model
size = len(required_keypoints)*2 # + 2

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(model_complexity=0)

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

keypoints_sequence = np.zeros((1,sequence_length,size))
num_frame = 0
num_collected_frames = 0

lower_red = np.array([160,80,75])
upper_red = np.array([180,255,255])

while True:
    ret, frame = video.read()
    if not ret:
        break
    if num_collected_frames >= sequence_length:
        break
    if num_frame >= 200:
        break
    
    if num_frame % 6 == 0 and num_frame > 0:
        frame.flags.writeable = True

        # OpenCV flips image if size is set to 640x480, so we flip back the other way
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        average = cv2.blur(frame.copy(), (5,5))
        hsv = cv2.cvtColor(average, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        x,y,w,h = cv2.boundingRect(mask)
        
        img_shape = frame.shape

        """
        Mask horizontal coordinates of the image where there is no red color present

        If/else statements ensure that mask size is within image limits
        """
        if x <= 100 and x+w+100 >= img_shape[1]:
            pass
        elif x <= 100:
            masked_image = np.zeros(img_shape,np.uint8)
            masked_image[0:img_shape[0],0:x+w+100] = frame[0:img_shape[0],0:x+w+100]
            frame = masked_image
        elif x+w+100 >= img_shape[1]:
            masked_image = np.zeros(img_shape,np.uint8)
            masked_image[0:img_shape[0],x-100:img_shape[1]] = frame[0:img_shape[0],x-100:img_shape[1]]
            frame = masked_image
        else:
            masked_image = np.zeros(img_shape,np.uint8)
            masked_image[0:img_shape[0],x-100:x+w+100] = frame[0:img_shape[0],x-100:x+w+100]
            frame = masked_image
        
        # Convert image to RGB for BlazePose
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        pose_kp = pose.process(frame)
        if hasattr(pose_kp.pose_landmarks, 'landmark'):
            pose_landmarks = pose_kp.pose_landmarks.landmark
            num_i = 0
            n_zone = (pose_landmarks[24].x, pose_landmarks[24].y)
            for i in required_keypoints:
                keypoints = pose_landmarks[i]
                keypoints_sequence[0][num_collected_frames][num_i] = keypoints.x - n_zone[0]
                num_i += 1
                keypoints_sequence[0][num_collected_frames][num_i] = keypoints.y - n_zone[1]
                num_i += 1
            
            # Comment out angle features if using 128/64 unit SimpleRNN
            """
            angle_red = ((math.degrees(math.atan2(pose_landmarks[11].y-pose_landmarks[13].y,pose_landmarks[11].x-pose_landmarks[13].x) - math.atan2(pose_landmarks[15].y-pose_landmarks[13].y,pose_landmarks[15].x-pose_landmarks[13].x)) + 360) % 360)/360
            angle_blue = ((math.degrees(math.atan2(pose_landmarks[12].y-pose_landmarks[14].y,pose_landmarks[12].x-pose_landmarks[14].x) - math.atan2(pose_landmarks[16].y-pose_landmarks[14].y,pose_landmarks[16].x-pose_landmarks[14].x)) + 360) % 360)/360
                        
            keypoints_sequence[0][num_collected_frames][num_i] = angle_red
            keypoints_sequence[0][num_collected_frames][num_i+1] = angle_blue
            """
            num_collected_frames += 1

    num_frame += 1

video.release()

"""
Two architectures for RNN model. Comment out model you are not using.

SimpleRNN Model:
    - Faster
    - Worse at classifying goal gestures when directly facing referee (midfield)
    - Better at classifying other poses

LSTM Model:
    - Slower
    - Better at classifying goal gestures
    - Slightly worse at other gestures
"""
file_path = './models/onnx/rnn_128x64_20.onnx'
#file_path = './models/onnx/lstm_30x30_20.onnx'

ort = onnxruntime.InferenceSession(file_path, providers=['CPUExecutionProvider'])
prediction = ort.run([ort.get_outputs()[0].name], {ort.get_inputs()[0].name: keypoints_sequence.astype(np.float32)})
predicted_label = np.argmax(prediction)

print("Label: " + str(prediction))
print("Decoded label: " + str(predicted_label))
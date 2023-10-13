from __future__ import division
from __future__ import print_function

from operator import mod
import math
# import numpy as np
import cv2 as cv
import time
# from naoqi import ALProxy

from BehaviourTask import BehaviourTask


class_dict = {'0' : "full-time",
                '1' : "goal-kick-blue",
                '2' : "goal-blue",
                '3' : "goal-kick-red",
                '4' : "goal-red",
                '5' : "corner-kick-blue",
                '6' : "corner-kick-red",
                '7' : "kick-in-blue",
                '8' : "kick-in-red",
                '9' : "pushing-free-kick-blue",
                '10' : "pushing-free-kick-red"}

def sortPixels(l):
    #print(l, l[2], l[3])
    return l[2] * l[3]

def get_coordinates(image_path):
    coordinates = {'red_y': None, 'red_x': None, 'blue_y': None, 'blue_x': None, 'face_x': None, 'face_y': None}
    contour_only_2 = []
    #coordinates['category'] = category
    img = image_path.copy()
    #img = cv.resize(img, (640, 854))
    dimensions = img.shape
    #print('dimensions', dimensions)
    #blank_image = np.zeros((dimensions[0], dimensions[1], 1), np.uint8)
    #blank_image.fill(255)
    face_cascade = cv.CascadeClassifier('/home/nao/data/haarcascade_frontalface_default.xml')
    #print(face_cascade, 'face_cascade')
    
    # Convert into grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    gray_face = gray[:int(dimensions[0]/1.75), :]
    
    # Detect faces
    f = 15
    ind = 0
    f_ind = 0
    face_found = False
    for j in range(130, 155, 3):
        if face_found:
            break
        for i in range(0, f, 1):
            faces = face_cascade.detectMultiScale(gray_face, j/100.0, i)
            if (faces is not None and len(faces) == 1 and faces[0][1] < dimensions[0]/1.75):
                ind = i
                f_ind = j
                face_found = True
                break
    
    kernel_5 = np.ones((5,5),np.uint8)
    
    average = cv.blur(img, (5,5))
    
    # BGR to HSV
    hsv = cv.cvtColor(average, cv.COLOR_BGR2HSV)

    # only red region
    lower_red = np.array([160,80,75])
    upper_red = np.array([180,255,255])
    
    # create a mask using the bounds set
    mask = cv.inRange(hsv, lower_red, upper_red)
    # create an inverse of the mask
    mask = cv.bitwise_not(mask)
    
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, (15,15))
    
    cnts = cv.findContours(cv.bitwise_not(opening), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    
    if(len(faces) == 1):
        x_face, y_face, w_face, h_face = 0, 0, 0, 0
        for (xf, yf, wf, hf) in faces:
            x_face, y_face, w_face, h_face = xf, yf, wf, hf
            #cv.rectangle(blank_image, (xf, yf), (xf+wf, yf+hf), (0, 0, 0), -1)
            #cv.line(blank_image, (xf + int(wf/2), yf), (xf + int(wf/2), 0), (0, 0, 0), thickness=5)
            #cv.line(blank_image, (xf, yf + int(hf/2)), (0, yf + int(hf/2)), (0, 0, 0), thickness=5)

            #cv.line(blank_image, (xf + int(wf/2), yf), (xf + int(wf/2), dimensions[0]), (0, 0, 0), thickness=5)
            #cv.line(blank_image, (xf, yf + int(hf/2)), (dimensions[1], yf + int(hf/2)), (0, 0, 0), thickness=5)
            
        countourList = []
        
        if(len(cnts)>=2):
            coordinates['face_x'] = x_face + int(w_face/2)
            coordinates['face_y'] = y_face + int(h_face/2)
            flat_data_arr = None
            for c in cnts:
                fl = False
                x,y,w,h = cv.boundingRect(c)
                if not (x <= x_face + w_face and x >= x_face - 5 and y <= y_face + h_face  and y >= y_face - 5  and w <= w_face + 5 and h <= h_face + 5):
                    #print(x, y, w, h)
                    if(w >= 5 and h >= 5 and w <= 115 and h <= 115):
                        fl = True

                    if fl:
                        countourList.append([x,y,w,h])

                    countourList.sort(key = sortPixels, reverse = True)

                    if(len(countourList) > 1):
                        for i in range(0,2):
                            #print(len(countourList), 'lllllllllllllllllll')
                            cList = countourList[i]
                            #cv.rectangle(blank_image, (cList[0], cList[1]), (cList[0] + cList[2], cList[1] + cList[3]), (0,0,0), -1)

                        contour_only_2 = []
                        contour_only_2.append(countourList[0])
                        contour_only_2.append(countourList[1])
                        
                        if (len(contour_only_2) == 2):
                            if(contour_only_2[0][0] <= contour_only_2[1][0]):
                                coordinates['blue_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                                coordinates['blue_y'] = contour_only_2[0][1]
                                coordinates['red_x'] = contour_only_2[1][0] + contour_only_2[1][2]/2
                                coordinates['red_y'] = contour_only_2[1][1]

                                coordinates['c_blue_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                                coordinates['c_blue_y'] = contour_only_2[0][1] + contour_only_2[0][3]/2
                                coordinates['c_red_x'] = contour_only_2[1][0] + contour_only_2[1][2]/2
                                coordinates['c_red_y'] = contour_only_2[1][1] + contour_only_2[1][3]/2


                            else:
                                coordinates['red_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                                coordinates['red_y'] = contour_only_2[0][1]
                                coordinates['blue_x'] = contour_only_2[1][0] + contour_only_2[1][2]/2
                                coordinates['blue_y'] = contour_only_2[1][1]

                                coordinates['c_red_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                                coordinates['c_red_y'] = contour_only_2[0][1] + contour_only_2[0][3]/2
                                coordinates['c_blue_x'] = contour_only_2[1][0] + contour_only_2[1][2]/2
                                coordinates['c_blue_y'] = contour_only_2[1][1] + contour_only_2[1][3]/2

        elif(len(cnts) == 1):
            coordinates['face_x'] = x_face + int(w_face/2)
            coordinates['face_y'] = y_face + int(h_face/2)
            for c in cnts:
                x,y,w,h = cv.boundingRect(c)
                fl = False
                if not (x <= x_face + w_face and x >= x_face - 5 and y <= y_face + h_face  and y >= y_face - 5 and w <= w_face + 5 and h <= h_face + 5):

                    if (x_face + int(w_face/2) <= (2*x + int(w))/2 + 80 and x_face + int(w_face/2) >= (2*x + int(w))/2 - 80):
                        if(w >= 5 and h >= 5):
                            fl = True

                    if fl:
                        countourList.append([x,y,w,h])

                    countourList.sort(key = sortPixels, reverse = True)

                    if(len(countourList) == 1):
                        for i in range(0,1):
                            #print(len(countourList), 'lllllllllllllllllll')
                            cList = countourList[i]
                            #cv.rectangle(blank_image, (cList[0], cList[1]), (cList[0] + cList[2], cList[1] + cList[3]), (0,0,0), -1)

                        contour_only_2 = []
                        contour_only_2.append(countourList[0])

                    if(len(contour_only_2) == 1):
                        coordinates['blue_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                        coordinates['blue_y'] = contour_only_2[0][1]
                        coordinates['red_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                        coordinates['red_y'] = contour_only_2[0][1]

                        coordinates['c_red_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                        coordinates['c_red_y'] = contour_only_2[0][1] + contour_only_2[0][3]/2
                        coordinates['c_blue_x'] = contour_only_2[0][0] + contour_only_2[0][2]/2
                        coordinates['c_blue_y'] = contour_only_2[0][1] + contour_only_2[0][3]/2

        #cv.imshow('Binary', blank_image)        
    return coordinates


def cropping_area(c_x, c_y, img_w, img_h, sz=64):
    r = sz//2
    w_lb = c_x - r if c_x - r >= 0 else 0
    w_ub = c_x + r if c_x + r < img_w else img_w - 1    
    h_lb = c_y - r if c_y - r >= 0 else 0
    h_ub = c_y + r if c_y + r < img_h else img_h - 1
    if w_ub - w_lb != sz:
        if w_ub != img_w-1:
            w_ub += sz - (w_ub - w_lb)
    if h_ub - h_lb != sz:
        if h_ub != img_h-1:
            h_ub += sz - (h_ub - h_lb)
    
    return int(w_lb), int(w_ub), int(h_lb), int(h_ub)


def get_radius_of_hands(image, hands_location):    
    features = []
    height, width, _ = image.shape
    # Threshold of blue in HSV space
    thrshold_red = {
        'red1': [[180, 255, 255], [159, 50, 70]],
        'red2': [[9, 255, 255], [0, 50, 70]]
    }
    lower = np.array(thrshold_red['red1'][1])
    upper = np.array(thrshold_red['red1'][0])

    skipped = False
    for (c_x, c_y) in hands_location:
        w_lb, w_ub, h_lb, h_ub = cropping_area(c_x, c_y, width, height)
        c_img = image[h_lb:h_ub, w_lb:w_ub]
        img_hsv = cv.cvtColor(c_img, cv.COLOR_BGR2HSV)
        # preparing the mask to overlay
        mask = cv.inRange(img_hsv, lower, upper)
        hand_img = cv.bitwise_and(c_img, c_img, mask = mask)
        g_hand_img = cv.cvtColor(hand_img ,cv.COLOR_BGR2GRAY)
        b_hand_img = np.where(g_hand_img == 0, 0, 255)
        g_hand_img[:,:] = b_hand_img
        cnts = cv.findContours(g_hand_img.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)                                        

        if len(cnts) == 2:
            cnts = cnts[0]
        elif len(cnts) == 3:
            cnts = cnts[1]
        else:
            print('Error length for contour.')
            cnts = []

        if cnts:
            cnts = sorted(cnts, key = cv.contourArea, reverse = True)[0]
            _, radius = cv.minEnclosingCircle(cnts)
            features.append(radius)
        else:
            skipped = True

    if skipped:
        return []
    return features

def get_possibility_of_pose(radius):    
    def normpdf(x, mean, sd):
        var = float(sd)**2
        denom = (2*math.pi*var)**.5
        num = math.exp(-(float(x)-float(mean))**2/(2*var))
        return num/denom
        
    poses_table = {              #[(left_mean, left_std),(right_mean, right_std)]
        "full-time":              [(14.565412, 4.160434),(17.140450, 4.420539)],
        "goal-kick-blue":         [(18.339703, 0.493482),(21.538291, 4.334216)],
        "goal-blue":              [(13.633775, 2.652553),(18.827396, 0.886346)],
        "goal-kick-red":          [(17.067477, 1.503005),(20.906887, 1.882879)],
        "goal-red":               [(15.235459, 1.023173),(14.186027, 3.333722)],
        "corner-kick-blue":       [(18.078065, 0.639560),(18.979960, 1.718608)],
        "corner-kick-red":        [(17.408690, 0.651424),(20.335243, 1.451128)],
        "kick-in-blue":           [(18.270560, 0.428223),(19.689457, 0.782369)],
        "kick-in-red":            [(15.862661, 2.057362),(21.231735, 2.907621)],
        "pushing-free-kick-blue": [(19.589211, 4.449950),(19.148688, 0.818958)],
        "pushing-free-kick-red":  [(15.285954, 1.494469),(19.616409, 6.430259)]
    }
    
    possibility_table = {}
    for r in radius:
        for pose in poses_table.keys():
            p_val = 1.0
            for (mean, std) in poses_table.get(pose):
                p_val *= normpdf(r, mean, std)
            possibility_table.setdefault(pose, p_val)

    max_val = max(possibility_table.values())
    for pose in possibility_table.keys():
        possibility_table[pose] = 1 - (possibility_table[pose]/max_val)

    return sorted(possibility_table.items(), key=lambda x: x[1])

def image_angles(frame):
    # play_sound(None,True)
    if (frame is not None):
        dimensions = frame.shape
        print("Loaded image size: ", dimensions)
        #cv.imshow('Frame', frame)
        maps = get_coordinates(frame)
        print("Detected Map: ", maps)
        
        none = False
        for key in maps:
            if maps[key] is None:
                none = True
                break

        if (not none):
            red_x = maps['red_x']
            red_y = maps['red_y']
            blue_x = maps['blue_x']
            blue_y = maps['blue_y']
            face_x = maps['face_x']
            face_y = maps['face_y']

            # Radius features
            # left_hand_loc = (maps['c_red_x'], maps['c_red_y'])
            # right_hand_loc = (maps['c_blue_x'], maps['c_blue_y'])
            # radius_of_hands = get_radius_of_hands(frame, [left_hand_loc, right_hand_loc])
            # if len(radius_of_hands) == 2:
            #     possibility_of_pose = get_possibility_of_pose(radius_of_hands)
            #     print("Possibility of pose:\n", possibility_of_pose)
            # else:
            #     prediction = 'Unable to detect'

            bx_blue = face_x - blue_x
            by_blue = blue_y - face_y

            bx_red = red_x - face_x
            by_red = red_y - face_y

            distance_face_blue = ((blue_x - face_x)**2 + (blue_y - face_y)**2)**0.5
            distance_face_red = ((red_x - face_x)**2 + (red_y - face_y)**2)**0.5

            blue_angle_x = math.acos(bx_blue/distance_face_blue)
            blue_angle_y = math.acos(by_blue/distance_face_blue)

            red_angle_x = math.acos(bx_red/distance_face_red)
            red_angle_y = math.acos(by_red/distance_face_red)

            maps['red_x'] = maps['red_x']/maps['face_x']
            maps['red_y'] = maps['red_y']/maps['face_y']

            maps['blue_x'] = maps['blue_x']/maps['face_x']
            maps['blue_y'] = maps['blue_y']/maps['face_y']

            maps['face_x'] = maps['face_x']/maps['face_x']
            maps['face_y'] = maps['face_y']/maps['face_y']

            # print(math.degrees(red_angle_y), '----------------------- Degree')

            prediction = ''
            # play_sound(None,True)
            if(maps['blue_x'] < 0.9 and maps['blue_y'] < 0.9 and maps['red_y'] >= 1.4):
                prediction = 'goal kick blue'

            elif(maps['red_x'] > 1 and maps['red_y'] < 0.9 and maps['blue_y'] >= 1.4):
                prediction = 'goal kick red'

            elif(red_y <= face_y + 140 and blue_y <= face_y + 140):

                #if(math.degrees(blue_angle_x) >= 0 and math.degrees(blue_angle_x) < 30 and math.degrees(red_angle_x) >= 0 and math.degrees(red_angle_x) < 30):
                if((abs(red_x - face_x)/abs(blue_x - face_x) >= 0.7) and (abs(red_x - face_x)/abs(blue_x - face_x) <= 1.45) and (red_x > face_x) and (blue_x < face_x)):
                    prediction = 'full time'

                elif((distance_face_blue/distance_face_red) >= 1.9):
                        prediction = 'pushing freekick blue'

                elif((distance_face_red/distance_face_blue) >= 1.9):
                        prediction = 'pushing freekick red'

                elif(maps['red_y']/maps['blue_y'] >= 0.8 and maps['red_y']/maps['blue_y'] <= 1.3 and ((distance_face_blue/distance_face_red) >= 2 or (distance_face_red/distance_face_blue) >= 2)):
                    if(math.degrees(blue_angle_x) >= 0 and math.degrees(blue_angle_x) < 35):
                            prediction = 'pushing freekick blue'

                    elif(math.degrees(red_angle_x) >= 0 and math.degrees(red_angle_x) < 35):
                            prediction = 'pushing freekick red'


            elif(math.degrees(blue_angle_x) >= 0 and math.degrees(blue_angle_x) < 35 and math.degrees(red_angle_y) > 5 and math.degrees(red_angle_y) <=25):
                if(abs(red_x/face_x) <=1.05):
                    prediction = 'goal blue'
                elif(red_x > face_x):
                    prediction = 'kick in blue'
                else:
                    prediction = 'goal blue'

            elif(math.degrees(blue_angle_x) >= 0 and math.degrees(blue_angle_x) < 35 and math.degrees(red_angle_y) >=0 and math.degrees(red_angle_y) <=5):
                prediction = 'goal blue'

            elif(math.degrees(red_angle_x) >= 0 and math.degrees(red_angle_x) < 35 and math.degrees(blue_angle_y) >=5 and math.degrees(blue_angle_y) <=25):
                if(abs(blue_x/face_x) >=0.95):
                    prediction = 'goal red'
                elif(blue_x < face_x):
                    prediction = 'kick in red'
                else:
                    prediction = 'goal red'

            elif(math.degrees(red_angle_x) >= 0 and math.degrees(red_angle_x) < 35 and math.degrees(blue_angle_y) >=0 and math.degrees(blue_angle_y) <=5):
                prediction = 'goal red'

            elif(math.degrees(blue_angle_x) >= 30 and math.degrees(blue_angle_x) <= 70 and math.degrees(red_angle_y) >=0 and math.degrees(red_angle_y) <=25 and (red_x >= face_x)):
                prediction = 'corner kick blue'

            elif(math.degrees(red_angle_x) >= 30 and math.degrees(red_angle_x) <= 70 and math.degrees(blue_angle_y) >=0 and math.degrees(blue_angle_y) <=25 and (blue_x <= face_x)):
                prediction = 'corner kick red'

            print('prediction-->', prediction)
            
            if prediction != '':
                print('prediction-->', prediction)
                print('blue angle x-->', math.degrees(blue_angle_x))
                print('blue angle y-->', math.degrees(blue_angle_y))
                print('red angle x-->', math.degrees(red_angle_x))
                print('red angle y-->', math.degrees(red_angle_y))
            return prediction

        else:
            prediction = 'Unable to detect'
            print('prediction-->', prediction)
            

    

# def play_sound(predicted, initialize=False):

#     ip = "192.168.100.155"
#     port = 9559
    
#     if (predicted is not None or ''):
#         speech = ALProxy("ALTextToSpeech", ip, port)
#         speech.say(predicted)
#         #time.sleep(1)

    
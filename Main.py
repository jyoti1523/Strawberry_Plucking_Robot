import cv2 as cv
import RPi.GPIO as GPIO
from numpy import *
import numpy as np
import os
import time
from time import sleep
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


def servo(xarm,yarm):
    px = yarm
    py = xarm  

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(13,GPIO.OUT)
    GPIO.setup(12,GPIO.OUT)
    GPIO.setup(19,GPIO.OUT)

    pwm=GPIO.PWM(13,50)
    pwm1=GPIO.PWM(12,50)
    pwm2=GPIO.PWM(19,50)

    pwm.start(0)
    pwm1.start(0)
    pwm2.start(0)
    def setangle(angle):
        duty=angle/18+2
        GPIO.output(13,True)
        pwm.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(13,False)
        pwm.ChangeDutyCycle(0)
    def setangle1(angle1):
        duty1=angle1/18+2
        GPIO.output(12,True)
        pwm1.ChangeDutyCycle(duty1)
        sleep(1)
        GPIO.output(12,False)
        pwm1.ChangeDutyCycle(0)
    def setangle2(angle2):
        duty2=angle2/18+2
        GPIO.output(19,True)
        pwm2.ChangeDutyCycle(duty2)
        sleep(1)
        GPIO.output(19,False)
        pwm2.ChangeDutyCycle(0)
    #Arm_Code


    # Length of links in cm
    a1= 9
    a2 = 9
    a3 = 7

    # Desired Position of End effector


    phi = 90
    phi = deg2rad(phi)

    # Equations for Inverse kinematics
    wx = px - a3*cos(phi)
    wy = py - a3*sin(phi)

    delta = wx**2 + wy**2
    c2 = ( delta -a1**2 -a2**2)/(2*a1*a2)
    s2 = sqrt(1-c2**2)  # elbow down
    theta_2 = arctan2(s2, c2)
    theta_2=rad2deg(theta_2)
    theta22=90-theta_2

    s1 = ((a1+a2*c2)*wy - a2*s2*wx)/delta
    c1 = ((a1+a2*c2)*wx + a2*s2*wy)/delta
    theta_1 = arctan2(s1,c1)
    theta_1=rad2deg(theta_1)
    theta11=90-theta_1

    theta_3 = phi-theta_1-theta_2
    theta33=135+theta_3
    print('theta_1: ', theta11)
    print('theta_2: ', theta22)
    print('theta_3: ', theta33)


    setangle(theta11)
    sleep(1)
    setangle1(theta22)
    sleep(1)
    setangle2(theta33)
    
    sleep(1)


    if(theta11<180 and theta11>0)and (theta22<180 and theta22>0)and (theta33<180 and theta33>0):
        setangle(theta11)

        print('I am on THeta 1')
        sleep(1)

        setangle1(theta22)
        print('I am on THeta 2')
        sleep(1)
        setangle2(theta33)
        print('I am on THeta 3')
        sleep(1)
        print('I will pluck strawberry')
    else:
        print('Hardware Limitations!!!')


        #setangle(0)
        #setangle1(0)
        #setangle2(0)
        #sleep(3)
    pwm.stop()
    pwm1.stop()
    pwm2.stop()
def armcord(up,dn):
    x=float(up*23)
    x=float(x/600)
    
    y=float(dn*17)
    y=float(y/600)
    x=round(x,2)
    y=round(y,2)
    
    return(x,y)
#detection code        
confThreshold = 0.8
nmsThreshold = 0.40
inpWidth = 416
inpHeight = 416

classesFile = "obj.names";
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

modelConf = 'yolov3-tiny-obj.cfg'
modelWeights = 'yolov3-tiny-obj_2000.weights'

def postprocess(frame, outs):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]

    classIDs = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            
            scores = detection [5:]
            classID = np.argmax(scores)
            confidence = scores[classID]

            if confidence > confThreshold:
                centerX = int(detection[0] * frameWidth)
                centerY = int(detection[1] * frameHeight)

                width = int(detection[2]* frameWidth)
                height = int(detection[3]*frameHeight )

                left = int(centerX - width/2)
                top = int(centerY - height/2)

                classIDs.append(classID)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])

    indices = cv.dnn.NMSBoxes (boxes,confidences, confThreshold, nmsThreshold )

    indices = cv.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        
        drawPred(classIDs[i], confidences[i], left, top, left + width, top + height)

def drawPred(classId, conf, left, top, right, bottom):
    # Draw a bounding box.
    cv.rectangle(frame, (left, top), (right, bottom), (255, 255, 255), 3)
    
    up=abs(left+right)
    dn=abs(top+bottom)
    up=int(up/2)
    dn=int(dn/2)
    cv.circle(frame,(up,dn),2,(0,255,0),-1)

    while(0<up<240 and 360<up<600):
        if(up>0 and up<240):
            os.system('python backward.py')
        elif(up>360 and up<600):
            os.system('python forward.py')
        
    print(up,dn)
    time.sleep(1)
    os.system('python stop.py')
    xarm,yarm=armcord(up,600-dn)
    print(xarm,'X for arm')
    print(yarm,'Y for arm')

    servo(xarm,yarm)
    label = '%.2f' % conf

    if classes:
        assert (classId < len(classes))
        label = '%s:%s' % (classes[classId], label)

    cv.putText(frame, label, (left,top), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

def getOutputsNames(net):

    layersNames = net.getLayerNames()
   
    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

net = cv.dnn.readNetFromDarknet(modelConf, modelWeights)
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

winName = 'strawberry detection'
cv.namedWindow(winName, cv.WINDOW_NORMAL)
cv.resizeWindow(winName, 600,600)

import time
fpsLimit=0.25
startTime=time.time()

os.system('python DC_MOtorpy.py')
while cv.waitKey(1) < 0:
    
#    hasFrame, frame = cap.read()
    os.system('python DC_MOtorpy.py')
    cap = cv.VideoCapture(0)
    time.sleep
    hasFrame, frame = cap.read()
    cap.release()

    frame=cv.resize(frame,(600,600))
    grid = 120
    height,width=600,600
    for x in range(0,width-1,grid):
        
        cv.line(frame,(x,0),(x,height),(0,0,255),1,1)
     
                     
    nowTime=time.time()
    if (nowTime-startTime)>fpsLimit:
    
        blob = cv.dnn.blobFromImage(frame, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop = False)

        net.setInput(blob)
        outs = net.forward (getOutputsNames(net))

        postprocess (frame, outs)

        cv.imshow(winName, frame)
        startTime=time.time()

import time
import RPi.GPIO as GPIO
import os
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
#print('I am in run file')
#mot1=white
in1=23
in2=24
en1=25

#mot2=yellow
in4=12
in3=16
en2=20

#mot3=orange
in5=17
in6=27
en3=22

#mot4=
in7=5
in8=6
en4=26
temp=1
#motor1
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)

#motor2
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)

#m3
GPIO.setup(in5,GPIO.OUT)
GPIO.setup(in6,GPIO.OUT)
GPIO.setup(en3,GPIO.OUT)

#m4
GPIO.setup(in7,GPIO.OUT)
GPIO.setup(in8,GPIO.OUT)
GPIO.setup(en4,GPIO.OUT)

p1=GPIO.PWM(en1,1000)
p2=GPIO.PWM(en2,1000)
p3=GPIO.PWM(en3,1000)
p4=GPIO.PWM(en4,1000)

p1.start(5)
p2.start(5)
p3.start(5)
p4.start(5)






GPIO.output(in1,GPIO.HIGH)
GPIO.output(in2,GPIO.LOW)
#print("mot1")
GPIO.output(in3,GPIO.HIGH)
GPIO.output(in4,GPIO.LOW)
#print("mot2")
GPIO.output(in5,GPIO.LOW)
GPIO.output(in6,GPIO.HIGH)
#print("mot3")
GPIO.output(in7,GPIO.LOW)
GPIO.output(in8,GPIO.HIGH)
#print("mot4")
time.sleep(5)
# stop motor

GPIO.output(en1,GPIO.LOW)
GPIO.output(en2,GPIO.LOW)
GPIO.output(en3,GPIO.LOW)
GPIO.output(en4,GPIO.LOW)
print("I am in forward file")

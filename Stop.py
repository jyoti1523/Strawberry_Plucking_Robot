import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
print('I will stop')
in1=23
in2=24
en1=25

in4=12
in3=16
en2=20

in5=17
in6=27
en3=22

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

GPIO.output(en1,GPIO.LOW)
GPIO.output(en2,GPIO.LOW)
GPIO.output(en3,GPIO.LOW)
GPIO.output(en4,GPIO.LOW)

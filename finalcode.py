import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from PIL import Image
import picamera.array
from picamera import PiCamera
import requests
import datetime
import threading
import math

def sendMessage(message):
    token='bot5893058754:AAF4k51mk1lc1zoYFQMdJ50C1ksG8C1IxwU'
    chatid='-1001658011959'
    response=requests.get('https://api.telegram.org/'+token+'/sendMessage?chat_id='+chatid+'&text='+message)
    
def sendPhoto(filename):
    token='bot5893058754:AAF4k51mk1lc1zoYFQMdJ50C1ksG8C1IxwU'
    chatid='-1001658011959'
    f=open('/home/pi/'+filename,'rb')
    file={'photo':f}
    ctime=datetime.datetime.now()
    stime=str( ctime)
    response=requests.post("https://api.telegram.org/"+token+"/sendPhoto?chat_id="+chatid+'&caption='+stime+"\n Human Body Found",files=file)
               
def frame2img(frame,name):
    data=Image.fromarray(frame)
    data.save(name+'.jpg')

def detecthumans():
    cam=PiCamera()
    time.sleep(1)
    hog = cv2.HOGDescriptor() 
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    cam.resolution=(640,480)
    while True:
        rawcap=picamera.array.PiRGBArray(cam)
        start=time.time()
        cam.capture(rawcap,format='bgr')
        frame=rawcap.array
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (regions, weight) = hog.detectMultiScale(gray_image, winStride=(2, 2),padding=(4, 4),scale=1.1)
        #print(regions,weight)
        found=False
        for i in weight:
            if i>0.30:
                found=True
                
        if(found):
            frame2img(frame,'lastfound')
            print("Human Body Found")
            time.sleep(1)
            sendPhoto('lastfound.jpg')
            time.sleep(10)
    
        #print('time taken',time.time()-start)
        #frame2img(gray_image,'gray_imagecap')
        #frame2img(frame,'cap')
        #time.sleep(1)



def monitorsensors():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO_TRIGGER = 23
    GPIO_ECHO = 24
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)
    SPI_PORT   = 0
    SPI_DEVICE = 0
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
    
    def distance():
    # set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)

        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
 
        StartTime = time.time()
        StopTime = time.time()
 
    # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
 
    # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
 
    # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
 
        return distance

    
    
    while True:
        mq4value=math.floor((10000.0/4096.0)*mcp.read_adc(0)+200)
        mq7value=math.floor(((mcp.read_adc(7)-310)/760)*100)
        depth=math.floor(distance())
        print(str(mq4value))
        print('Methane Concentration-',mq4value,' ppm')
        print('Carbon Monoxide Concentration-',mq7value,' ppm')
        print('Height of water-',depth,"cm from top")
        time.sleep(1)
        
        ctime=datetime.datetime.now()
        stime=str(ctime)
        
        
        if(mq4value>1000):
            if(mq4value>5000):
                sendMessage('Methane Concentration Highly Critital : '+str(mq4value)+' ppm\n'+stime)
            else:
                sendMessage('Methane Concentration Hign : '+str(mq4value)+' ppm\n'+stime )
            time.sleep(30)
        if(mq7value>50):
            if(mq7value>200):
                sendMessage('Carbon Monoxide Concentratio Lethal : '+str(mq7value)+' ppm\n'+stime)
            else:
                sendMessage('Carbon Monoxide Concentration High : '+str(mq7value)+' ppm\n'+stime)
            time.sleep(30)
            
        if(depth<5):
            sendMessage('Height of water ' + str(depth) +' cm from top')
            time.sleep(5)
    
    
camerathread=threading.Thread(target=detecthumans)
sensorsthread=threading.Thread(target=monitorsensors)

camerathread.start()
sensorsthread.start()
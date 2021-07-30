import numpy as np
import jetson.inference
import jetson.utils
import argparse
import sys
import time
import threading
import math
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import cv2 as cv
import face_recognition
import os
import pickle

myKit = ServoKit(channels=16)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

in1 = 19
in2 = 26
in3 = 5
in4 = 6
en2 = 12
en = 13
temp1 = 1

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)

GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

p = GPIO.PWM(en, 10000)
p2 = GPIO.PWM(en2, 10000)
p.start(90)
p2.start(90)


panAngle = 60
tiltAngle = 60
targetOffset = 0
width = 1280
height = 720
x_lock = 0
y_lock = 0
evt = -1
motor_run_time = 4
target = "NIL"
lockOn = False ########
auto = True
lockedTarget = False
font = cv.FONT_HERSHEY_COMPLEX_SMALL
targetClr = [0,0,0]
clrOffset = 60
timeStamp = time.time()
fpsFilt = 0
face = False
targetCount = 0
fire = False

myKit.servo[3].angle = 123
input = jetson.utils.videoSource('csi://0',  ['-flip-method=rotate-180','–input-width=3264', '–input-height=2464'])
#3264x2464

Names = []
Encodings = []

with open('train.pkl','rb') as f:
    Names=pickle.load(f)
    Encodings=pickle.load(f)

net = jetson.inference.detectNet("ssd-mobilenet-v1", threshold=0.4)

#net = jetson.inference.detectNet(argv=['--model=jetson-inference/python/training/detection/ssd/models/banana/ssd-mobilenet.onnx', '--labels=jetson-inference/python/training/detection/ssd/models/banana/labels.txt', '--input-blob=input_0','--output-cvg=scores', '--output-bbox=boxes'],threshold=0.5)

def click(event, xSelect, ySelect, flags, params):
	global x_lock
	global y_lock
	global evt
	if event == cv.EVENT_LBUTTONDOWN:
		x_lock = xSelect
		y_lock = ySelect
		evt = event

def fire_init():
	fire = False
	trigger_timer = motor_run_time
	GPIO.output(in1, GPIO.HIGH)
	GPIO.output(in2, GPIO.LOW)
	GPIO.output(in3, GPIO.HIGH)
	GPIO.output(in4, GPIO.LOW)
	for x in range(trigger_timer):
		trigger_timer -= 1
		time.sleep(1)
	myKit.servo[3].angle = 68
	trigger_timer = 1
	for x in range(trigger_timer):
		trigger_timer -= 1
		time.sleep(1)
	myKit.servo[3].angle = 123
	GPIO.output(in1, GPIO.LOW)
	GPIO.output(in2,GPIO.LOW)
	GPIO.output(in3, GPIO.LOW)
	GPIO.output(in4, GPIO.LOW)

def get_target_count(targets):
	#targets.count("PERSON")
 	pass
 	
 	
cv.namedWindow('feed',  flags=cv.WINDOW_OPENGL)
cv.setMouseCallback('feed', click)

while True:
	myKit.servo[0].angle = 180 - panAngle
	myKit.servo[1].angle = 180 - tiltAngle
	
	img = input.Capture()

	detections = net.Detect(img, width, height, overlay='none')
	frame = jetson.utils.cudaToNumpy(img, width, height, 4)

	frame = cv.cvtColor(frame.astype(np.uint8), cv.COLOR_RGBA2BGR)
	
	targetTxtsize = cv.getTextSize("TRG " + target,font,1,1)
	cv.putText(frame, "TRG " + target, (width - targetTxtsize[0][0] -10,620), font, 1, (255,255,255), 1)
	
	tcolorSize = cv.getTextSize("TRG CLR "+str(targetClr[1]) + " " + str(targetClr[2]) + " " + str(targetClr[0]),font,1,1)
	cv.putText(frame, "TRG CLR "+ str(targetClr[1]) + " " + str(targetClr[2]) + " " + str(targetClr[0]), (width - tcolorSize[0][0] -10, 560), font, 1, (255,255,255), 1)
	
	offsetTxtsize = cv.getTextSize("CLR OFFSET "+str(clrOffset),font,1,1)
	#cv.rectangle(frame, (width - colorSize[0][0] - 10,590),(1270,620), (255,255,255), -1)
	cv.putText(frame,"CLR OFFSET "+ str(clrOffset), (width - offsetTxtsize[0][0] -10, 530), font, 1, (255,255,255), 1)
	
	cv.rectangle(frame, (1200,670),(1270,700), (255,255,255), -1)
	cv.putText(frame, "CLEAR", (1200,690), font, 1, (0,0,0), 1)
	
	if (target != "NIL" and lockOn == False):
		cv.rectangle(frame, (10,630),(70,660), (255,255,255), -1)
		cv.putText(frame, "LOST", (10,650), font, 1, (0,0,0), 1)
	
	if (auto):
		cv.rectangle(frame, (129,10),(191,29), (255,255,255), -1)
		cv.putText(frame, "AUTO", (130,25), font, 1, (0,0,0), 1)
	else:
		cv.rectangle(frame, (129,10),(191,29), (255,255,255), -1)
		cv.putText(frame, "MANL", (130,25), font, 1, (0,0,0), 1)
	#fire
	if (x_lock >= 10 and x_lock <= 67 and y_lock >= 630 and y_lock <= 660 or fire == True):
		print("active")
		init = threading.Thread(target = fire_init)
		init.start()
		x_lock = 0
		y_lock = 0
		fire = False
	#clear
	if (x_lock >= 1200 and x_lock <= 1270 and y_lock >= 670 and y_lock <= 700):
		evt = -1
		target = "NIL"
		lockOn = False
		tiltAngle = 60
		panAngle = 60
		targetClr = [0,0,0]
		x_lock = 0
		y_lock = 0
		myKit.servo[3].angle = 123
		face = False
		fire = False
	#auto
	if (x_lock >= 129 and x_lock <= 191 and y_lock >= 10 and y_lock <= 29):
		break
	
	#print(len(detections))
	
	for detect in detections:
		ID = detect.ClassID
		top = int(detect.Top)
		left = int(detect.Left)
		bottom = int(detect.Bottom)
		right = int(detect.Right)
		centreY = int ((top + bottom) / 2)
		centreX = int ((left + right) / 2)
		item = net.GetClassDesc(ID).upper()
		

		cv.line(frame, (left, top), (left + 20, top), (255,255,255), 2)
		cv.line(frame, (left, top), (left, top + 20), (255,255,255), 2)
		cv.line(frame, (right, top), (right - 20, top), (255,255,255), 2)
		cv.line(frame, (right, top), (right, top + 20), (255,255,255), 2)

		cv.line(frame, (left, bottom), (left + 20, bottom), (255,255,255), 2)
		cv.line(frame, (left, bottom), (left, bottom - 20), (255,255,255), 2)
		cv.line(frame, (right, bottom), (right - 20, bottom), (255,255,255), 2)
		cv.line(frame, (right, bottom), (right, bottom - 20), (255,255,255), 2)

		cv.putText(frame, item, (int(((left + right) / 2)-30),top), font, 1, (255,255,255), 2)
		
		# face detection init -----------------------------------------------
		if (item == "PERSON" and targetClr[0] == 0):
			x_lock = centreX
			y_lock = centreY
			evt = 1
		
		if (evt == 1):
			if (len(detections) != 0 and x_lock >= detect.Left and x_lock <= detect.Right and y_lock <= detect.Bottom and y_lock >= detect.Top):
				target = item
				
				if (target == "PERSON"):
					targetOffset = 90
				else:
					targetOffset = 0
				evt = -1
				#selected clr on lock
				targetClr[1] = frame[centreY - targetOffset,centreX,0]
				targetClr[2] = frame[centreY - targetOffset,centreX,1]
				targetClr[0] = frame[centreY - targetOffset,centreX,2]
				
		#live clr		
		blue = frame[centreY - targetOffset,centreX,0]
		green = frame[centreY - targetOffset,centreX,1]
		red = frame[centreY - targetOffset,centreX,2]
		# clr after dev calc
		blueDev = [targetClr[1] - clrOffset, targetClr[1] + clrOffset]
		greenDev = [targetClr[2] - clrOffset, targetClr[2] + clrOffset]
		redDev = [targetClr[0] - clrOffset, targetClr[0] + clrOffset]
		

		if (lockOn):
		
			cv.rectangle(frame, (10,590),(70,620), (255,255,255), -1)
			cv.putText(frame, "LOCK", (10,610), font, 1, (0,0,0), 1)
			cv.rectangle(frame, (10,630),(67,660), (255,255,255), -1)
			cv.putText(frame, "FIRE", (10,650), font, 1, (0,0,0), 1)
			
			if (target == "PERSON" and face == False):
				cutout = frame[left:right, top:bottom]
				try:
					frameSmall = cv.resize(cutout,(0,0),fx=.50,fy=.50)
				except cv.error as e:
					print('invalid frame')
				#frameRGB=cv.cvtColor(frameSmall,cv.COLOR_BGR2RGB)
				facePositions = face_recognition.face_locations(frameSmall, model='cnn')
				allEncodings = face_recognition.face_encodings(frameSmall, facePositions)
				for (top,right,bottom,left),face_encoding in zip(facePositions,allEncodings):
					#name = 'Unkown Person'
					matches = face_recognition.compare_faces(Encodings,face_encoding)
					if True in matches:
						first_match_index = matches.index(True)
						name = Names[first_match_index]
						target = "NIL"
						targetClr = [0,0,0]
						lockOn = False
						face = True
						print("MATCH", name)			
					else:
						face = True
						fire = True
						print("UNKNOWN")
						
					#top=top*4
					#right=right*4
					#bottom=bottom*4
					#left=left*4
					#cv.rectangle(frame,(left,top),(right, bottom),(0,0,255),2)
					#cv.putText(frame,name,(left,top-6),font,.75,(0,0,255),2)
				
				x_lock = 0
				y_lock = 0
				

			#offsetTxtsize = cv.getTextSize("TRG SIZE "+ str(right - left),font,1,1)
			#cv.rectangle(frame, (width - colorSize[0][0] - 10,590),(1270,620), (255,255,255), -1)
			#cv.putText(frame,"TRG SIZE "+ str(right - left), (width - offsetTxtsize[0][0] -10, 440), font, 1, (255,255,255), 1)

			cv.line(frame, (int(width / 2) - 30, int(height / 2)), (int(width / 2) - 10, int(height / 2)), (255,255,255), 2)
			cv.line(frame, (int(width / 2) + 30, int(height / 2)), (int(width / 2) + 10, int(height / 2)), (255,255,255), 2)
			cv.line(frame, (int(width / 2), int(height / 2) - 30), (int(width / 2), int(height / 2) - 10), (255,255,255), 2)
			cv.line(frame, (int(width / 2), int(height / 2) + 30),(int(width / 2), int(height / 2) +  10) , (255,255,255), 2)
			
			colorSize = cv.getTextSize("CLR "+str(red) + " " + str(blue) + " " + str(green),font,1,1)
			cv.putText(frame, "CLR "+ str(red) + " " + str(blue) + " " + str(green), (width - colorSize[0][0] -10,590), font, 1, (255,255,255), 1)

		if (item == target and blue >= blueDev[0] and blue <= blueDev[1] and green >= greenDev[0] and green <= greenDev[1] and red >= redDev[0] and red <= redDev[1]):

			
			if (width/2 <= right and width/2 >= left and height/2 <= bottom and height/2 >= top):			
				lockOn = True

			frame = cv.circle(frame, (centreX, centreY - targetOffset), 5, (0,0,255), -1)
			panError = int((width / 2) - centreX)
			tiltError = int((height / 2) - centreY + targetOffset)

			cv.putText(frame, "P-E "+ str(abs(panError)),(10,530),font,1,(255,255,255),1)
			cv.putText(frame, 'T-E '+ str(abs(tiltError)), (10,560), font, 1, (255,255,255), 1)
			
			if (abs(panError) >= 1 and int(panAngle - panError / 75) <= 120 and int(panAngle - panError / 75) >= 0):
				panAngle = int(panAngle - panError / 75)
			if (abs(tiltError) >= 1 and int(tiltAngle + tiltError / 75) <= 120 and int(tiltAngle + tiltError / 75) >= 0):
				tiltAngle = int(tiltAngle + tiltError / 75)
		else:
			lockOn = False #######

	dt=time.time()-timeStamp
	timeStamp=time.time()
	fps=1/dt
	fpsFilt=.9*fpsFilt + .1*fps
	cv.putText(frame,'FPS ' + str(round(fpsFilt,1)),(10,25),font,1,(255,255,255),1)
	cv.putText(frame, 'P ' + str(panAngle), (10,45), font, 1, (255,255,255), 1)
	cv.putText(frame, 'T ' + str(tiltAngle), (10,65), font, 1, (255,255,255), 1)	
	# arrow
	cv.circle(frame, (int(width/2) + 60,height - 55), 50, (255,255,255), 1)
	cv.circle(frame, (int(width/2) - 60,height - 55), 50, (255,255,255), 1)
	cv.arrowedLine(frame, (int(width/2) + 60, height - 55), (round(-50 * math.cos(math.radians(panAngle)) + (width/2) + 60), round((height-55) - (50 * math.sin(math.radians(panAngle))))), (255,255,255), 1, tipLength = .2)
	cv.line(frame, (int(width/2) + 60, height - 55), (int(width / 2) + 60, height - 105), (255,255,255), 1)
	cv.imshow('feed',frame)
	cv.moveWindow('feed',0,20)
	cv.resizeWindow('feed', 1280,720)
	keyEvent = cv.waitKey(1)

		

		
GPIO.cleanup()
cv.destroyAllWindows()

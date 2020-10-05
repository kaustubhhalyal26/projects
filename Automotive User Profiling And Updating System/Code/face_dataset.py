''''
Capture multiple Faces from multiple users to be stored on a DataBase (dataset directory)
	==> Faces will be stored on a directory: dataset/ (if does not exist, pls create one)
	==> Each face will have a unique numeric integer ID as 1, 2, 3, etc                       

Based on original code by Anirban Kar: https://github.com/thecodacus/Face-Recognition    

Developed by Marcelo Rovai - MJRoBot.org @ 21Feb18    

'''
from file_handling import *
import cv2
import os
import random

def data_entry():
	cam = cv2.VideoCapture(0)
	cam.set(3, 640) # set video width
	cam.set(4, 480) # set video height

	face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

	# For each person, enter one numeric face id 
	id=[]
	id=readlistint("id")
	if len(id)==0:
		face_id = 0
	else:
		id.sort()
		face_id=id[-1]+1

	print("\n [INFO] Initializing face capture. Look the camera and wait ...")
	# Initialize individual sampling face count
	count = 0

	while(True):

	    ret, img = cam.read()
	    #img = cv2.flip(img, -1) # flip video image vertically
	    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	    faces = face_detector.detectMultiScale(gray, 1.3, 5)

	    for (x,y,w,h) in faces:

	        cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)     
	        count += 1

	        # Save the captured image into the datasets folder
	        cv2.imwrite("dataset/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])

	        cv2.imshow('image', img)

	    k = cv2.waitKey(100) & 0xff # Press 'ESC' for exiting video
	    if k == 27:
	        break
	    elif count >= 30: # Take 30 face sample and stop video
	         break

	# Do a bit of cleanup
	print("\n [INFO] Exiting Program and cleanup stuff")
	cam.release()
	cv2.destroyAllWindows()

	#update the id file
	id.append(face_id)
	writelistint(id,"id")
	return face_id



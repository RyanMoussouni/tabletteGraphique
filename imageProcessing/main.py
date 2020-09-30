#### Packages #####
from threading import Thread
from picamera.array import PiRGBArray
from picamera import PiCamera
import io
import time
import picamera
import datetime
import cv2
import numpy as np
import imutils
import socket  
import threading, Queue
import matplotlib.colors as colors
###############################################################################################################
###################################   Variables partagees entre les threads   #################################
###############################################################################################################




pression= False
thickness = 10
erase =  False
color=(0,0,0,255)



width,height = 1100,600


CompteurCalibrage = -1
L=[]
LC=[]
C=[]
cnts=[]


img = np.ones((height,width,3), np.uint8)*255

#AFFICHAGE DE LA CAMERA = True    (dev only)
affichage = False


def HexToRGB(hex_string):
    hx = "#"+hex_string[2:]
    rgb = colors.hex2color(hx)
    transp = int(hex_string[:2],16)
    return (tuple([int(255*x) for x in rgb]),transp)


def communicationbluetooth():
    import serial
    import time
    global pression
    broken=True;
    ser = serial.Serial(
        port='/dev/rfcomm0',\
        baudrate=9600,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=0)
    print("connected to: " + ser.portstr)
    while True:
	
        for line in ser.read():
                if broken:
		    print ("Connection bluetooth reussie")
		    broken=False
        	pression = int(line)

                        
    ser.close()
thread2 = threading.Thread(target=communicationbluetooth,args=())
thread2.start()
   
def communicationandroid():
    import socket
    #Booleen pour envoi d'image
    global thickness
    global img
    global erase
    global color
    soc = socket.socket()         
    host = "192.168.2.2" 
    ############################             Verfier si l'adresse change au redemarrage           ###########################
    port = 2004                
    soc.bind((host, port))       
    soc.listen(5)          
           
    while True:
        conn, addr = soc.accept()     
        print ("Got connection from",addr)
        msg = conn.recv(1024)
        print msg
        if msg == "thin" :
            thickness = 5
	    erase=False
        if msg == "normal":
            thickness = 10
	    erase=False
        if msg == "thick":
            thickness = 15
	    erase=False
        if msg == "eraser":
            erase = True
        if "colour" in msg:
            colour = msg[6:]
            colorRGB,transp = HexToRGB(colour)
            color = (colorRGB[2],colorRGB[1],colorRGB[0],transp)

                         
        if msg == "sendimage":
            
            cv2.imwrite("image.jpg",img)
            imgfile = open("image.jpg", 'rb')
            obj = imgfile.read()
	    if conn.sendall(obj) == None:
	        imgfile.flush()
	        imgfile.close()
	        print ("New Image Sent")
            else:
	        print ("Error sending image")

    soc.close()
	
thread1 = threading.Thread(target=communicationandroid,args=())
thread1.start()

        


def ThreadCalibrage():
        font=cv2.FONT_HERSHEY_SIMPLEX
        global CompteurCalibrage
        global pression
        global cnts
	global homo
        def fonctioncalibrage(direction1,direction2):
                cv2.putText(img, "Cliquer sur le coin en "+ str(direction1) + " a " + str(direction2), (200,200), font, 0.8, (0,0,0), 2)
                time.sleep(2)
                while not pression or len(cnts)==0:
		    time.sleep(0.05)
                cv2.putText(img, "Cliquer sur le coin en " + str(direction1) + " a "  + str(direction2), (200,200), font, 0.8, (255,255,255), 2)
		time.sleep(1)
                c=max(cnts, key=cv2.contourArea)
                ((x,y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                C.append(center)



        if CompteurCalibrage == -1 :
            CompteurCalibrage+=1
	#Lancement	
	if CompteurCalibrage == 0 :
	    cv2.putText(img, "Debut du calibrage", (200,150), font, 0.8, (0,0,0), 2)
	    cv2.putText(img, "Appuyer sur l'ecran pour lancer le calibrage", (200,200), font, 0.8,(0,0,0), 2)
	    while not pression:
		time.sleep(0.05)
    	    cv2.putText(img, "Debut du calibrage", (200,150), font, 0.8, (255,255,255), 2)
	    cv2.putText(img, "Appuyer sur l'ecran pour lancer le calibrage", (200,200), font, 0.8, (255,255,255), 2)
	    CompteurCalibrage+=1
		
	#Haut Gauche
	if CompteurCalibrage ==1:
	        time.sleep(1)
	    	fonctioncalibrage("HAUT","GAUCHE")
	    	CompteurCalibrage+=1
	    	
	#Haut Droite
	if CompteurCalibrage == 2:
	        time.sleep(1)
	    	fonctioncalibrage("HAUT","DROITE")
	    	CompteurCalibrage+=1
	
	#Bas Droite		
	if CompteurCalibrage == 3:
	        time.sleep(1)
	    	fonctioncalibrage("BAS", "DROITE")
	    	CompteurCalibrage +=1
	
	#Bas Gauche	 
	if CompteurCalibrage == 4:
	        time.sleep(1)
	    	fonctioncalibrage("BAS","GAUCHE")
	    	CompteurCalibrage+=1
	
	
	if CompteurCalibrage == 5:
	    print('Calibrage fin')
            #Coins de l'image sur laquelle on dessine
	    points2=np.float32([(0,0),(width,0),(width,height),(0,height)])
	    #Coins pointes par l'utilisateur
	    points1=np.float32([C[0],C[1],C[2],C[3]])
	    #Calcule l'homographie
	    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
	    #L'utilise
	    #height, width, channels = image.shape
	    #Donne l'image recalee
	    #img = cv2.warpPerspective(img, h, (width, height)) 
	    

	    homo = h
	    time.sleep(1)
	    CompteurCalibrage +=1
		
	  

	 

            
                    
               
	    




			###    TRAME PRINCIPALE   ###


class WebcamVideoStream:
	def __init__(self, resolution=(640, 480), framerate=32):
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format="bgr", use_video_port=True)

		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame = None
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		t = Thread(target=self.update, args=())
		t.daemon = True
		t.start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frame = f.array
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return

	def read(self):
		# return the frame most recently read
		return self.frame

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True






vs = WebcamVideoStream().start()
L=[]
LC=[]


while vs.read() is None:
   print "Waiting for camera to warm up"
   time.sleep(.1)






while True :
	frame = vs.read()
	cv2.imshow("Frame", frame)
	cv2.namedWindow("img",cv2.WND_PROP_FULLSCREEN)
	cv2.setWindowProperty("img",cv2.WND_PROP_FULLSCREEN,cv2.cv.CV_WINDOW_FULLSCREEN)
        cv2.imshow("img",img)
	key = cv2.waitKey(1) & 0xFF
	if not pression :
            L = []
            LC=[]
             
		
                
				## Trouver les contours  ##
		
		        
	lower_white=np.array([0,0,230])
        upper_white=np.array([0,0,255])
        blurred  = cv2.GaussianBlur(frame,(1,1),0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,lower_white,upper_white)
        mask = cv2.dilate(mask, None, iterations=1)
        cntsexp = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cntsexp = imutils.grab_contours(cntsexp)
	if len(cntsexp)>0:
	    cnts=cntsexp
            contouHierar =[]
            for contour in cnts:
                    if 900>cv2.contourArea(contour) :
                        a=-1
	
        if CompteurCalibrage == -1:
            print "Calibrage lancement"
            thread4 = threading.Thread(target = ThreadCalibrage,args =())
            thread4.start()


	if len(cnts)>0 and CompteurCalibrage>5:
		contouHierar =[]
                for contour in cnts:
                    if cv2.contourArea(contour) < 900:
                        contouHierar.append(contour)
		if len(contouHierar)>0:
			c=max(contouHierar, key=cv2.contourArea)
			((x,y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]),1)
			centerC = np.dot(homo,center)
		        #Ici faudra appliquer l'homographie
	 		
	
		

		
		if pression :
                    L.append((center[0],center[1]))
                    
	            LC.append((int(centerC[0]),int(centerC[1])))  
                    if len(LC)>2:
                        LC.pop(0)


		    if len(LC)>1:
                        #print np.sqrt((LC[0][0]-LC[1][0])**2 + (LC[0][1] - LC[1][1])**2)
                        if np.sqrt((LC[0][0]-LC[1][0])**2 + (LC[0][1] - LC[1][1])**2)<50:
                                               
                            if not erase:
	                        cv2.line(img,LC[0],LC[1],color,thickness)
	                        LC.pop(0)
                            else:
                                cv2.line(img,LC[0],LC[1],(255,255,255),10)
                                LC.pop(0)
	    

cv2.destroyAllWindows()
vs.stop()
        






        
		
            
	



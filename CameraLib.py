import os
import numpy as np
from threading import Thread, Event
from Instrumentation import Inst

if os.name == 'nt': # Assume Windows/PC
    import cv2
else:               # Assume Linux/RPi
    from picamera2 import Picamera2
#if

BLUE   = (255,0,0)
GREEN  = (0,255,0)
RED    = (0,0,255)

class Camera():

    def __init__(self): 

        self.inst = Inst()
        self.picam = True
        self.camera = None
        self.imageHeight = 0
        self.imageWidth = 0
        self.imageChannels = 0
        self.streamThreadRunning = True
        self.framesRxed = 0
        self.rotate = False

        # Select the platform
        if(self.picam):
            self.camName = "Picam"
        else:
            self.camName = "OS camera"
        # if

        # Camera-specific connection
        if (not self.picam):
            self.inst.Print(Inst.CR,"Running video-capture connection")
            self.camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)
            #width = 1920
            #height = 1080
            width = 640
            height = 480
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.inst.Print(Inst.CR,"Video capture ready")
        else:
            self.inst.Print(Inst.CR,"Running Picam connection")
            self.camera = Picamera2()
            # resolution = (1640, 1232)
            resolution = (640, 480)
            config = self.camera.create_preview_configuration(main={"format": 'XRGB8888', "size": resolution})
            self.camera.configure(config)
            self.camera.start()
        # if
    # def

    def Read(self):
        if(self.picam):
            # Get Frame
            image = self.camera.capture_array()
            
            # Rotate
            if(self.rotate):
                return self.Rotate(image) 
            else:
                return image
            # if
        else:
            ret, image = self.camera.read()
        # if

        # Check for successful acquisition
        if(image is None):
            return None
        # if

        # Needed to make overlaying elements on the RGB frame work
        image = np.ascontiguousarray(image, dtype=np.uint8)
        return image
    # end def

    def InfoText(self):
        return self.camName
    # def

    def StatusText(self):
        # Create status message
        statusMsg = ""
        if(self.camera==None):
            statusMsg += "Not connected"
        else:
            statusMsg += "Connected"
        # if

        return statusMsg
    # def

    def PicamStreaming(self):
        self.inst.Print(Inst.CR,"Streamming")
        for frame in self.stream:
            self.frame = frame.array
            self.rawCapture.truncate(0)
            self.framesRxed += 1
            if(self.streamThreadRunning==False):
                self.inst.Print(Inst.CR,"Camera streaming exited")
                return
            else:
                sleep(0.05)
            #end if
        #end for
    # end def
    
    def Dimensions(self,image):
        height,width,channels = image.shape
        return int(height),int(width)
    # end def

    def Centre(self,image):
        height,width,channels = image.shape
        return int(height/2),int(width/2)
    # end def
    
    def Rotate(self,image):
        # Note cv2.ROTATE_90_CLOCKWISE and cv2.ROTATE_90_COUNTERCLOCKWISE available
        return cv2.rotate(image,cv2.ROTATE_180)
    # end def

# class

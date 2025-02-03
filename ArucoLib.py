import cv2
from cv2 import aruco
import numpy as np
import math

# Class for storing one AruCo marker's state
class Aruco():

    def __init__(self,corner,id,mmPerPixel,rotationVector, translationVector,sheetX,sheetY,camX,camY): 
        self.corner = corner
        self.id = id
        self.mmPerPixel = mmPerPixel
        self.rotationVector = rotationVector
        self.depth = translationVector[2]
        self.sheetX = sheetX
        self.sheetY = sheetY
        self.camX = camX
        self.camY = camY

        # Unpack the rotation vector
        R, _ = cv2.Rodrigues(rotationVector) # Convert rotation vector to a 3×3 rotation matrix
        yawRad = np.arctan2(R[1, 0], R[0, 0])  # Extract the yaw angle (rotation around the Z-axis)
        self.yaw = np.degrees(yawRad)
    # def

    def InfoText(self):
        return "[%d] Yaw[%.1f] Sheet[%.1f,%.1f] Cam[%.1f,%.1f]" % (self.id,
                                                                   self.yaw,
                                                                   self.sheetX,self.sheetY,
                                                                   self.camX,self.camY)
    # def

# class

class ArucoSheet():

    def __init__(self):

        # Set the scale factor relative to A0
        self.horizScaler = 12.0/50.0    # A4
        self.vertScaler  = 710.0/1472.0 # Measured from rig

        # Create a dictionary of locations (in mm)
        self.coords = {
            1:  (   0,    0, 0), 
            2:  (   0,  -40, 0), 
            3:  (   0,  -80, 0), 
            4:  (   0, -130, 0), 
            5:  (   0, -190, 0), 
            6:  (   0, -260, 0), 
            7:  (   0, -340, 0), 
            8:  ( 120, -340, 0), 
            9:  ( 120, -240, 0), 
            10: ( 120, -130, 0), 
            11: ( 120,    0, 0), 
            12: ( 120,  140, 0), 
            13: (-80,   320, 0), 
            15: (-265,  325, 0), 
            16: ( 100,  310, 0), 
            17: ( 450,    0, 0), 
            20: (-480,  300, 0), 
            25: ( 435, -255, 0), 
            30: ( 405,  245, 0), 
            50: (-320, -140, 0)
        }
    # def

    def Location(self,id):
        # Dictionary lookup
        try:
            x,y,z = self.coords.get(int(id), None)
            scaledLocation = (float(x) * self.horizScaler, float(y) * self.horizScaler, float(z) * self.vertScaler)
            return scaledLocation
        except:
            return None  # Return None for invalid input
        # try
    # def

# class

class ArucoDetector():

    def __init__(self): 
        # Load the camera calibration
        self.cameraMatrix = np.loadtxt('./matrix.txt', delimiter=',')
        self.cameraDistortion = np.loadtxt('./dist.txt', delimiter=',')

        # Load the sheet calibration
        self.arucoSheet = ArucoSheet()

        # Create the detector
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        # State of objects found
        self.arucos = []

        # Target location in pixels
        self.averageX = None
        self.averageY = None
        self.averageZ = None
        self.deviationXY = None

        # Target location in mm
        self.targetX = None
        self.targetY = None
        self.targetZ = None

    # def

    def InfoText(self):
        if((self.targetX is None) or (self.targetY is None) or (self.targetZ is None)):
            return "No Arucos"
        else:
            return "Arucos[%d] TargetMM[%.2f,%.2f,%.2f]" % (len(self.arucos),self.targetX,self.targetY,self.targetZ)
        # if
    # def

    def CreateAruco(self,corner,id):

            # Find the x,y location for this id
            location = self.arucoSheet.Location(id)
            # Check for invalid id
            if(location is None):
                return None
            else:
                sheetX,sheetY,sheetZ = location
            # if

            # Size is derived from id
            markerSize = id * self.arucoSheet.horizScaler * 10.0 # mm

            # Calculate image resolution
            x0,y0 = corner[0][0]
            x1,y1 = corner[0][1]
            side1 = math.hypot(x1-x0,y1-y0)
            x1,y1 = corner[0][1]
            x2,y2 = corner[0][2]
            side2 = math.hypot(x2-x1,y2-y1)
            sidePixels = (side1 + side2) / 2.0
            mmPerPixel =  markerSize / sidePixels

            # Define 3D object points of the marker corners (assuming the marker lies in the XY plane)
            halfSize = int(markerSize / 2)
            markerPoints = np.array([
                    [-halfSize, halfSize, 0],   # Top-left corner
                    [halfSize, halfSize, 0],    # Top-right corner
                    [halfSize, -halfSize, 0],  # Bottom-right corner
                    [-halfSize, -halfSize, 0]  # Bottom-left corner
            ], dtype=np.float32)

            # Estimate pose 
            success, rvec, tvec = cv2.solvePnP(markerPoints, corner[0], self.cameraMatrix, self.cameraDistortion)
            if(not success):
                return None
            # if

            # Project to target
            sheetLocation = np.array([-sheetX, -sheetY,  -sheetZ], dtype=np.float32).reshape(1, 3)
            targetLocation, _ = cv2.projectPoints(sheetLocation, rvec, tvec, self.cameraMatrix, self.cameraDistortion)
            camX,camY = tuple(map(int, targetLocation[0][0]))

            # Package into a single object
            arucoEntry = Aruco(corner,id,mmPerPixel,rvec,tvec,sheetX,sheetY,camX,camY)

            # Report
            return arucoEntry
    # def

    def ProcessArucos(self,imageCentreX,imageCentreY):

        # Average all arucos
        xTot,yTot,zTot = 0.0,0.0,0.0
        mmPerPixelTot = 0.0
        for loc in self.arucos:
            xTot += loc.camX
            yTot += loc.camY
            zTot += loc.depth
            mmPerPixelTot += loc.mmPerPixel
            #print("mmPerPixel=" + str(loc.mmPerPixel))
        # for
        self.averageX = int(xTot/len(self.arucos))
        self.averageY = int(yTot/len(self.arucos))
        self.averageZ = int(zTot/len(self.arucos))
        self.mmPerPixel = mmPerPixelTot / len(self.arucos)

        # Mean deviation from average
        devTot = 0.0
        for loc in self.arucos:
            dev = math.hypot(loc.camX - self.averageX,loc.camY - self.averageY)
            devTot += dev
        # for
        self.deviationXY = int(devTot/len(self.arucos))

        # Translate pixels to mm from centre
        self.targetX = ((self.averageX - imageCentreX) * self.mmPerPixel) / 1000.0 # metres
        self.targetY = ((self.averageY - imageCentreY) * self.mmPerPixel) / 1000.0 # metres

        # Translate depth to mm below
        self.targetZ = (self.averageZ * self.arucoSheet.vertScaler) / 1000.0 # metres
        
    # def

    def Read(self,frame):

        # Valid input check
        if(frame is None):
            return None
        # if

        # Find the ArUco marker
        yPixels,xPixels,_ = frame.shape
        grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(grey)

        # Check for no arucos found
        if((corners is None) or (ids is None)):
            corners = np.array([])
            ids = np.array([])
        # if

        # Create list of valid objects found
        self.arucos = []
        for corner,id in zip(corners,ids):

            # Process into a useable object
            arucoEntry = self.CreateAruco(corner,id)

            # Add to the list if valid
            if(not arucoEntry is None):
                # Decorate image with projected targets
                cv2.circle(frame, (arucoEntry.camX,arucoEntry.camY), 5, (0, 255, 255), -1)
                self.arucos.append(arucoEntry)
            # if

        # for

        # Measure image
        imageCentreX = int(frame.shape[1]/2)
        imageCentreY = int(frame.shape[0]/2)

        # Process all arucos into a merged location
        if( len(self.arucos) < 1):
            self.averageX = None
            self.averageY = None
            self.targetX = None
            self.targetY = None
            self.deviationXY = None
        else:
            self.ProcessArucos(imageCentreX,imageCentreY)
            cv2.circle(frame, (self.averageX,self.averageY), self.deviationXY, (0, 0, 255), -1)
        # if

        # Decorate image with recognised markers
        markedImg = aruco.drawDetectedMarkers(frame, corners, ids)

        # Decorate image with cross sight
        length = 30
        thickness = 2
        top = imageCentreY - length
        bot = imageCentreY + length
        left = imageCentreX - length
        right = imageCentreX + length
        colour = (0, 165, 255)
        cv2.line(markedImg, (imageCentreX, bot    ), (imageCentreX, top    ), colour, thickness) 
        cv2.line(markedImg, (right  , imageCentreY), (left   , imageCentreY), colour, thickness) 

        # Display 
        cv2.imshow('ArucoDetector', markedImg)

        # Report
        return self.arucos
    # def

    def TargetOffset(self):
        return self.targetX,self.targetY,self.targetZ
    # def

# class

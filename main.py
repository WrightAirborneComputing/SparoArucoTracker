import time, cv2
from Instrumentation import Inst
from CameraLib import Camera
from ArucoLib import ArucoDetector,Aruco
from MavlinkLib import Mavlink

_inst = Inst()
_camera = Camera()
_arucoDetector = ArucoDetector()
_mavlink = Mavlink()
#_mavlink.Connect("127.0.0.1",14552)
_mavlink.Connect("",4)

while(True):

    # Grab a camera frmae
    frame = _camera.Read()

    # Extract arucos from frame
    arucos = _arucoDetector.Read(frame)

    # Inst
    _inst.Print(Inst.CR,_arucoDetector.InfoText())

    # User display/exit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        # cap.release()
        cv2.destroyAllWindows()
        break
    # if

    # Extract target location
    targetX,targetY,targetZ,yaw = _arucoDetector.TargetPose()

    # Send to vehicle
    if(not targetZ is None):
        _mavlink.SendRangefinderMetres(targetZ)
    # if

    if(_mavlink.mavPort is None):
        _inst.Print(Inst.CR,"No vehicle connection")
    elif((not targetX is None) and (not targetY is None) and (not targetZ is None)):
        _mavlink.TargetedLand(targetX,targetY,targetZ,yaw,1)
    else:
        # If no markers detected send 0 values
        _mavlink.TargetedLand(0.0,0.0,0.0,0.0,0)
    # if

    # Scheduling
    time.sleep(0.1)

# while


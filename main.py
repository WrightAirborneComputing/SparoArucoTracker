import time, cv2
from Instrumentation import Inst
from CameraLib import Camera
from ArucoLib import ArucoDetector,Aruco
from MavlinkLib import Mavlink

_inst = Inst()
_camera = Camera()
_arucoDetector = ArucoDetector()
_mavlink = Mavlink(True)

while(True):

    # Grab a camera frmae
    frame = _camera.Read()

    # Extract arucos from frame
    arucos = _arucoDetector.Read(frame)

    # Inst
    for aruco in arucos:
        if((aruco.id==25) or (aruco.id==50)):
            _inst.Print(Inst.CR,aruco.InfoText())
        # if
    # for
    _inst.Print(Inst.CR,_arucoDetector.InfoText())

    # User display/exit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        # cap.release()
        cv2.destroyAllWindows()
        break
    # if

    # Extract target location
    targetX,targetY,targetZ = _arucoDetector.Target()

    if((not targetX is None) and (not targetY is None)):
        _mavlink.SendRangfinderMetres(targetZ)
        _mavlink.TargetedLand(targetX,targetY,targetZ,
                                   dist_m=targetaverage_z_m, quart=quaternion, valid=1)
    # If no markers detected send 0 values and set "Valid" to 0 (tells ardupilot ekf)
    else:
        _mavlink.TargetedLand(x_m=0, y_m=0, z_m=0,
                                   dist_m=0, quart=[1, 0, 0, 0], valid=0)
    # if

    # Scheduling
    time.sleep(0.5)

# while


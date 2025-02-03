import os
from pymavlink import mavutil
from threading import Thread, Event
from time import sleep, time
import math
import struct
from Instrumentation import Inst

class Mavlink(object):

    # Constructor
    def __init__(self):

        # Instrumentation
        self.inst = Inst()

        # Initialise settings
        self.portName = None
        self.mavPort = None
        self.dataTimeout = 1
        self.packetsRxed = 0
        self.lastReceivedTime = time()
        self.receiveTimeout = 1.1 # Able to detect heartbeat
        self.packetRxed = False 
        self.lastAckedCmd = None
        self.lastAckedResult = None
        self.packetData = None
        self.fileChunkSize = 4000

        self.packetError = False
        self.packetCount = 0
        self.runStart = time()
        self.byteCount = 0
        self.byteCountStart = time()

        # Groundstation commands
        self.longCommandMsg = None

        # Cached stick inputs
        self.channel1 = 1500
        self.channel2 = 1500
        self.channel3 = 1500
        self.channel4 = 1500
        self.channel5 = 1500
        self.channel6 = 1500
        self.channel7 = 1500
        self.channel8 = 1500
        self.channel9 = 1500

        # Cached airdata for display
        self.heartbeat = None
        self.isArmed   = 0
        self.pitch   = 0.0
        self.roll     = 0.0
        self.yaw       = 0.0
        self.voltage   = 0.0
        self.current   = 0.0
        self.fix       = 0
        self.lat       = 0.0
        self.lon       = 0.0
        self.gpsAlt = -10.0
        self.sats     = 0
        self.speed   = 0
        self.altAsl = 0.0
        self.altAgl = 0.0
        self.heading   = -999.0
        self.lidarStat = -999.0
        self.lidarAlt  = -999.0
        self.rawRc1 = -999
        self.rawRc2 = -999
        self.rawRc3 = -999
        self.rawRc4 = -999
        self.rawRc5 = -999
        self.rawRc6 = -999
        self.rawRc7 = -999
        self.rawRc8 = -999

    # def

    def Connect(self, ipAddress, portNumber):

        # Construct port name from number
        if((portNumber>5000) and (portNumber<=5770)):
            address = "tcp:" + ipAddress
            self.portName  = address + ":" + str(portNumber)
            self.baudRate = 115200
        elif((portNumber>14540) and (portNumber<14590)):
            address = "udp:" + ipAddress
            self.portName  = address + ":" + str(portNumber)
            self.baudRate = 115200
        elif os.name == 'nt':
            self.inst.Print(Inst.CR,"Running serial connection on Windows")

            if(portNumber<100):
                self.portName  = "COM" + str(portNumber)
                self.baudRate = 115200
            #end if

        # Assumed to be running on Linux 
        else:
            self.inst.Print(Inst.CR,"Running serial connection on Linux")

            # Port 1 = GUI drone link
            if(portNumber==1):
                self.portName = "/dev/ttyAMA0"
                self.baudRate = 57600
            # Port 2 = Drone ground link
            elif(portNumber==2):
                self.portName = "/dev/ttyUSB0"
                self.baudRate = 57600
            # Port 3 = Drone FCS link
            elif(portNumber==3):
                self.portName = "/dev/serial0"
                self.baudRate = 500000
           # Port 4 = Drone FCS link
            elif(portNumber==4):
                self.portName = "/dev/ttyS0"
                self.baudRate = 57600
            #end if
        #end if

        # Clear down any old port
        self.mavPort = None

        # Start receiver monitor
        self.inst.Print(Inst.CR,"Starting Mavlink receiver monitor")
        self._closeMonitorThread = Event()
        self._monitorThread = Thread(target=self.ProcessMessages)
        self._monitorThread.setDaemon(True)
        self._monitorThread.start()

    # Connect

    def Disconnect(self):
        """Disconnect from a Mavlink RC device"""
        self._closeMonitorThread.set()
        self._monitorThread.join()
        self.mavPort.close()
    # def

    # command processing methods 

    def RadsToDegrees(self,value):
        return round(value * 180 / math.pi,1)
    # def

    def ProcessMessages(self):
        global localpositionnedcoords
        global globalpositionintcoords
        global yawcoords

        # Run continuously
        while(not self._closeMonitorThread.isSet()):

            # Check if connect needed
            if(self.mavPort==None): 
                self.inst.Print(Inst.CR,"Connecting to " + self.portName)
                try:

                    # Connect to link 
                    self.mavPort  = mavutil.mavlink_connection(self.portName, self.baudRate)
                    self.inst.Print(Inst.CR,"Connected to " + self.portName)

                    # Wait for the heartbeat msg to find the system ID
                    self.inst.Print(Inst.CR,"Waiting for heartbeat")
                    self.mavPort.wait_heartbeat()
                    self.inst.Print(Inst.CR,"Heartbeat System=" + str(self.mavPort.target_system) + " Component=" + str(self.mavPort.target_component) )

                    # Request all available UAV data
                    self.RequestData()

                except Exception as e:
                    self.inst.Print(Inst.CR,"Connect failed: " +str(e))
            #end if

            # Wait for a message
            try:
                msg = self.mavPort.recv_match(blocking=True)
            except Exception as e:
                self.inst.Print(Inst.CR,"Receive failed: " +str(e))
                sleep(0.5)
                if self.mavPort is not None:
                    self.mavPort.close()
                    self.mavPort = None
                msg = None

            # Check if a message has arrived
            if not msg:
                self.inst.Print(Inst.CR ,"Nothing heard")
                continue
            #end if

            # Instrumentation
            if msg.get_type() != "BAD_DATA":
                self.inst.Print(Inst.OFF ,"MessageType=" + msg.get_type())
            # if

            self.packetError = False
            self.packetCount+=1
            self.byteCount+=len(msg.get_msgbuf())

            # Log receipt time for timeout checking
            self.lastReceivedTime = time()

            # Decode message
            instMode = Inst.OFF
            if msg.get_type() == "BAD_DATA":
                self.inst.Print(instMode,"Bad=" + str(msg))
                #if mavutil.all_printable(msg.data):
                #   self.inst.Print(Inst.CR,"Bad message data=" + str(msg.data))
                #end if
            elif msg.get_type() == "HEARTBEAT":

                # Filter non-drone heartbeats
                if((msg.type==mavutil.mavlink.MAV_TYPE_GCS) or 
                   (msg.type==mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER)):
                    pass
                else:
                    self.inst.Print(instMode,"A")
                    # Store heartbeat for decode if needed
                    self.heartbeat = msg
                    # Do immediate armed decode
                    self.isArmed = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                 # if

            elif msg.get_type() == "COMMAND_ACK":
                self.lastAckedCmd = msg.command
                self.lastAckedResult = msg.result
                self.inst.Print(Inst.OFF,"COMMAND_ACK=" + str(msg))
            elif msg.get_type() == "STATUSTEXT":
                self.inst.Print(instMode,"B")
                self.inst.Print(Inst.CR,"[" + msg.text + "]")
            elif msg.get_type() == "ATTITUDE":
                yawcoords = msg.yaw
                self.inst.Print(instMode,"C")
                self.pitch = self.RadsToDegrees(msg.pitch)
                self.roll  = self.RadsToDegrees(msg.roll)
                self.yaw   = self.RadsToDegrees(msg.yaw)
            elif msg.get_type() == "SYS_STATUS":
                self.inst.Print(instMode,"D")
                self.voltage = round(msg.voltage_battery / 1000.0,1) # Convert to Volts
                self.current = round(msg.current_battery / 100.0, 2) # Convert to Amps
            elif msg.get_type() == "DISTANCE_SENSOR":
                self.inst.Print(instMode,"E")
                self.lidarStat  = msg.covariance
                self.lidarAlt   = round(msg.current_distance / 100.0,2) # Convert cm to m
                self.inst.Print(instMode,"Distance=" + str(self.lidarStat) + " " + str(self.lidarAlt) )
            elif msg.get_type() == "COMMAND_LONG":
                self.inst.Print(instMode,"H")
                self.longCommandMsg = msg
            # GPS
            elif msg.get_type() == "GPS_RAW_INT":
                self.inst.Print(instMode,"F")
                self.sats = msg.satellites_visible
                self.fix = msg.fix_type
                self.heading = msg.cog
            elif msg.get_type() == "LOCAL_POSITION_NED":
                #print(msg.x, msg.y, msg.z)
                localpositionnedcoords = [msg.x, msg.y, msg.z]
            elif msg.get_type() == "GLOBAL_POSITION_INT":

                self.inst.Print(instMode,"G")
                self.lat = (msg.lat / 10000000.0) # Convert to degrees
                self.lon = (msg.lon / 10000000.0) # Convert to degrees
                globalpositionintcoords = [self.lat, self.lon]
                self.altAsl = round(msg.alt / 1000.0,                      2) # Convert mm to m
                self.altAgl = round(msg.relative_alt / 1000.0,             2) # Convert mm to m
                self.speed  = round(math.sqrt((msg.vx ** 2) + (msg.vy ** 2)),1) # m/s
            elif msg.get_type() == "RC_CHANNELS_RAW":
                self.inst.Print(Inst.CR,"RC_CHANNELS_RAW C1[" + str(msg.chan1_raw) + "] C2[" + str(msg.chan2_raw) + "] C3[" + str(msg.chan3_raw) + "] C4[" + str(msg.chan4_raw) + "]")
                self.rawRc1 = msg.chan1_raw
                self.rawRc2 = msg.chan2_raw
                self.rawRc3 = msg.chan3_raw
                self.rawRc4 = msg.chan4_raw
                self.rawRc5 = msg.chan5_raw
                self.rawRc6 = msg.chan6_raw
                self.rawRc7 = msg.chan7_raw
                self.rawRc8 = msg.chan8_raw
            elif msg.get_type() == "MOUNT_ORIENTATION":
                self.inst.Print(Inst.OFF,"MOUNT_ORIENTATION [" + str(msg.pitch) + "] [" + str(msg.roll) + "] [" + str(msg.yaw) + "]")
            elif msg.get_type() == "MOUNT_STATUS":
                self.inst.Print(Inst.OFF,"MOUNT_STATUS")
            #end if

    # def

    def GetStatus(self):
        if(time() - self.lastReceivedTime > self.receiveTimeout):
            return False
        else:
            return True
        # if
    # def

    def DataRate(self):

        # Calculate byte rate based on statistics since last call
        countSecs = time() - self.byteCountStart
        byteRate = 0
        packetRate = 0
        if(countSecs>0):
            byteRate = self.byteCount / countSecs
            packetRate = self.packetCount / countSecs
        # if

        # Reset count
        self.packetCount = 0
        self.byteCount = 0
        self.byteCountStart = time()

        # Report result
        return int(packetRate),int(byteRate)
    # def

    def Attitude(self):
        return (self.roll,self.pitch,self.yaw)
    # def

    def DecodeModeFlags(self):

        # Wait for a fresh heartbeat
        #if(self.WaitForHeartbeat()==False):
        #    return "Unknown"
        if(self.heartbeat==None):
            return "Unknown"

        # Extract mode from message
        isArmed      = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        isManual     = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        isHil        = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED
        isStabilized = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
        isGuided     = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        isAuto       = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED
        isTest       = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED
        isCustom     = self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

        # Generate text message
        statusText = ""
        if(isArmed):
            statusText += "Armed."
        else:
            statusText += "Disarmed."

        if(isManual):
            statusText += "Manual."
        if(isHil):
            statusText += "HIL."
        if(isStabilized):
            statusText += "Stabilized."
        if(isGuided):
            statusText += "Guided."
        if(isAuto):
            statusText += "Auto."
        if(isTest):
            statusText += "Test."
        if(isCustom):
            statusText += "Custom."

        if(self.heartbeat.custom_mode==0):
            statusText += "Stabilized"
        elif(self.heartbeat.custom_mode==1):
            statusText += "Acro"
        elif(self.heartbeat.custom_mode==2):
            statusText += "AltHold"
        elif(self.heartbeat.custom_mode==3):
            statusText += "Auto"
        elif(self.heartbeat.custom_mode==4):
            statusText += "Guided"
        elif(self.heartbeat.custom_mode==5):
            statusText += "Loiter"
        elif(self.heartbeat.custom_mode==6):
            statusText += "RTL"
        elif(self.heartbeat.custom_mode==7):
            statusText += "Circle"
        elif(self.heartbeat.custom_mode==9):
            statusText += "Land"
        elif(self.heartbeat.custom_mode==15):
            statusText += "AutoTune"
        elif(self.heartbeat.custom_mode==16):
            statusText += "PosHold"
        elif(self.heartbeat.custom_mode==18):
            statusText += "Throw"
        elif(self.heartbeat.custom_mode==22):
            statusText += "FlowHold"
        else:
            statusText += str(self.heartbeat.custom_mode)
        # if


        # LiDAR text
        lidarText = str(self.lidarStat) + " " + str(self.lidarAlt)

        # Battery text
        batteryText = str(self.voltage) + "V"

        #return(mavutil.mode_string_v10(self.heartbeat)) ####################
        # return statusText + " " + lidarText + " " + batteryText
        return statusText

    # def

    def WaitForAck(self,cmd):

        # Initialise timeout
        startTime = time()
        timeoutSecs = 1.0

        # Wait for command_ack for this command
        while((self.lastAckedCmd == None) or not(self.lastAckedCmd == cmd)):

            # Check for timeout
            if(time() - startTime > timeoutSecs):
                self.inst.Print(Inst.CR,"Timeout on ack for [" + str(cmd) + "]")
                return False

            sleep(0.01)

        # while

        # Decode
        resultText = str(self.lastAckedResult)
        if(self.lastAckedResult==0):
            resultText = "Accepted"
        elif(self.lastAckedResult==1):
            resultText = "TempRejection"
        elif(self.lastAckedResult==2):
            resultText = "Denied"
        elif(self.lastAckedResult==3):
            resultText = "Unsupported"
        elif(self.lastAckedResult==4):
            resultText = "Failed"
        elif(self.lastAckedResult==5):
            resultText = "InProgress"
        elif(self.lastAckedResult==6):
            resultText = "Cancelled"
        # if

        # Instrumentation
        ackTime = time() - startTime
        self.inst.Print(Inst.CR,"Got ack Cmd[" + str(self.lastAckedCmd) + "] Result[" + resultText + "] Latency[" + str(int(ackTime*1000)) + "]ms")

        # Clear down to wait for next ack
        self.lastAckedCmd = None
        return True
    # def

    def WaitForHeartbeat(self):

        # Initialise timeout
        startTime = time()
        timeoutSecs = 2.0

        # Wait for a new heartbeat
        self.heartbeat = None
        while(self.heartbeat == None):

            # Check for timeout
            if(time() - startTime > timeoutSecs):
                self.inst.Print(Inst.CR,"Timeout on heartbeat")
                return False

            sleep(0.01)

        # while

        return True
    # def

    def IsArmed(self):
        return self.isArmed
    # def
    
    def ArmDisarm(self,arm,override):

        # Ensure data is on
        self.RequestData()

        if(override):
            overrideCode = 21196
        else:
            overrideCode = 0
        # if
        self.inst.Print(Inst.CR,"ArmDisarm[" + str(arm) + "] Override[" + str(override) + "]")
        # end if

        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,            # Confirmation
            arm,          # 1=arm 0=disarm
            overrideCode, # 0=On-ground 21196=Emergency
            0,            # Not Used
            0,            # Not Used
            0,            # Not Used
            0,            # Not Used
            0)            # Not Used

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    # def
    
    def TakeOff(self, altitude):
        # First send take off comand
        self.inst.Print(Inst.CR,"Takeoff to [" + str(altitude) + "]")
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
            1,          # confirmation
            0,          # Pitch
            0,          # Unused
            0,          # Unused
            0,          # Yaw
            0,          # Latitude
            0,          # Longitude
            altitude)   # Altitude

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    # def
    
    def MissionStart(self):
        self.inst.Print(Inst.CR,"Start mission")
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_MISSION_START)
    # def
    
    def CmdYaw(self, angle):

        angleMode = 1
        if(angle>0.0):
            dir = 1
        else:
            dir = -1
            angle *= -1
        # if

        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
            0, # confirmation
            angle, # param1 (angle value)
            0, # param2 (angular speed value) isn't used!
            dir, # param3 (direction)
            angleMode, # param4 (mode: 0->absolute / 1->relative)
            0, # param5
            0, # param6
            0) # param7
    # def

    def ToQuaternion(self,roll = 0.0, pitch = 0.0, yaw = 0.0):
        # Convert degrees to quaternions
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return [w, x, y, z]
    # def

    def Land(self):
        self.inst.Print(Inst.CR,"Land")
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,  # command
            0,  # confirmation
            0,  # param1 abort altitude, not used
            0,  # param2 land mode, not used
            0,  # param3
            0,  # param4 desired yaw angle NaN for no change
            0,  # param5 Lat
            0,  # param6 Long
            -0.2)  # Landing altitude (Current Frame)

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_NAV_LAND)

    # def
    
    def TargetedLand(self,x,y,z):

        # Derive descent angle
        xRads = math.atan(z/x)
        yRads = math.atan(z/y)

        self.mavPort.mav.landing_target_send(
            int(time() * 1e6),  # Timestamp
            0,  # Target number
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame of reference
            xRads,   # X-offset in radians (centered target = 0)
            yRads,   # Y-offset in radians (centered target = 0)
            z,   # Distance to target (meters)
            0.5,   # Target size (meters)
            0      # Target type (0 = Fiducial marker, 1 = beacon)
            )

        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0
            )

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_NAV_LAND)
    # def

    # Dummy Rangefinder
    def SendRangefinderMetres(self,metres):

        # Dummy voltage
        voltage = 5.0

        # MAVLink message: RANGEFINDER
        self.mavPort.mav.rangefinder_send(
            metres,  # Distance in meters
            voltage  # Voltage in volts
        )
    # def

    def CmdAttitude(self,yawRate):
        self.mavPort.mav.set_attitude_target_send(
            0, # time_boot_ms
            self.mavPort.target_system,
            self.mavPort.target_component,
            0b00000000,
            self.ToQuaternion(0.0, 0.0, 0.0), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yawRate), # Body yaw rate in radian/second
            0.5  # Thrust
        )
    # def

    def MoveVelocity(self, velocity_x, velocity_y, velocity_z):
        self.mavPort.mav.set_position_target_local_ned_send(
            0, # System Time
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame or jut the number 8, mavutil.mavlink.MAV_FRAME_BODY_NED
            0b0011111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, -velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)   # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_FRAME_BODY_NED)
    # def

    def CmdVelocityYawRate(self, fwdVel, vertVel, yawRate):
        self.mavPort.mav.set_position_target_local_ned_send(
                                  0,  # system time in milliseconds
                                  self.mavPort.target_system,
                                  self.mavPort.target_component,
                                  mavutil.mavlink.MAV_FRAME_BODY_NED,# coordinate frame
                                  0B011111000111,                    # type mask
                                  0, 0, 0,                           # position x,y,z
                                  fwdVel, 0.0, -vertVel,             # velocity x,y,z (convert to FRD)
                                  0.0, 0.0, 0.0,                     # accel x,y,z
                                  0, math.radians(yawRate))          # yaw, yaw rate
    # def

    def CmdPosition(self, x_m, y_m, z_m):
        self.inst.Print(Inst.CR,"CmdPosition[x:%.1f, y:%.1f, z:%.1f]" % (x_m, y_m, z_m))
        self.mavPort.mav.set_position_target_local_ned_send(
                                  0,  # system time in milliseconds
                                  self.mavPort.target_system,
                                  self.mavPort.target_component,
                                  mavutil.mavlink.MAV_FRAME_BODY_NED,  # coordinate frame
                                  0B010111111000,     # type mask: 0B010111111000=>no turning, 0B110111111000=>turning
                                  x_m, y_m, z_m,  # position x,y,z
                                  0, 0, 0,  # velocity x,y,z
                                  0, 0, 0,  # accel x,y,z
                                  0, 0)     # yaw, yaw rate
    # def

    def SetFlightMode(self, basemode, submode):
        self.inst.Print(Inst.CR,"Setting flight mode Base[" + str(basemode) + "] Sub[" + str(submode) + "]")
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command
            0,  # confirmation
            basemode, 
            submode,  
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0)  # param7

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_DO_SET_MODE)

    # def

    def RcChannelSet(self, channel1, channel2, channel3, channel4, channel7, channel8, channel9):
        self.channel1 = channel1
        self.channel2 = channel2
        self.channel3 = channel3
        self.channel4 = channel4
        self.channel5 = 1500
        self.channel6 = 1500
        self.channel7 = channel7
        self.channel8 = channel8
        self.channel9 = channel9
    # def
    
    def RcChannelOverride(self):
        if(self.mavPort==None): return

        channels = [self.channel1,self.channel2,self.channel3,self.channel4,self.channel5,self.channel6,self.channel7,self.channel8,self.channel9]
        # print("Channels=" + str(channels))
        self.mavPort.mav.rc_channels_override_send(
            self.mavPort.target_system,
            self.mavPort.target_component,# Message
            *channels)
    # def
    
    def SetDefaultGlobalOrigin(self,latitude,longitude,altitude):  ## Must be run at each power cycle to give local coordinates

        self.inst.Print(Inst.CR,"SetDefaultGlobalOrigin Lat[" + str(latitude) + "] Long[" + str(longitude) + "] Alt[" + str(altitude) + "]")
        latitudeInt = int(latitude * 1e7)
        longitudeInt = int(longitude * 1e7)
        altitudeInt = int(altitude * 1e2)
        self.mavPort.mav.set_gps_global_origin_send(
            self.mavPort.target_system,
            latitudeInt,
            longitudeInt,
            altitudeInt)

        # Wait completion - no explicit ack
        sleep(0.1)

    # def

    def RequestData(self):
        if(self.mavPort==None):
            self.inst.Print(Inst.CR,"Warning. No drone connection")
            return
        # if

        # Request all data to be sent 
        requestedRate = 5
        requestedInterval = 1000000 / requestedRate
        start = 1
        stop = 0
        self.mavPort.mav.request_data_stream_send(self.mavPort.target_system, self.mavPort.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL, requestedRate, start)
        #self.mavPort.mav.request_data_stream_send(self.mavPort.target_system, self.mavPort.target_component,mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, requestedRate, start)
        return
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            35, # RC_CHANNELS_RAW,
            1,
            0,
            0,
            0,
            0,
            0,
            0)  # command
    # def
        
    def RequestRate(self):
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            1, # confirmation
            32, # mavutil.mavlink.LOCAL_POSTION_NED,
            0,
            0,
            0,
            0,
            0,
            0)  # command

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)

    # def

    def RequestReboot(self): 
        self.mavPort.mav.command_long_send(
            self.mavPort.target_system,
            self.mavPort.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            1, # confirmation
            1, # 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
            0,
            0,
            0,
            0,
            0,
            0)  # command

        # Wait for ack
        self.WaitForAck(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)

    # def

    def Refresh(self,refreshPeriod):
        pass # Nothing to do - stub to match simulation support
    # def

# class


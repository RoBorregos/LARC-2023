#!/usr/bin/env python3
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger, TriggerResponse
from main_engine.msg import lineSensor
import os, time
import _thread

import math
from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

import roslib

from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int32, UInt16, Float32, String, Bool
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Range, Imu

from tf.transformations import quaternion_from_euler

import struct
import binascii
 
ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


class Microcontroller:
    ''' Configuration Parameters
    '''
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        self.WAITING_FF = 0
        self.WAITING_AA = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PACKAGE = 3
        self.RECEIVE_CHECK = 4
        self.HEADER0 = 0xff
        self.HEADER1 = 0xaa
        
        self.SUCCESS = 0
        self.FAIL = -1

        self.receive_state_ = self.WAITING_FF
        self.receive_check_sum_ = 0
        self.payload_command = b''
        self.payload_ack = b''
        self.payload_args = b''
        self.payload_len = 0
        self.byte_count_ = 0
        self.receive_message_length_ = 0
    
        # Keep things thread safe
        self.mutex = _thread.allocate_lock()

    def connect(self):
        try:
            print("Connecting to Microcontroller on port", self.port, "...")
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            state_, val = self.get_baud()
            if val != self.baudrate:
                time.sleep(1)
                state_, val  = self.get_baud()
                if val != self.baudrate:
                    raise SerialException
            print("Connected at", self.baudrate)
            print("Microcontroller is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Microcontroller!")
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd)


    def receiveFiniteStates(self, rx_data):
        if self.receive_state_ == self.WAITING_FF:
            #print str(binascii.b2a_hex(rx_data))
            if rx_data == b'\xff':
                self.receive_state_ = self.WAITING_AA
                self.receive_check_sum_ =0
                self.receive_message_length_ = 0
                self.byte_count_=0
                self.payload_ack = b''
                self.payload_args = b''
                self.payload_len = 0


        elif self.receive_state_ == self.WAITING_AA :
             if rx_data == b'\xaa':
                 self.receive_state_ = self.RECEIVE_LEN
                 self.receive_check_sum_ = 0
             else:
                 self.receive_state_ = self.WAITING_FF

        elif self.receive_state_ == self.RECEIVE_LEN:
             self.receive_message_length_, = struct.unpack("B",rx_data)
             self.receive_state_ = self.RECEIVE_PACKAGE
             self.receive_check_sum_ = self.receive_message_length_
        elif self.receive_state_ == self.RECEIVE_PACKAGE:
             if self.byte_count_==0:
                 self.payload_ack = rx_data
             else:
                 self.payload_args += rx_data
             uc_tmp_, = struct.unpack("B",rx_data)
             self.receive_check_sum_ = self.receive_check_sum_ + uc_tmp_
             self.byte_count_ +=1
             if self.byte_count_ >= self.receive_message_length_:
                 self.receive_state_ = self.RECEIVE_CHECK

        elif self.receive_state_ == self.RECEIVE_CHECK:
            self.receive_state_ = self.WAITING_FF
            return 1 
        else:
            self.receive_state_ = self.WAITING_FF
        return 0

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Microcontroller
        '''
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        while self.receiveFiniteStates(c) != 1:
            c = self.port.read(1)
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return 0
        return 1
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Microcontroller returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd)
            res = self.recv(self.timeout)
            while attempts < ntries and res !=1 :
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    res = self.recv(self.timeout)
                    #print "response : " + str(binascii.b2a_hex(res))
                except:
                    print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
                attempts += 1
        except:
            self.mutex.release()
            print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
            return 0
        
        self.mutex.release()
        return 1
                                 

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x00) + struct.pack("B", 0x01)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, val 
        else:
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, 0

    def get_odometry(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           velX, velY, velTh, posX, posY = struct.unpack('5f', self.payload_args)
           return  self.SUCCESS, velX, velY, velTh, posX, posY
        else:
           return self.FAIL, 0, 0

    def imu_angle(self, angle):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x05) + struct.pack("f", angle) + struct.pack("B", 0x06)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_emergency_button(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           emergency_state, = struct.unpack('B', self.payload_args)
           return  self.SUCCESS, emergency_state
        else:
           return self.FAIL, 0

    def reset_odometry(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x03) + struct.pack("B", 0x04)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_check_sum(self,list):
        list_len = len(list)
        cs = 0
        for i in range(list_len):
            cs += list[i]
        cs=cs%255
        return cs

    def drive(self, x, y, th):
        # data1 = struct.pack("h", x)
        # data2 = struct.pack("h", th)
        # d1, d2 = struct.unpack("BB", data1)
        # c1, c2 = struct.unpack("BB", data2)
        # self.check_list = [0x05,0x04, d1, d2, c1, c2]
        # self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x0D, 0x04) + struct.pack("fff", x, y, th) + struct.pack("B", 0x05)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
    
    def intake(self, command):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x06) + struct.pack("i", command) + struct.pack("B", 0x07)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
        
    def elevator(self, command):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x08) + struct.pack("i", command) + struct.pack("B", 0x09)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
    
    def line_sensors(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0B) + struct.pack("B", 0x0C)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           fl1, fl2, fr1, fr2, bl1, bl2, br1, br2 = struct.unpack('8c', self.payload_args)
           return  self.SUCCESS, fl1, fl2, fr1, fr2, bl1, bl2, br1, br2
        else:
            return self.FAIL, 0, 0, 0, 0, 0, 0, 0, 0
        
    def warehouse(self, level):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x0A) + struct.pack("i", level) + struct.pack("B", 0x0B)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def set_global_setpoint(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0D) + struct.pack("B", 0x0E)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
            return self.SUCCESS
        else:
            return self.FAIL
        
    def stop(self):
        ''' Stop both motors.
        '''
        return self.drive(0, 0, 0)

    def get_hardware_version(self):
        ''' Get the current version of the hardware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x13) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, val
        else:
           return self.FAIL, -1, -1


""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, Microcontroller, base_frame):
        self.Microcontroller = Microcontroller
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
        self.useImu = rospy.get_param("~useImu", True)
        
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.1518)
        self.wheel_track = rospy.get_param("~wheel_track", 0.375)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 42760)
        self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
       
        self.start_rotation_limit_w = rospy.get_param("~start_rotation_limit_w", 0.4) 
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        self.encoder_min = rospy.get_param('encoder_min', 0)
        self.encoder_max = rospy.get_param('encoder_max', 65535)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_x = 0                    # cmd_vel setpoint
        self.v_y = 0
        self.v_th = 0
        self.last_cmd_vel = now

        self.angle = 0
        self.quaternion = Quaternion()

        self.intake_command = 0
        self.elevator_command = 0
        self.warehouse = 0 
        self.line_sensor = 0

        self.global_setpoint = 0
        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        rospy.Subscriber("intake", Int32, self.intakeCallback)
        rospy.Subscriber("elevator", Int32, self.elevatorCallback)
        rospy.Subscriber("warehouse", Int32, self.warehouseCallback)
        rospy.Subscriber("global_setpoint", Bool, self.globalSetpointCallback)
        self.line_sensor_pub = rospy.Publisher("line_sensors", lineSensor, queue_size=5)
        
        # Clear any old odometry info
        #self.Microcontroller.reset_encoders()
        #self.Microcontroller.reset_IMU()

        self.imu_frame_id = rospy.get_param('imu_frame_id', 'imu_base')
        self.imu_offset = rospy.get_param('imu_offset', 1.01)
        rospy.Subscriber("imu_rpy", Vector3, self.imuRPYCallback)
        rospy.Subscriber("imu_data", Imu, self.imuDataCallback)
        #self.imuPub = rospy.Publisher('imu', Imu, queue_size=5)
        #self.imuAnglePub = rospy.Publisher('imu_angle', Float32, queue_size=5)
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        rospy.Subscriber("reset_odom", Bool, self.resetOdomCallback)
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

        self.SUCCESS = 0
        self.FAIL = -1
   
        self.micro_version=0
        _,version=self.Microcontroller.get_hardware_version()
        self.slam_project_version = rospy.get_param("~slam_project_version",0)
        rospy.loginfo ("*************************************************")
        rospy.loginfo ("micro hardware_version is "+str(version)+str(".")+str(version))
        rospy.loginfo ("micro software_version is "+str(version)+str(".")+str(version))
        rospy.loginfo ("slam version is "+str(self.slam_project_version))
        rospy.loginfo ("*************************************************")

    def resetEncoderCallback(self, req):
        if req.data==1:
            try:
                res = self.Microcontroller.reset_encoders()
                if res==self.FAIL:
                    rospy.logerr("reset encoder failed ")
            except:
                rospy.logerr("request to reset encoder exception ")

    def resetImuCallback(self, req):
        if req.data==1:
            try:
                res = self.Microcontroller.reset_imu()
                if res==self.FAIL:
                    rospy.logerr("reset imu failed ")
            except:
                rospy.logerr("request to reset imu exception ")

    def globalSetpointCallback(self, req):
        if req.data:
            self.Microcontroller.set_global_setpoint()

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:

            dt = now - self.then
            self.then = now
            dt = dt.to_sec()

            vx = 0.0
            vy = 0
            vth = 0
            # Get odometry from the microcontroller
            try:
                ack, vx, vy, vth, self.x, self.y  = self.Microcontroller.get_odometry()
                if ack==self.FAIL:
                    rospy.logerr("get odometry failed ")
                    return
            except:
                rospy.logerr("get odometry exception ")
                return
            
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = self.quaternion
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth

            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)

            """
            fl1 = '0'
            fl2 = '0'
            fr1 = '0'
            fr2 = '0'
            bl1 = '0'
            bl2 = '0'
            br1 = '0'
            br2 = '0'
            try:
                ack, fl1, fl2, fr1, fr2, bl1, bl2, br1, br2 = self.Microcontroller.get_line_sensors()
                if ack==self.FAIL:
                    rospy.logerr("get line sensors failed ")
                    return
            except:
                rospy.logerr("get line sensors exception ")
                return

            line_data = lineSensor()
            line_data.frontLeft1 = fl1
            line_data.frontLeft2 = fl2
            line_data.frontRight1 = fr1
            line_data.frontRight2 = fr2
            line_data.backLeft1 = bl1
            line_data.backLeft2 = bl2
            line_data.backRight1 = br1
            line_data.backRight2 = br2
            self.line_sensor_pub.publish(line_data)
            """
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_x = 0
                self.v_y = 0
                self.v_th = 0

            # Set motor speeds in encoder ticks per PID loop
            if ((not self.stopped)):
                self.Microcontroller.drive(self.v_x, self.v_y, self.v_th)
                self.Microcontroller.imu_angle(self.angle)
                
            if(self.intake_command != 0):
                self.Microcontroller.intake(self.intake_command)
                self.intake_command = 0

            if(self.elevator_command != 0):
                self.Microcontroller.elevator(self.elevator_command)
                self.elevator_command = 0

            if(self.warehouse != 0):
                self.Microcontroller.warehouse(self.warehouse)
                self.warehouse = 0

                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.Microcontroller.drive(0, 0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        y = req.linear.y         # m/s
        th = req.angular.z       # rad/s

        self.v_x = x
        self.v_y = y
        self.v_th = th
    
    def imuDataCallback(self, req):
        self.quaternion = req.orientation

    def imuRPYCallback(self, req):
        #TODO - implement rosservice call to reset if value is nan
        #NOTE - when calling /reset service, speeds dies
        if math.isnan(req.z):
            self.angle = 0 
        else:
            self.angle = -req.z

    def intakeCallback(self, req):
        self.intake_command = req.data

    def elevatorCallback(self, req):
        self.elevator_command = req.data
    
    def resetOdomCallback(self, req):
        if req.data:
            self.Microcontroller.reset_odometry()
    
    def warehouseCallback(self, req):
        self.warehouse = req.data


class MicroControllerROS():
    def __init__(self):
        rospy.init_node('Microcontroller', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", default="/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 10))
        r = rospy.Rate(self.rate)


        self.use_base_controller = rospy.get_param("~use_base_controller", True)
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # Initialize the controlller
        self.controller = Microcontroller(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Microcontroller on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = _thread.allocate_lock()
              
        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame)
    
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()
            r.sleep()
    
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Microcontroller Node...")

def testController():
    # Initialize the controlller
    port = "/dev/ttyACM0"
    baud = 115200 
    timeout = 0.1
    controller = Microcontroller(port, baud, timeout)
    controller.connect()
    rospy.loginfo("Connected to Microcontroller on port " + port + " at " + str(baud) + " baud")
    print(controller.get_hardware_version())
    print(controller.get_baud())
    print(controller.get_odometry())
    
    print(controller.stop())
    
    velX = [0.7, 0.0, -0.7, 0.0]
    velY = [0.0, 0.7, 0.0, -0.7]
    imu_vals = [0.0, 2.0, -2.0, 10.0]
    index = 0
    start_time = time.time()

    while(1):
        print(controller.get_odometry())
        if time.time() - start_time > 2.5:
            index = (index + 1) % 4
            start_time = time.time()
        controller.drive(velX[index], velY[index], 0.0)
        #controller.drive(0.0, 0.0, 0.0)
        controller.imu_angle(imu_vals[index])
        time.sleep(0.1) 

if __name__ == '__main__':
    myMicroController = MicroControllerROS()
    #testController()
    
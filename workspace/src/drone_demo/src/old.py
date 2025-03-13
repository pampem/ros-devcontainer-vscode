#!/usr/bin/env python

# use LQR control to control drone xy
# use P-PID-MRAC to control drone z
## This program is used for uav5 control in experimnt

from std_msgs.msg import *
from mavros_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import *
from math import *
from importlib.resources import *
from tf import transformations
from unicodedata import *
from ast import *
from mpl_toolkits.mplot3d import *
from scipy.signal import butter, filtfilt
from mmap import ACCESS_COPY
import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import random
import matplotlib.pyplot as plt
import math, time
import matplotlib
import matplotlib.font_manager as fm

# define important parameter for /mavros/setpoint_raw/local
FRAME_LOCAL_NED=1
FRAME_LOCAL_OFFSET_NED=7
FRAME_BODY_NED=8
FRAME_BODY_OFFSET_NED=9
IGNORE_PX=1
IGNORE_PY=2
IGNORE_PZ=4
IGNORE_VX=8
IGNORE_VY=16
IGNORE_VZ=32
IGNORE_AFX=64
IGNORE_AFY=128
IGNORE_AFZ=256
FORCE=512
IGNORE_YAW=1024
IGNORE_YAW_RATE=2048

# define important parameter for /mavros/setpoint_raw/global
FRAME_GLOBAL_INT=5
FRAME_GLOBAL_REL_ALT=6
FRAME_GLOBAL_TERRAIN_ALT=11
IGNORE_LATITUDE=1
IGNORE_LONGITUDE=2
IGNORE_ALTITUDE=4
IGNORE_VX=8
IGNORE_VY=16
IGNORE_VZ=32
IGNORE_AFX=64
IGNORE_AFY=128
IGNORE_AFZ=256
FORCE=512
IGNORE_YAW=1024
IGNORE_YAW_RATE=2048


# for attitude control
IGNORE_ROLL_RATE=1
IGNORE_PITCH_RATE=2
IGNORE_YAW_RATE_ATT=4
IGNORE_THRUST=64
IGNORE_ATTITUDE_ATT=128

# general function
def constrain(inpu_signal, max_signal, min_signal):
    if inpu_signal>=max_signal:
        return max_signal
    else:
        if inpu_signal<=min_signal:
            return min_signal
        else:
            return inpu_signal

## distance calculation
def distance_calculation(x1,x2,y1,y2):
    return math.sqrt((x1-x2) ** (2) + ((y1-y2) ** (2))) 

## solve parameter drift
def safe_check(input_number, add_number, max_num, min_num):
    if input_number >= max_num:
        if add_number>=0:
            add_number = 0

    if input_number <= min_num:
        if add_number<=0:
            add_number = 0
    
    return np.nan_to_num(input_number+add_number)
    
# butter filter
## fs: sampe hz
## fc: stop hz
class MY_BUTTER_CLASS:

    def __init__(self,fs, fc):
        self.b, self.a = butter(4, fc/(fs/2), "lowpass")
        self.input_signal_list = np.zeros(100)

    def my_butter_filter(self, input_signal):
        self.input_signal_list = np.concatenate((self.input_signal_list, input_signal), axis = None) 
        signal_need_to_be_filter = self.input_signal_list[-100:]
        signal_after_filter = filtfilt(self.b, self.a, signal_need_to_be_filter)
        return signal_after_filter 

# PID controller class
class MY_PID_CONTROLLER:
    # initialize class
    def __init__(self, kp, ki, kd, dt):
        # basic parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.my_filter = MY_BUTTER_CLASS(100.0, 10)

        # error to store for derivate
        self.last_error = 0.0
        self.integral = 0.0
    
    def update(self, error):
        error = np.nan_to_num(error)
        
        # use the filter to cutoff noise
        error = self.my_filter.my_butter_filter(error)
        error = error[-1]

        #self.integral += error*self.dt 
        self.integral = safe_check(self.integral, error*self.dt, 1.0, -1.0)
        self.integral = constrain(self.integral,5.0,-5.0)
        derivative = (error- self.last_error)/self.dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        # update error
        self.last_error = error
        # return result, need to be constained
        return output

# 2nd order ESO
class MY_SECOND_ORDER_ESO:
    # initialize class
    def __init__(self, w0, b0, dt):
        # basic parameters
        self.z1_hat = 0
        self.z2_hat = 0
        self.dis = 0
        self.w0 = w0
        self.b0 = b0
        self.dt = dt
        self.my_filter1 = MY_BUTTER_CLASS(100.0, 10)
        self.my_filter2 = MY_BUTTER_CLASS(100.0, 10)
    
    def update(self, u_final, feedback, b0):
        # pass the filter first
        u_final = self.my_filter1.my_butter_filter(u_final)
        feedback = self.my_filter2.my_butter_filter(feedback)
        u_final = u_final[-1]
        feedback = feedback[-1]

        error = self.z1_hat - feedback 
        dot_z1_hat = self.z2_hat + b0*u_final- 2*self.w0*error
        dot_z2_hat = -self.w0*self.w0*error
        self.z1_hat +=dot_z1_hat*self.dt
        self.z2_hat +=dot_z2_hat*self.dt
        # return result, need to be constained
        self.dis = self.z2_hat/b0

# referrence model
class MY_REFERRENCE_MODEL_Z:
    # initialize class
    def __init__(self, mass, dt):
        # model velocity
        self.Vm = 0
        self.mass = mass
        self.dt = dt
    
    def update_referrence_model(self, u_pid_acc):
        g = 9.8
        dot_Vm = 0.0
        #if abs(round((u_pid_acc/g)*0.5-0.5,3))<0.05:
            #dot_Vm = 0.0
            #print(round((u_pid_acc/g)-0.5,3))

        if (abs(round((u_pid_acc/g)*0.5-0.5,3))>=0.05)and(u_pid_acc/g<=2.0)and(u_pid_acc/g>=0.2):
            #dot_Vm = (((u_pid_acc/g)-0.5)/0.5)*0.2
            dot_Vm = round((u_pid_acc/g)*0.5-0.5,3)*0.3
            #dot_Vm = u_pid_acc-9.80665
            #print("big",round((u_pid_acc/g)-0.5,3))
        
        self.Vm = self.Vm + self.dt*dot_Vm
        return self.Vm

# adaptive controller
class MY_ADAPTIVE_CONTROLLER:
    # initialize class
    def __init__(self, refer_mass, gama_x, gama_r, k, dt):
        self.my_refer_model = MY_REFERRENCE_MODEL_Z(refer_mass, dt)
        self.gama_x = gama_x
        self.gama_r = gama_r
        self.k = k
        self.kx = 0
        self.kr = 1
        self.theta = 0
        self.dt = dt
    
    def update(self, u_pid_acc, feedback):
        self.my_refer_model.update_referrence_model(u_pid_acc)
        error = self.my_refer_model.Vm - feedback
        dot_kx = self.gama_x*feedback*error
        dot_kr = self.gama_r*u_pid_acc*error
        dot_theta = -self.k*1*error
        self.kx = safe_check(self.kx, self.dt * dot_kx, 1.0, -1.0)
        self.kr = safe_check(self.kr, self.dt * dot_kr, 1.2, 0.9)
        self.theta = self.theta + self.dt*dot_theta
        #if self.kr>1.5:
            #self.kr = 1.0

        #self.kx = self.kx + self.dt * dot_kx
        #self.kr = self.kr + self.dt * dot_kr
        return self.kr*u_pid_acc +self.kx*feedback -self.theta*1.0

# backsteping class
class MY_BACKSTEPPING_CONTROLLER:
    # initialize class
    def __init__(self, m, k1, k2, dt):
        self.m = m
        self.k1 = k1
        self.k2 = k2
        # For calculation of dot_xd, dot_dot_xd
        self.last_xd = 0
        self.last_dot_xd = 0
        self.dt = dt
    
    def calculation_of_dot_xd(self, new_xd):
        dot_xd = (new_xd-self.last_xd)/self.dt
        self.last_xd = new_xd
        return dot_xd
    
    def calculation_of_dot_dot_xd(self, new_dot_xd):
        dot_dot_xd = (new_dot_xd-self.last_dot_xd)/self.dt
        self.last_dot_xd = new_dot_xd
        return dot_dot_xd

    def update(self,xd, xp, vp):
        dot_xd = self.calculation_of_dot_xd(xd)
        dot_dot_xd = self.calculation_of_dot_dot_xd(dot_xd)
        e1 = constrain(xd-xp, 1.0, -1.0)
        e2 = constrain(dot_xd-vp, 1.0, -1.0)
        u0 = self.m*dot_dot_xd+ self.k1*e1+self.k2*e2
        return u0

# LQR controller class
class MY_LQR_CONTROLLER:
    """
    * https://github.com/schlagenhauf/lqr_solve/blob/master/lqr_solve.cpp
    * @brief Computes the LQR gain matrix (usually denoted K) for a discrete time
    * infinite horizon problem.
    *
    * @param A State matrix of the underlying system
    * @param B Input matrix of the underlying system
    * @param Q Weight matrix penalizing the state
    * @param R Weight matrix penalizing the controls
    * @param N Weight matrix penalizing state / control pairs
    * @param K Pointer to the generated matrix (has to be a double/dynamic size
    * matrix!)
    * @param eps Delta between iterations that determines when convergence is
    * reached
    """
    def __init__(self, eps, dt):
        self.eps = eps
        LQR_A = np.eye(4)
        LQR_B = np.zeros((4,2))
        LQR_Q = 0.5*np.eye(4)
        LQR_R = 0.1*np.eye(2)
        LQR_N = np.zeros((4,2))
        LQR_A[0,2] = dt
        LQR_A[1,3] = dt
        LQR_B[2,0] = dt
        LQR_B[3,1] = dt
        self.k= self.calculate_k(LQR_A, LQR_B, LQR_Q, LQR_R, LQR_N)
        self.u_LQR_xy = np.zeros((2,1))
    
    def calculate_k(self,A, B, Q, R, N):
        if (A.shape[0]==A.shape[1])and(B.shape[0]==A.shape[0])\
            and(Q.shape[0]==Q.shape[1])and(Q.shape[0]==A.shape[0])\
            and(R.shape[0]==R.shape[1])and(R.shape[0]==B.shape[1])\
            and(N.shape[0]==A.shape[0])and(N.shape[1]==B.shape[1]):
            # get here means that matrix is fine

            # precompute some matrix
            B_Trans = np.transpose(B)
            # R reverse matrix
            R_inv = np.linalg.inv(R)
            Acal = A -  np.dot( np.dot(B,R_inv), np.transpose(N))
            Acal_Trans = np.transpose(Acal)
            Qcal = Q - np.dot( np.dot(N, R_inv), np.transpose(N)) 

            # initialize P with Q
            P_temp = Q

            # iterate until P converges
            P_old =P_temp
            while True:
                inver = np.linalg.inv( R + np.dot(np.dot(B_Trans, P_temp), B) )
                P_temp = np.dot(np.dot(Acal_Trans,P_temp),Acal) -  \
                         np.dot(np.dot(np.dot(np.dot( np.dot(np.dot(np.dot(Acal_Trans,P_temp),B), inver), B_Trans), P_temp),Acal), Qcal)
                delta = P_temp - P_old
                if fabs(np.amax(delta))< self.eps :
                    break
                P_old = P_temp
            
            # get here means that while loop is break
            part_1 = np.linalg.inv( R + np.dot( np.dot(B_Trans,P_temp), B))
            part_2 = np.dot(np.dot(B_Trans, P_temp), A) + np.transpose(N)
            K_matrix = np.dot(part_1, part_2)
            return K_matrix
        else:
            print("check matrix rows and cols")

    def LQR_updata(self, target_xy, target_vxy, target_axy, state_xy_vxy):
        # transfer target xy vxy into 4*1 matrix
        Target_matrix = np.array([target_xy,target_vxy]).reshape(4,1)
        taget_acc_matrix = np.array(target_axy).reshape(2,1)
        state_matrix = np.array([state_xy_vxy]).reshape(4,1)
        self.u_LQR_xy = np.dot(self.k, Target_matrix - state_matrix) + taget_acc_matrix

# get target position from my defined topic: my_target_position
class MY_GET_TARGET_POSITION:
    def __init__(self):
    # target position for each drone
        self.Tx_4 =0
        self.Ty_4 =0
        self.Tz_4 =0
        self.Tx_5 =0
        self.Ty_5 =0
        self.Tz_5 =0
    
    # sub to my_get_target_position_topic
        self.T_pose_read = rospy.Subscriber("my_get_target_position_topic", Float32MultiArray, self.T_pose_read_callback) 
    
    def T_pose_read_callback(self, msg):
        self.Tx_4 = msg.data[0]
        self.Ty_4 = msg.data[1]
        self.Tz_4 = msg.data[2]
        self.Tx_5 = msg.data[3]
        self.Ty_5 = msg.data[4]
        self.Tz_5 = msg.data[5]


# drone class
class MY_DRONE:

    def __init__(self,drone_number_string):
        self.drone_number = drone_number_string
        # drone actual position velocity acceleration attitude
        self.pos_x =0
        self.pos_y =0
        self.pos_z =0
        self.vel_x =0
        self.vel_y =0
        self.vel_z =0
        self.acc_x =0
        self.acc_y =0
        self.acc_z =0
        self.acc_x_ground =0
        self.acc_y_ground =0
        self.acc_z_ground =0
        self.roll = 0
        self.pitch =0
        self.yaw = 0
        self.roll_imu = 0
        self.pitch_imu =0
        self.yaw_imu = 0
        # drone target position velocity acceleration
        self.Tp_x = 0
        self.Tp_y = 0
        self.Tp_z = 0
        self.Tv_x = 0
        self.Tv_y = 0
        self.Tv_z = 0
        self.Ta_x = 0
        self.Ta_y = 0
        self.Ta_z = 0
        # drone target attitude thrust
        self.T_roll =0
        self.T_pitch =0
        self.T_yaw =0
        self.T_thrust =0
        # rc 
        self.rc1=0
        self.rc2=0
        self.rc3=0
        self.rc4=0
        self.rc5=0
        self.rc6=0
        self.rc7=0
        self.rc8=0
        self.rc9=0
        # pwm 
        self.pwm1=0
        self.pwm2=0
        self.pwm3=0
        self.pwm4=0
        self.pwm5=0
        self.pwm6=0
        self.base_pwm = 0
        # force
        self.m = 1.66
        self.Force = 0
        self.fx =0
        self.fy =0
        self.fz =0
        self.force_ground = Float32MultiArray()
        self.hover_thrust_percentage = 0.5
        # ESO
        self.w0 = 5
        self.ESO_x = MY_SECOND_ORDER_ESO(5.0, 1/self.m, 0.01)
        self.ESO_y = MY_SECOND_ORDER_ESO(5.0, 1/self.m, 0.01)
        self.ESO_z = MY_SECOND_ORDER_ESO(5.0, 1/self.m, 0.01)
        self.ESO_list = Float32MultiArray()
        # flight mode
        self.mode = "S"
        self.arm_flag = True
        # data published to uav1
        self.uav_pub_data = PositionTarget()
        self.uav_pub_target_attitude = AttitudeTarget()	
        ##############
        # topitc to publish
        # # set target position or velocity or acceleration 
        self.my_setpoint_pub = rospy.Publisher(self.drone_number + "mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.my_set_target_attitude = rospy.Publisher(self.drone_number + "mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

        # publish foce calculated below
        self.my_force_ground = rospy.Publisher(self.drone_number + "force_ground", Float32MultiArray, queue_size=10)
        # publish ESO calculated below
        self.my_ESO_pub = rospy.Publisher(self.drone_number + "ESO", Float32MultiArray, queue_size=10)
        ##############
        # read in data from 
        self.joy_data_read = rospy.Subscriber(self.drone_number + "mavros/rc/in", RCIn, self.joy_callback)
        # get actual position and actual attitude
        self.pose_read = rospy.Subscriber(self.drone_number + "mavros/local_position/pose", PoseStamped, self.pose_callback)
        # get actual velocity 
        self.vel_read = rospy.Subscriber(self.drone_number + "mavros/local_position/velocity_local", TwistStamped, self.vel_callback)
        # get actual acceleration 
        self.acc_read = rospy.Subscriber(self.drone_number + "mavros/imu/data", Imu, self.imu_callback)
        # get actula flight state
        self.state_read = rospy.Subscriber(self.drone_number + "mavros/state", State, self.state_callback) 
        # get pwm of each motor 
        self.pwm_read = rospy.Subscriber(self.drone_number + "mavros/rc/out", RCOut, self.pwm_callback) 
        ##############

    # return radion data from -1 to 1
    def get_radion_data(self, radio_input, radio_max, radion_min):
        radio_input = constrain(radio_input, radio_max, radion_min)			
        if radio_max>radion_min:
            radio_data = 2*(radio_input-radion_min)/(radio_max-radion_min) -1
        return radio_data	
        
    # get radion data for furthuer use	
    def joy_callback(self, msg):
        # change radio input to -1 ~ 1
        self.rc1 = self.get_radion_data(msg.channels[0],1932,1092)
        self.rc2 = self.get_radion_data(msg.channels[1],1935,1095)   
        self.rc3 = self.get_radion_data(msg.channels[2],1935,1092)
        self.rc4 = self.get_radion_data(msg.channels[3],1925,1105)
        self.rc5 = self.get_radion_data(msg.channels[4],2065,965)
        self.rc6 = self.get_radion_data(msg.channels[5],1935,1095)
        self.rc7 = self.get_radion_data(msg.channels[6],2065,965)
        self.rc8 = self.get_radion_data(msg.channels[7],1935,1095)
        self.rc9 = self.get_radion_data(msg.channels[8],2065,965)

    # get actual position data	
    def pose_callback(self, msg):
        #self.attitude = transformations.euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        self.pos_x=msg.pose.position.x
        self.pos_y=msg.pose.position.y
        self.pos_z=msg.pose.position.z
        self.roll, self.pitch, self.yaw = transformations.euler_from_quaternion( [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        #print("read pos")

    # get actual velocity data
    def vel_callback(self, msg):		
        self.vel_x=msg.twist.linear.x
        self.vel_y=msg.twist.linear.y
        self.vel_z=msg.twist.linear.z
        #print("read vel")	
    
    # get actual acceleration data
    def imu_callback(self, msg):	
        self.acc_x=msg.linear_acceleration.x
        self.acc_y=msg.linear_acceleration.y
        self.acc_z=msg.linear_acceleration.z
        self.roll_imu, self.pitch_imu, self.yaw_imu = transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.pitch_imu = -self.pitch_imu #due to the tranfromations function coordinate problem, pitch must *-1
        self.acc_x_ground
        #print("read acc")	
    
    # get actual flight state
    def state_callback(self, msg):
        self.mode = msg.mode
        self.arm_flag = msg.armed

    # set target position
    def set_target_position(self, Tx, Ty, Tz, T_yaw=0.0):
        # publish target position 
        self.Tp_x = Tx
        self.Tp_y = Ty
        self.Tp_z = Tz
        self.uav_pub_data.coordinate_frame = FRAME_LOCAL_NED
        self.uav_pub_data.type_mask = IGNORE_VX+IGNORE_VY+IGNORE_VZ+IGNORE_AFX+IGNORE_AFY+IGNORE_AFZ+IGNORE_YAW_RATE
        self.uav_pub_data.position.x = self.Tp_x  
        self.uav_pub_data.position.y = self.Tp_y  
        self.uav_pub_data.position.z = self.Tp_z
        self.uav_pub_data.yaw = T_yaw
        self.my_setpoint_pub.publish(self.uav_pub_data)
    
    # set target velocity
    def set_target_velocity(self, Tvx, Tvy, Tvz, T_yaw_rate=0.0):
        # publish target position 
        self.Tv_x = Tvx
        self.Tv_y = Tvy
        self.Tv_z = Tvz
        self.uav_pub_data.coordinate_frame = FRAME_LOCAL_NED
        self.uav_pub_data.type_mask = IGNORE_PX+IGNORE_PY+IGNORE_PZ+IGNORE_AFX+IGNORE_AFY+IGNORE_AFZ+IGNORE_YAW
        self.uav_pub_data.velocity.x = self.Tv_x  
        self.uav_pub_data.velocity.y = self.Tv_y  
        self.uav_pub_data.velocity.z = self.Tv_z
        self.uav_pub_data.yaw_rate = T_yaw_rate
        self.my_setpoint_pub.publish(self.uav_pub_data)

    # set target acceleration
    def set_target_acceleration(self, Tax, Tay, Taz):
        # publish target position 
        self.Ta_x = Tax
        self.Ta_y = Tay
        self.Ta_z = Taz
        self.uav_pub_data.coordinate_frame = FRAME_LOCAL_NED
        self.uav_pub_data.type_mask = IGNORE_PX+IGNORE_PY+IGNORE_PZ+IGNORE_VX+IGNORE_VY+IGNORE_VZ+IGNORE_YAW_RATE
        self.uav_pub_data.acceleration_or_force.x = self.Ta_x  
        self.uav_pub_data.acceleration_or_force.y = self.Ta_y  
        self.uav_pub_data.acceleration_or_force.z = self.Ta_z
        self.uav_pub_data.yaw = 0
        self.my_setpoint_pub.publish(self.uav_pub_data)
    
    # stop publish pos vel acc
    def stop_pub_pos_vel_acc(self):
        self.uav_pub_data.type_mask = IGNORE_PX+IGNORE_PY+IGNORE_PZ+IGNORE_VX+IGNORE_VY+IGNORE_VZ+IGNORE_AFX+IGNORE_AFY+IGNORE_AFZ+IGNORE_YAW
        self.my_setpoint_pub.publish(self.uav_pub_data)

    # from roll pitch yaw get xyzw
    def get_xyzw_from_rpy(self,roll, pitch, yaw, axes='sxyz'):
        #print("%f,%f,%f" %(roll,pitch,yaw))
        return transformations.quaternion_from_euler(roll, pitch, yaw, axes)

    # get pwm of each motor
    def pwm_callback(self,msg):
        self.pwm1 = msg.channels[0]
        self.pwm2 = msg.channels[1]
        self.pwm3 = msg.channels[2]
        self.pwm4 = msg.channels[3]
        self.pwm5 = msg.channels[4]
        self.pwm6 = msg.channels[5]

    def force_cal(self):
        arm_pwm = 6600
        pwm = self.pwm1+self.pwm2+self.pwm3+self.pwm4+self.pwm5+self.pwm6
        Force = (pwm/self.base_pwm)*(self.m*9.8) 
        #Force = (pwm-arm_pwm)/(self.base_pwm-arm_pwm)*(self.m*9.8) 
        self.Force = Force
        self.fx = ( cos(self.yaw)*sin(self.pitch)*cos(self.roll)+sin(self.yaw)*sin(self.roll))*Force
        self.fy = ( cos(self.roll)*sin(self.pitch)*sin(self.yaw)-sin(self.roll)*cos(self.yaw))*Force
        #Force = (pwm-arm_pwm)/(self.base_pwm-arm_pwm)*(self.m*9.8)
        self.fz = ( cos(self.roll)*cos(self.pitch))*Force
        self.force_ground.data = [self.fx, self.fy, self.fz]

        # publish data
        self.my_force_ground.publish(self.force_ground)
        return self.fx, self.fy, self.fz
    
    def ESO_cal(self):
        self.force_cal()
        self.ESO_x.update(self.fx, self.vel_x, 1/self.m)
        self.ESO_y.update(self.fy, self.vel_y, 1/self.m)
        self.ESO_z.update(self.fz, self.vel_z, 1/self.m)
        self.ESO_list.data = [self.ESO_x.dis, self.ESO_y.dis, self.ESO_z.dis]
        self.my_ESO_pub.publish(self.ESO_list)

    # set target roll pitch yaw thrust
    def set_target_rpy_thrust(self, roll, pitch, yaw, thrust):
        #self.stop_pub_pos_vel_acc()
        # for coordinate system translation problem roll need to be reversed
        self.T_roll =-roll
        self.T_pitch =pitch
        self.T_yaw =yaw
        x,y,z,w=self.get_xyzw_from_rpy(self.T_roll, self.T_pitch, self.T_yaw)
        self.uav_pub_target_attitude.type_mask = IGNORE_ROLL_RATE+IGNORE_PITCH_RATE+IGNORE_YAW_RATE_ATT
        self.uav_pub_target_attitude.orientation.x = x
        self.uav_pub_target_attitude.orientation.y = y
        self.uav_pub_target_attitude.orientation.z = z
        self.uav_pub_target_attitude.orientation.w = w
        self.uav_pub_target_attitude.thrust = thrust
        self.my_set_target_attitude.publish(self.uav_pub_target_attitude)

    # calculate target roll pitch
    def from_target_acc_to_att(self, accx, accy, yaw):
        #for some coordinate system translation problem, 
        # input of accx accy accz and yaw need to be changed as follows:
        accx = -accx
        yaw = -yaw
        accel_forward = accx * cos(yaw) + accy * sin(yaw)
        accel_right = -accx * sin(yaw) + accy *cos(yaw)
        target_pitch = atan(-accel_forward / 9.8) 
        target_roll = atan( (accel_right * cos(target_pitch)) /9.8)
        return target_roll, target_pitch

    # stop pub attitude thrust
    def stop_pub_att_thrust(self):
         self.uav_pub_target_attitude.type_mask = IGNORE_ROLL_RATE+IGNORE_PITCH_RATE+IGNORE_YAW_RATE_ATT+IGNORE_ATTITUDE_ATT+IGNORE_THRUST
         self.my_set_target_attitude.publish(self.uav_pub_target_attitude)

    # pass the pose controller result acc_x acc_y acc_z and target_yaw, actual_yaw to copter
    # by trans those data abpve to traget_roll target_pitch target_yaw and thrust_percentage 
    # the publish them to copter
    def my_pass_acc_to_att_to_copter(self, acc_x, acc_y, acc_z, T_yaw):
        traget_roll, target_pitch = self.from_target_acc_to_att(acc_x, acc_y, self.yaw)
        thrust_percentage = (acc_z/9.8)*self.hover_thrust_percentage
        self.set_target_rpy_thrust(traget_roll, target_pitch, T_yaw, thrust_percentage)


if __name__ == '__main__':
    rospy.init_node('psc_control_test2', anonymous=True)
    uav0=MY_DRONE("/drone0/")
    uav1=MY_DRONE("/drone1/")
    uav2=MY_DRONE("/drone2/")
    uav3=MY_DRONE("/drone3/")

    # for real drone 5 mass is 2.02kg (drone with 2battery and bateery protect)
    uav1.m = 2.02
    # for real drone 5 base pwm is 9025
    uav1.base_pwm = 9025

    # xy only used in test
    my_pid_x =MY_PID_CONTROLLER(1.0, 0.0, 0.0, 0.01)
    my_pid_y =MY_PID_CONTROLLER(0.8, 0.0, 0.0, 0.01)
    my_pid_z =MY_PID_CONTROLLER(2, 0.0, 0.0, 0.01)
    my_pid_vel_x =MY_PID_CONTROLLER(2, 0.0, 0.3, 0.01)
    my_pid_vel_y =MY_PID_CONTROLLER(2, 0.0, 0.3, 0.01)
    my_pid_vel_z =MY_PID_CONTROLLER(2.0, 0.8, 0.2, 0.01)

    my_adaptive_z = MY_ADAPTIVE_CONTROLLER(1, 0, 0.005, 0.0, 0.01)

    my_LQR_controller_xy = MY_LQR_CONTROLLER(1e-5, 0.01)

    my_get_target_position= MY_GET_TARGET_POSITION()


    while not rospy.is_shutdown():
        #cal ESO
        uav1.ESO_cal()
        # give target position
        Tx1 = uav0.rc2*1 
        Ty1 = uav0.rc1*1.0 
        Tz123 = 1.55+uav0.rc3*1.0
        T_yaw = 0
        Tx23 = Tx1 +2
        Ty2 = Ty1 -2
        Ty3 = Ty1 +2 
        # go back to start position

        uav1.set_target_position(Tx1, Ty1, Tz123, T_yaw)
        uav2.set_target_position(Tx23, Ty2, Tz123, T_yaw)
        uav3.set_target_position(Tx23, Ty3, Tz123, T_yaw)
        #print("pos, %.3f, %.3f, %.3f, %.3f"%(uav1.pos_x, uav1.pos_y, uav1.pos_z, uav1.yaw))
        #print("pos, %.3f, R:%.3f,     P:%.3f,      Y:%.3f"%(uav0.rc2*5, uav1.roll*100, uav1.pitch*100, uav1.yaw_imu))
        print("pwm, %.3f, %.3f, %.3f"%(Tx1, Ty1, Tz123))
        #t=np.arange(0,1, 1/100)
        #x1 = np.sin(2*3.14*5*t)+np.random.randn(len(t))
        #print(uav2.pos_z)
    
        rospy.Rate(100).sleep()
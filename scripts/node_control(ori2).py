#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Error, Actuator, Movement, IsStable
import numpy as np

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = 80

        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.start_time = rospy.get_time()
        self.last_time = 0

        self.last_error = 0

    def __call__(self, error):
        dt = rospy.get_time() - (self.last_time if self.last_time is not None else self.start_time)
        # error = error/1000
        d_error = error - self.last_error

        self.proportional = self.kp * error
        self.integral += -self.ki * error * dt
        self.derivative = self.kd * d_error / dt

        # self.proportional = self.kp * error
        # self.integral += -self.ki * error
        # self.derivative = self.kd * error/d_error

        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max

        self.last_error = error
        self.last_time = rospy.get_time()

        return self.proportional + self.integral + self.derivative

class ThrusterMovement():
    def __init__(self):
        self.pwm_actuator = Actuator()
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500

        self.pub_pwm_actuator = rospy.Publisher('pwm_actuator', Actuator, queue_size=10)

    def surge_sway_yaw(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4

    def heave_roll_pitch(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5
        self.pwm_actuator.thruster_6 = pwm_thruster_6
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    def boost(self, pwm_thruster_9, pwm_thruster_10):
        self.pwm_actuator.thruster_9 = pwm_thruster_9
        self.pwm_actuator.thruster_10 = pwm_thruster_10

    def surge(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4):
        self.pwm_actuator.thruster_1 = pwm_thruster_1 
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3 
        self.pwm_actuator.thruster_4 = pwm_thruster_4

    def sway(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4

    def yaw(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4

    def heave(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5 
        self.pwm_actuator.thruster_6 = pwm_thruster_6 
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    def roll(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5
        self.pwm_actuator.thruster_6 = pwm_thruster_6
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    def pitch(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5 
        self.pwm_actuator.thruster_6 = pwm_thruster_6 
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    # def pitch(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
    #     self.pwm_actuator.thruster_5 = pwm_thruster_5 
    #     self.pwm_actuator.thruster_6 = pwm_thruster_6 
    #     if pwm_thruster_5 < 1500:
    #         self.pwm_actuator.thruster_7 =  (1500 - pwm_thruster_5)+1500
    #     elif pwm_thruster_5 > 1500:
    #         self.pwm_actuator.thruster_7 = 1500 - (pwm_thruster_5 - 1500)
    #     else:
    #         sel.pwm_actuator.thruster_7 = pwm_thruster_7

    #     if pwm_thruster_6 < 1500:
    #         self.pwm_actuator.thruster_8 =  (1500 - pwm_thruster_6)+1500
    #     elif pwm_thruster_6 > 1500:
    #         self.pwm_actuator.thruster_8 = 1500 - (pwm_thruster_6 - 1500)
    #     else:
    #         sel.pwm_actuator.thruster_8 = pwm_thruster_8

    def stop(self):
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500

    def publish(self):
        print('Publish PWM', self.pwm_actuator)
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.error = Error()
        self.is_stable = IsStable() 
        self.movement = ThrusterMovement()
        self.movement.stop()
        
        self.param_arming_duration = rospy.get_param('/nuc/arming_duration')

        self.pid_heave = PID(1600, 0, 200)
        #self.pid_heave = PID(1600, 100, 0)
        #self.pid_heave = PID(1000, 100, 500)
        self.pid_roll = PID(400, 0, 10)
        self.pid_pitch = PID(1000, 0, 500)
        # self.pid_pitch = PID(700, 0, 500)
        
        #self.pid_roll = PID(400, 0, 10)
        #self.pid_pitch = PID(400, 0, 10)
        
        # self.pid_yaw = PID(10, 1,  5)

        # self.pid_yaw = PID(10, 0.5,  5)

        self.pid_yaw = PID(3, 0,  0)

        self.start_time = 0
        self.is_armed = False
        self.is_pre_calibrating = False
        self.dive = False

        self.offset_roll = 0
        self.offset_pitch = 0
        self.offset_yaw = 0
        
        self.offset_surge = 0
        self.offset_sway = 0
        self.offset_heave = 0

        self.pwm_roll = 0
        self.pwm_pitch = 0
        self.pwm_yaw = 0

        self.pwm_surge = 0
        self.pwm_sway = 0
        self.pwm_heave = 0

        # Subscriber
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('movement', Movement, self.callback_movement)
        rospy.Subscriber('is_stable', IsStable, self.callback_is_stable)

        self.pub_dive = rospy.Publisher('dive',Bool,queue_size=10)

    def constrain(self, value, _min, _max):
        return min(max(value, _min), _max)
    
    def pre_calibrate(self):
        self.offset_roll = self.pwm_roll
        self.offset_pitch = self.pwm_pitch
        self.offset_yaw = self.pwm_yaw
        
        self.offset_surge = self.pwm_surge
        self.offset_sway = self.pwm_sway
        self.offset_heave = self.pwm_heave

    def get_offset(self, offset):
        return offset if not self.is_pre_calibrating else 0

    def force_calculation(self) :
        self.d = 1. #dalam meter
        self.sqrt2 = np.sqrt(2)
        
        # Matriks transformasi
        self.M = np.array([
            [1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2],
            [1/self.sqrt2, -1/self.sqrt2, -1/self.sqrt2, 1/self.sqrt2],
            [d/2, -d/2, -d/2, d/2]
        ])
    def control(self, Fx, Fy, tau):
        # Vektor gaya yang diinginkan
        F = np.array([Fx, Fy, tau])
        
        # Menghitung thrust yang diperlukan menggunakan pseudo-inverse matriks
        T = np.linalg.pinv(self.M).dot(F)

        return T

    def thruster_value_calculation(self) :
        # Gaya yang diinginkan (Fx, Fy, tau)
        Fx = 0# Gaya maju
        Fy = 0   # Gaya ke samping
        tau = self.constrain(self.pwm_yaw, -2, 2) # Momen yaw

        print(f"tau : {tau}")

        # Hitung thrust yang diperlukan untuk setiap thruster
        thrusts = self.control(Fx, Fy, tau)
        # thrust.thruster_1 = thrusts[0]
        # thrust.thruster_2 = thrusts[1]
        # thrust.thruster_3 = thrusts[2]
        # thrust.thruster_4 = thrusts[3]


    

    # ORI
    def surge_yaw(self):
        min_pwm = 1000
        max_pwm = 2000
        pwm_thruster_1 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        self.movement.surge(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)


    # def surge_yaw(self):
    #     min_pwm = 1000
    #     max_pwm = 2000

    #     if self.is_stable.yaw:
    #         print("STABLE")
    #         pwm_thruster_1 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
    #         pwm_thruster_2 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
    #         pwm_thruster_3 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
    #         pwm_thruster_4 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
    #     else:
    #         print("NOT STABLE")
    #         pwm_thruster_1 = self.constrain(1500 + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #         pwm_thruster_2 = self.constrain(1500 - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #         pwm_thruster_3 = self.constrain(1500 + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #         pwm_thruster_4 = self.constrain(1500 - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     self.movement.surge(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    def surge(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_1 = 1500
        # self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
        pwm_thruster_2 = 1500
        # self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
        pwm_thruster_3 = 1500 #*(0.01*1558)
        # self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
        pwm_thruster_4 = 1500 
        # self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)), min_pwm, max_pwm)
        self.movement.surge(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    def sway(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_1 = self.constrain(1500 + (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 - (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 - (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 + (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
        self.movement.sway(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    def yaw(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_1 = self.constrain(1500 + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        self.movement.yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    def heave_roll_pitch(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 - (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 - (self.pwm_heave - self.get_offset(self.offset_heave)) - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)  
        self.movement.heave_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def heave_roll(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)  
        self.movement.heave_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def heave_pitch(self):
        min_pwm = 1000
        max_pwm = 2000
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) , min_pwm, max_pwm)
        # - (self.pwm_pitch - self.get_offset(self.offset_pitch))
        pwm_thruster_8 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) , min_pwm, max_pwm)
        # - (self.pwm_pitch - self.get_offset(self.offset_pitch))  
        self.movement.heave_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def roll_pitch(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)  
        self.movement.heave_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)
    
    def heave(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)), min_pwm, max_pwm)  
        self.movement.heave(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)
    
    def roll(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)  
        self.movement.roll(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def pitch(self):
        min_pwm = 1200
        max_pwm = 1800
        pwm_thruster_5 = self.constrain(1500 - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)  
        self.movement.pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def stabilize_roll(self, error):
        self.pwm_roll = self.pid_roll(error)

    def stabilize_pitch(self, error):
        self.pwm_pitch = self.pid_pitch(error)

    def stabilize_yaw(self, error):
        print("PWM YAW",self.pwm_yaw)
        self.pwm_yaw = self.pid_yaw(error)
        # self.thruster_value_calculation(self.pwm_yaw)

    def stabilize_depth(self, error):
        self.pwm_heave = self.pid_heave(error)

    #Check if Stable
    def callback_is_stable(self, data: IsStable):
        self.is_stable.depth = data.depth
        self.is_stable.yaw = data.yaw
        self.is_stable.pitch = data.pitch
        self.is_stable.roll = data.roll

    # Collect Error Data
    def callback_error(self, data: Error):
        self.error = data
        self.stabilize_roll(data.roll)
        self.stabilize_pitch(data.pitch)
        self.stabilize_depth(data.depth)
        self.stabilize_yaw(data.yaw)

    # Collect Movement Data
    def callback_movement(self, data: Movement):
        if data.type == 'SURGE':
            self.pwm_surge = data.pwm
        if data.type == 'SWAY':
            self.pwm_sway = data.pwm
        if data.type == 'YAW':
            self.pwm_yaw = data.pwm
        if data.type == 'HEAVE':
            self.pwm_heave = data.pwm
        if data.type == 'ROLL':
            self.pwm_roll = data.pwm
        if data.type == 'PITCH':
            self.pwm_pitch = data.pwm
        if data.type == 'STOP':
            self.movement.stop()

    def stabilize(self):
        self.surge()
        # self.heave_pitch()
        # self.surge()
        

    def callback_is_start(self, data: Bool):
        self.is_pre_calibrating = not data.data

        # Wait for a secs to await sensor value changes / spikes after pre calibrating
        if not self.is_pre_calibrating and self.start_time + self.param_arming_duration > rospy.get_time():
            rospy.loginfo('READY TO DIVE...')
            self.pub_dive.publish(True)
            return
        self.pub_dive.publish(False)

        # Condition for pre calibrating
        if data.data:
            self.stabilize()
        else:
            self.start_time = rospy.get_time()
            self.pre_calibrate()
            self.movement.stop()

        self.movement.publish()
    
    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()
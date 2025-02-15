#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32, Bool, Int8
from robotic_sas_auv_ros.msg import SetPoint, IsStable, Movement, ObjectDifference

class Subscriber():
    def __init__ (self):
        self.is_start = False
        self.boot_time = 0
        self.start_time = 0
        self.bucket = False
        self.bucket_detected = False
        self.flag = 0
        self.yaw = -83 #-13

        self.start = 0
        self.current_time = 0
        self.elapsed_time = 0
        self.delay = 5
        self.start_delay = False

        self.is_stable = IsStable()
        self.set_point = SetPoint()
        self.movement = Movement()
        self.object_difference = ObjectDifference()
        self.filter = 0

        self.set_point.roll = 0 #y
        self.set_point.pitch = 0 #x
        self.set_point.yaw = self.yaw
        self.set_point.depth = -0.45
        self.towards_bucket = False

        self.param_delay = rospy.get_param('/nuc/delay')
        self.param_duration = rospy.get_param('/nuc/duration')

        # Publisher
        self.pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', Movement, queue_size=10)
        self.pub_constrain_pwm = rospy.Publisher('constrain_pwm', Int32, queue_size=10)
        self.pub_move = rospy.Publisher('move', String, queue_size=10)

        # Subscriber
        rospy.Subscriber('/rosserial/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('dive', Bool, self.callback_dive)
        rospy.Subscriber('/rosserial/bucket_detected', Bool, self.callback_bucket)
        rospy.Subscriber('flag', Int8, self.callback_flag)
        rospy.Subscriber('object_difference', ObjectDifference, self.callback_object_difference)

    def set_heading(self, heading):
        # Change yaw set point from the given value
        rospy.loginfo('Set Yaw %s', heading)
        self.set_point.yaw = heading

    def delay_time(self, event):
        self.current_time = rospy.get_time()
        self.elapsed_time = self.current_time - self.start

    def callback_object_difference(self, data: ObjectDifference):
        self.object_difference.object_type = data.object_type
        self.object_difference.x_difference = data.x_difference

    def callback_dive(self, data: Bool):
        self.dive = data

    def callback_flag(self, data: Int8):
        self.flag = data.data
    
    def callback_bucket(self, data: Bool):
        self.bucket = data.data
        if self.bucket:
            self.bucket_detected = True

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def stop_auv(self):
        # Stop AUV
        rospy.loginfo('STOP')
        self.pub_is_start.publish(False)

    def start_auv(self):
        # Stop AUV when the timer reaches 27 secs since pre calibration
        # if self.is_in_range(61, None):
        #     self.stop_auv()
        #     return
        
        # Start AUV mission
        self.pub_set_point.publish(self.set_point)
        self.pub_is_start.publish(True)

        if not self.dive.data:
            # if self.is_in_range(3, 6): # 20 detik
            #     rospy.loginfo("Last")
            #     self.pub_move.publish("last")
            # if self.is_in_range(7, 27):
            #     rospy.loginfo("Forward")
            #     self.pub_move.publish("forward")
            #     self.set_heading(self.yaw)
            # if self.is_in_range(28, 31): # 3 detik
            #     rospy.loginfo("Right")
            #     self.pub_move.publish("right")
            # if self.is_in_range(32, 100) and self.flag != 3: # 20 detik
            #     rospy.loginfo("Forward")
            #     self.pub_move.publish("forward")
            #     self.set_heading(self.yaw)
            # if self.flag == 3 and self.bucket_detected == False:
            #     if self.object_difference.object_type == "Bucket":
            #         rospy.loginfo("Centering Bucket")
            #         self.pub_move.publish("camera")
            #         self.set_point.depth = 0.35
            #         self.towards_bucket = True
            #     elif self.towards_bucket == False:
            #         rospy.loginfo("Search for Bucket")
            #         self.pub_move.publish("yaw_left")

            # if self.is_in_range(0,None) and self.bucket_detected == False:
            #     if self.object_difference.object_type == "Bucket":
            #         rospy.loginfo("Centering Bucket")
            #         self.pub_move.publish("camera")
            #         self.set_point.depth = 0.3
            #         self.towards_bucket = True
            #     elif self.towards_bucket == False:
            #         rospy.loginfo("Search for Bucket")
            #         self.pub_move.publish("yaw_left")

            if self.is_in_range(0, 3): # 20 detik
                rospy.loginfo("Last")
                self.pub_move.publish("last")
            if self.is_in_range(4, None):
                rospy.loginfo("Forward")
                self.pub_move.publish("forward")
                self.set_heading(self.yaw)
            # if self.is_in_range(18, 20): # 3 detik
            #     rospy.loginfo("Right")
            #     self.pub_move.publish("right")
            # if self.is_in_range(21, 100) and self.flag != 3: # 20 detik
            #     rospy.loginfo("Forward")
            #     self.pub_move.publish("forward")
            #     self.set_heading(self.yaw)
            # if self.flag == 3 and self.bucket_detected == False:
            #     if self.object_difference.object_type == "Bucket":
            #         rospy.loginfo("Centering Bucket")
            #         self.pub_move.publish("camera")
            #         self.set_point.depth = 0.35
            #         self.towards_bucket = True
            #     elif self.towards_bucket == False:
            #         rospy.loginfo("Search for Bucket")
            #         self.pub_move.publish("yaw_left")

            # if self.bucket_detected == True:
            #     rospy.loginfo("Surface")
            #     self.pub_move.publish("surface")
            #     if self.start_delay == False:
            #         self.start = rospy.get_time()
            #         rospy.Timer(rospy.Duration(1), self.delay_time, oneshot=False)
            #         self.start_delay = True
            #     elif self.elapsed_time <= self.delay:
            #         pass
            #     else:
            #         self.set_point.depth = 0.05

            if self.is_in_range(6,7):
                self.pub_constrain_pwm.publish(1500)
            if self.is_in_range(7,8): 
                self.pub_constrain_pwm.publish(1490)
            if self.is_in_range(8,9):
                self.pub_constrain_pwm.publish(1480)
            if self.is_in_range(9,10):
                self.pub_constrain_pwm.publish(1470)
            if self.is_in_range(10, 11):
                self.pub_constrain_pwm.publish(1460)
            if self.is_in_range(11,12):
                self.pub_constrain_pwm.publish(1450)
            if self.is_in_range(12,13):
                self.pub_constrain_pwm.publish(1440)
            if self.is_in_range(13,14):
                self.pub_constrain_pwm.publish(1430)
            if self.is_in_range(14,15):
                self.pub_constrain_pwm.publish(1420)
            if self.is_in_range(15,16):
                self.pub_constrain_pwm.publish(1410)
            if self.is_in_range(16,None):
                self.pub_constrain_pwm.publish(1400)
            # if self.is_in_range(17,18):
            #     self.pub_constrain_pwm.publish(1390)
            # if self.is_in_range(18,19):
            #     self.pub_constrain_pwm.publish(1380)
            # if self.is_in_range(19,20):
            #     self.pub_constrain_pwm.publish(1370)
            # if self.is_in_range(20,21):
            #     self.pub_constrain_pwm.publish(1360)
            # if self.is_in_range(21,None):
            #     self.pub_constrain_pwm.publish(1350)
            # if self.is_in_range(22,23):
            #     self.pub_constrain_pwm.publish(1340)
            # if self.is_in_range(23,24):
            #     self.pub_constrain_pwm.publish(1330)
            # if self.is_in_range(24,25):
            #     self.pub_constrain_pwm.publish(1320)
            # if self.is_in_range(25,26):
            #     self.pub_constrain_pwm.publish(1310)
            # if self.is_in_range(26,None):
            #     self.pub_constrain_pwm.publish(1300)
 
    def callback_is_start(self, data: Bool):
        if data.data:
            # Set start time
            if not self.is_start:
                self.start_time = rospy.get_time()
                self.is_start = True

            # Generate boot time
            self.boot_time = rospy.get_time() - self.start_time
            
            # Wait for a secs to tell other nodes (accumulator & control) to calibrate
            if self.boot_time < self.param_delay:
                rospy.loginfo('STARTING...')
                self.pub_is_start.publish(False)
                return

            # Timer condition
            if self.boot_time < self.param_duration if self.param_duration >= 0 else True:
                self.start_auv()
            else:
                self.stop_auv()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
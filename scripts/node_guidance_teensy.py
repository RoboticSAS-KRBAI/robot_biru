#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from robotic_sas_auv_ros.msg import SetPoint, MultiPID, PID

class Subscriber():
    def __init__(self):
        self.is_start = False

        self.start_time_flag = 0  # Mengganti nama self.start untuk menghindari konflik
        self.start_delay = False
        self.delay = 3
        self.boot_time = 0
        self.start_time = 0
        self.current_time = 0
        self.elapsed_time = 0
        self.status = "stop"
        self.status_ssy = "stop"
        self.status_dpr = "stop"
        self.boost = 150

        # Tambahkan flag untuk melacak publikasi status
        self.has_published_dpr_ssy = False
        self.has_published_all = False
        self.has_published_stop = False

        self.set_point = SetPoint()
        self.multi_pid_msg = MultiPID()
        self.set_point.roll = 0 
        self.set_point.pitch = 0 
        self.set_point.yaw = -95
        self.set_point.depth = -0.63

        self.param_delay = 5
        self.param_duration = 0

        # Publisher
        self.pub_multi_pid = rospy.Publisher("PID", MultiPID, queue_size=10)
        self.pub_set_point = rospy.Publisher("SetPoint", SetPoint, queue_size=10)
        self.pub_status = rospy.Publisher("Status", String, queue_size=10)
        self.pub_status_ssy = rospy.Publisher("Status_ssy", String, queue_size=10)
        self.pub_status_dpr = rospy.Publisher("Status_dpr", String, queue_size=10)
        self.pub_boost = rospy.Publisher("boost",Float32,queue_size=10)

        # Create multiple PID messages
        pid_yaw = PID()
        pid_yaw.Kp = 12.0  # Proportional constant for yaw
        pid_yaw.Ki = 0.0   # Integral constant for yaw
        pid_yaw.Kd = 0.0   # Derivative constant for yaw

        pid_pitch = PID()
        pid_pitch.Kp = 2000.0 #1150.0  # Proportional constant for pitch
        pid_pitch.Ki = 0.0    # Integral constant for pitch
        pid_pitch.Kd = 0.0    # Derivative constant for pitch

        pid_roll = PID()
        pid_roll.Kp = 500.0   # Proportional constant for roll
        pid_roll.Ki = 0.0     # Integral constant for roll
        pid_roll.Kd = 0.0     # Derivative constant for roll

        pid_depth = PID()
        pid_depth.Kp = 1700.0  # Proportional constant for depth
        pid_depth.Ki = 0.0     # Integral constant for depth
        pid_depth.Kd = 0.0     # Derivative constant for depth

        # Create MultiPID message and add multiple PID sets
        self.multi_pid_msg.pid_yaw = pid_yaw
        self.multi_pid_msg.pid_pitch = pid_pitch
        self.multi_pid_msg.pid_roll = pid_roll
        self.multi_pid_msg.pid_depth = pid_depth

        self.pub_multi_pid.publish(self.multi_pid_msg)  # Publishing MultiPID
        self.pub_set_point.publish(self.set_point)
        self.pub_status.publish(self.status)

    def delay_time(self):
        self.current_time = rospy.get_time()
        self.elapsed_time = self.current_time - self.start_time_flag

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def start(self):
        if not self.is_start:
            self.start_time = rospy.get_time()
            self.is_start = True

        # Generate boot time
        self.boot_time = rospy.get_time() - self.start_time
            
        # Wait for a secs to tell other nodes (accumulator & control) to calibrate
        if self.boot_time < self.param_delay:
            rospy.loginfo('STARTING...')
            return

        # Timer condition
        if self.param_duration <= 0 or self.boot_time < self.param_duration:
            self.start_auv()
 
    def start_auv(self):
        if not self.has_published_dpr_ssy and self.is_in_range(0, 3):
            rospy.loginfo("Go Depth")
            self.pub_boost.publish(self.boost)
            self.pub_multi_pid.publish(self.multi_pid_msg)  # Publishing MultiPID
            self.pub_set_point.publish(self.set_point)
            self.pub_status.publish("dpr_ssy")
            self.has_published_dpr_ssy = True  # Set flag agar tidak dipublish lagi

        if not self.has_published_all and self.is_in_range(4, 10):
            rospy.loginfo("Go!!!")
            self.pub_status.publish("all")
            self.has_published_all = True  # Set flag agar tidak dipublish lagi

        if not self.has_published_stop and self.is_in_range(11, None):
            rospy.loginfo("stopppp")
            self.pub_status.publish("stop")
            self.has_published_stop = True  # Set flag agar tidak dipublish lagi

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()
    # Loop utama
    rate = rospy.Rate(10)  # Frekuensi loop dalam Hz (misalnya, 10 Hz)
    while not rospy.is_shutdown():
        subscriber.start()
        rate.sleep()  # Tunggu hingga siklus berikutnya

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from robotic_sas_auv_ros.msg import SetPoint, MultiPID, PID

def status_callback(data):
    rospy.loginfo("Status received: %s", data.data)

def set_point_callback(msg):
    rospy.loginfo("SetPoint received - Yaw: %f, Pitch: %f, Roll: %f, Depth: %f", 
                  msg.yaw, msg.pitch, msg.roll, msg.depth)
    
def boost_callback(msg):
    rospy.loginfo("Boost received: %f", msg.data)

def multi_pid_callback(msg):
    # Accessing each individual PID from MultiPID message
    rospy.loginfo(f"Yaw PID received - Kp: {msg.pid_yaw.Kp}, Ki: {msg.pid_yaw.Ki}, Kd: {msg.pid_yaw.Kd}")
    rospy.loginfo(f"Pitch PID received - Kp: {msg.pid_pitch.Kp}, Ki: {msg.pid_pitch.Ki}, Kd: {msg.pid_pitch.Kd}")
    rospy.loginfo(f"Roll PID received - Kp: {msg.pid_roll.Kp}, Ki: {msg.pid_roll.Ki}, Kd: {msg.pid_roll.Kd}")
    rospy.loginfo(f"Depth PID received - Kp: {msg.pid_depth.Kp}, Ki: {msg.pid_depth.Ki}, Kd: {msg.pid_depth.Kd}")

def listener():
    # Inisialisasi node ROS
    rospy.init_node('auv_listener', anonymous=True)
    
    # Inisialisasi subscriber
    rospy.Subscriber("status_msg", String, status_callback)
    rospy.Subscriber("set_point_msg", SetPoint, set_point_callback)
    rospy.Subscriber("pid_msg", MultiPID, multi_pid_callback)
    rospy.Subscriber("boost_msg", Float32, boost_callback)
    
    # ROS akan menunggu hingga node ini dihentikan
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

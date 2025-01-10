#!/usr/bin/env python3
import rospy
from robotic_sas_auv_ros.msg import PID, MultiPID, SetPoint
from std_msgs.msg import String, Float32

def main():
    # Publishers
    pub_multi_pid = rospy.Publisher("PID", MultiPID, queue_size=10)
    pub_set_point = rospy.Publisher("SetPoint", SetPoint, queue_size=10)
    pub_status = rospy.Publisher("Status", String, queue_size=10)
    pub_boost = rospy.Publisher("Boost", Float32, queue_size=10)

    # Allow some time for the publishers to set up
    rospy.sleep(1)

    # Create multiple PID messages
    pid_yaw = PID()
    pid_yaw.Kp = 12.0  # Proportional constant for yaw
    pid_yaw.Ki = 0.0   # Integral constant for yaw
    pid_yaw.Kd = 0.0   # Derivative constant for yaw

    pid_pitch = PID()
    pid_pitch.Kp = 1150.0  # Proportional constant for pitch
    pid_pitch.Ki = 0.0    # Integral constant for pitch
    pid_pitch.Kd = 0.0    # Derivative constant for pitch

    pid_roll = PID()
    pid_roll.Kp = 500.0   # Proportional constant for roll
    # pid_roll.Kp = 0
    pid_roll.Ki = 0.0     # Integral constant for roll
    pid_roll.Kd = 0.0     # Derivative constant for roll

    pid_depth = PID()
    pid_depth.Kp = 1700.0  # Proportional constant for depth
    pid_depth.Ki = 0.0     # Integral constant for depth
    pid_depth.Kd = 0.0     # Derivative constant for depth

    # Create MultiPID message and add multiple PID sets
    multi_pid_msg = MultiPID()
    multi_pid_msg.pid_yaw = pid_yaw
    multi_pid_msg.pid_pitch = pid_pitch
    multi_pid_msg.pid_roll = pid_roll
    multi_pid_msg.pid_depth = pid_depth

    # SetPoint message data
    set_point = SetPoint()
    set_point.yaw = -95
    set_point.pitch = 0
    set_point.roll = 0
    set_point.depth = -0.6

    # Status message data
    status = String()
    status.data = "stop"
    boost = 150

    # Log information
    rospy.loginfo("-------------SEND DATA-------------")
    print("Status", status)
    print("MultiPID", multi_pid_msg)
    print("Set Point", set_point)

    # Publish messages once
    pub_status.publish(status)
    pub_multi_pid.publish(multi_pid_msg)  # Publishing MultiPID
    pub_set_point.publish(set_point)
    pub_boost.publish(boost)
    

if __name__ == '__main__':
    rospy.init_node("pub_teensy", anonymous=True)  # Initialize the ROS node
    main()  # Run the main function

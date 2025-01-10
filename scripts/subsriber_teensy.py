#!/usr/bin/env python3
import rospy
from robotic_sas_auv_ros.msg import Actuator

def callback_actuator_data(data: Actuator):
    # Extract the PWM values from the Actuator message
    thruster_1_pwm = data.thruster_1
    thruster_2_pwm = data.thruster_2
    thruster_3_pwm = data.thruster_3
    thruster_4_pwm = data.thruster_4
    thruster_5_pwm = data.thruster_5
    thruster_6_pwm = data.thruster_6
    thruster_7_pwm = data.thruster_7
    thruster_8_pwm = data.thruster_8

    
    # Log the PWM values of each thruster
    rospy.loginfo("-----------------------------------")
    rospy.loginfo("Thruster 1 PWM: %d", thruster_1_pwm)
    rospy.loginfo("Thruster 2 PWM: %d", thruster_2_pwm)
    rospy.loginfo("Thruster 3 PWM: %d", thruster_3_pwm)
    rospy.loginfo("Thruster 4 PWM: %d", thruster_4_pwm)
    rospy.loginfo("Thruster 5 PWM: %d", thruster_5_pwm)
    rospy.loginfo("Thruster 6 PWM: %d", thruster_6_pwm)
    rospy.loginfo("Thruster 7 PWM: %d", thruster_7_pwm)
    rospy.loginfo("Thruster 8 PWM: %d", thruster_8_pwm)

def main():
    rospy.init_node("sub_actuator", anonymous=True)  # Initialize the ROS node
    rospy.Subscriber("actuator_pwm", Actuator, callback_actuator_data)  # Subscribe to the actuator topic
    rospy.spin()  # Keep the node running and processing incoming messages

if __name__ == "__main__":
    main()

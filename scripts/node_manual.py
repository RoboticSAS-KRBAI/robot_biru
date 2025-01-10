#!/usr/bin/env python3  

import rospy  
from robotic_sas_auv_ros.msg import Manual
from std_msgs.msg import String
from pynput import keyboard  

fx = 0.0  
fy = 0.0  
tau = 0.0 
depth_msg = "surface"  # Initialize depth message

def on_press(key):
    global fy, tau, depth_msg  
    try:
        if key.char == 'w':  
            fy += 1  
        elif key.char == 's':  
            fy -= 1  
        elif key.char == 'a':  
            tau -= 1  
        elif key.char == 'd':  
            tau += 1  
        elif key.char == 'r':
            depth_msg = "surface"
        elif key.char == 't': 
            depth_msg = "depth"  
    except AttributeError:  
        pass  

def main():  
    rospy.init_node('node_manual', anonymous=True)  
    manual_pub = rospy.Publisher("/manual_control", Manual, queue_size=10)  
    depth_pub = rospy.Publisher("/depth_status", String, queue_size=10)
    rate = rospy.Rate(10)  
    manual = Manual()  

    listener = keyboard.Listener(on_press=on_press)  
    listener.start()  

    while not rospy.is_shutdown():  
        manual.fx = fx  
        manual.fy = fy  
        manual.tau = tau  

        depth_pub.publish(depth_msg)
        manual_pub.publish(manual) 
        rospy.loginfo(f"Published: Fx={manual.fx}, Fy={manual.fy}, tau={manual.tau}, Status={depth_msg}")  
        rate.sleep()  
    
    listener.stop()  

if __name__ == '__main__':  
    try:
        main()  
    except rospy.ROSInterruptException:  
        pass  

#!/usr/bin/env python3
import pygame
import rospy
from robotic_sas_auv_ros.msg import Joystick

# Inisialisasi pygame dan joystick
pygame.init()

if pygame.joystick.get_count() == 0:
    print("Joystick tidak terdeteksi!")
    exit(1)

pygame_joystick = pygame.joystick.Joystick(0)
pygame_joystick.init()

print(f"Joystick connected: {pygame_joystick.get_name()}")
print(f"Number of axes: {pygame_joystick.get_numaxes()}")

# Inisialisasi ROS node dan publisher
rospy.init_node("joystick_control")
pub_joystick = rospy.Publisher("Joystick", Joystick, queue_size=10)

# Parameter kecepatan maksimal
batas = 2.0

# ID tombol lingkaran (ganti sesuai hasil deteksi tombol)
circle_button_id = 1   
square_button_id = 3   
triangle_button_id = 2 
cross_button_id = 0

# Buat objek Joystick untuk ROS
joystick = Joystick()
joystick.status = "stop"
joystick.A0 = 0
joystick.A1 = 0
joystick.A3 = 0
joystick.A4 = 0    

def shutdown_callback():
    # Fungsi untuk melakukan pembersihan saat shutdown
    rospy.loginfo("Shutting down...")

# Daftarkan fungsi shutdown pada ROS
rospy.on_shutdown(shutdown_callback)

rate = rospy.Rate(10)  # 10 Hz

try:
    while not rospy.is_shutdown():
        pygame.event.pump()
        
        # Ambil nilai joystick (normalisasi ke -1 hingga 1)
        yaw = pygame_joystick.get_axis(0)  # yaw  joystick kiri
        surge = pygame_joystick.get_axis(1)  # surge joystick kiri
        sway = pygame_joystick.get_axis(3)  # sway joystick kanan
        depth = pygame_joystick.get_axis(4)  # depth joystick kanan
        
        # Cek apakah tombol lingkaran ditekan
        if pygame_joystick.get_button(circle_button_id):
            joystick.status = "stop"
            
        elif pygame_joystick.get_button(triangle_button_id):
            joystick.status = "joystick"
            
        if joystick.status == "stop":
            joystick.A1 = 0
            joystick.A0 = 0
            joystick.A3 = 0
            joystick.A4 = 0
            rospy.loginfo("Robot berhenti.")

        elif joystick.status == "joystick":
            joystick.A1 = max(-batas, min(batas, -surge * batas))
            joystick.A0 = max(-batas, min(batas, yaw * batas))
            joystick.A3 = max(-batas, min(batas, sway * batas))
            joystick.A4 = max(-batas, min(batas, -depth * batas))
            rospy.loginfo(f"Published: A0={joystick.A1:.2f}, A1={joystick.A0:.2f}")
            rospy.loginfo(f"Published: A3={joystick.A3:.2f}, A4={joystick.A4:.2f}")    
            rospy.loginfo("Robot Jalan.")
        
        pub_joystick.publish(joystick)
        rate.sleep()  # Control the loop rate

except KeyboardInterrupt:
    print("Program dihentikan.")
    pygame.quit()  # Menutup pygame dengan baik
    rospy.signal_shutdown("KeyboardInterrupt: Program dihentikan.")

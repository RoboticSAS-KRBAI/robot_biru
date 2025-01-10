import pygame
import rospy
from geometry_msgs.msg import Twist

# Inisialisasi pygame dan joystick
pygame.init()

if pygame.joystick.get_count() == 0:
    print("Joystick tidak terdeteksi!")
    exit(1)

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick connected: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")

# Inisialisasi ROS node dan publisher
rospy.init_node("joystick_control")
pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

# Parameter kecepatan maksimal
max_linear_speed = 2.0
max_angular_speed = 2.0

# ID tombol lingkaran (ganti sesuai hasil deteksi tombol)
circle_button_id = 1  # Misalnya, tombol lingkaran memiliki ID 0 (ganti sesuai pengujian)

try:
    while not rospy.is_shutdown():
        pygame.event.pump()

        # Ambil nilai joystick (normalisasi ke -1 hingga 1)
        x1 = joystick.get_axis(3)  # Sumbu X joystick kiri
        y1 = joystick.get_axis(1)  # Sumbu Y joystick kiri

        # Cek apakah tombol lingkaran ditekan
        if joystick.get_button(circle_button_id):
            # Hentikan robot (linear.x dan angular.z = 0)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            rospy.loginfo("Robot berhenti karena tombol lingkaran ditekan.")
        else:
            # Buat pesan Twist untuk gerakan normal
            twist = Twist()
            twist.linear.x = max(-max_linear_speed, min(max_linear_speed, -y1 * max_linear_speed))
            twist.angular.z = max(-max_angular_speed, min(max_angular_speed, -x1 * max_angular_speed))

            # Publish data ke ROS
            pub.publish(twist)
            rospy.loginfo(f"Published: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

except KeyboardInterrupt:
    print("Program dihentikan.")

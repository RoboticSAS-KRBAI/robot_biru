import pygame
import rospy

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick connected: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")

try:
    while True:
        pygame.event.pump()
        x1 = joystick.get_axis(0)  # Sumbu X joystick kiri
        y1 = joystick.get_axis(1)  # Sumbu Y joystick kiri
        x2 = joystick.get_axis(3)  # Sumbu X joystick kanan
        y2 = joystick.get_axis(4)  # Sumbu Y joystick kanan

        print(f"Left Stick -> X: {x1:.2f}, Y: {-(y1):.2f}")
        print(f"Right Stick -> X: {x2:.2f}, Y: {-(y2):.2f}")
except KeyboardInterrupt:
    print("Program dihentikan.")

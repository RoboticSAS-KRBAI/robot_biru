import numpy as np

class DPRController: # Depth Pitch Roll
    def __init__(self, Lx, Ly):
        self.Lx = Lx
        self.Ly = Ly

        # Control matrix A
        self.A = np.array([
            [ Ly, -Ly,  Ly, -Ly],  # Roll contributions
            [ Lx,  Lx, -Lx, -Lx],  # Pitch contributions 
            [  1,   1,   1,   1]   # Depth contributions
        ])

    def control(self, control_depth, control_pitch, control_roll):
        desired_control = np.array([control_depth, control_pitch, control_roll])
        
        T = np.linalg.pinv(self.A).dot(desired_control)

        return T

dprController = DPRController(0.5, 0.5)
control_roll = 0
control_pitch = 0
control_depth = 0
thrusts = dprController.control(control_roll, control_pitch, control_depth)

print(f'Thruster 5: {thrusts[2]:.2f}')
print(f'Thruster 6: {thrusts[1]:.2f}')
print(f'Thruster 7: {thrusts[0]:.2f}')
print(f'Thruster 8: {thrusts[3]:.2f}')
print(f'PWM Thruster 5: {1500 - thrusts[2]*500:.2f}')
print(f'PWM Thruster 6: {1500 - thrusts[1]*500:.2f}')
print(f'PWM Thruster 7: {1500 - thrusts[0]*500:.2f}')
print(f'PWM Thruster 8: {1500 - thrusts[3]*500:.2f}')
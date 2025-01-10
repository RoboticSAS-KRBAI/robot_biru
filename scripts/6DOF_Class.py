import numpy as np

class ThrusterController:
    def __init__(self, d=1.0):
        # Initialize parameters
        self.d = d
        self.sqrt2 = np.sqrt(2)
        
        # Transformation matrix for 8 thrusters
        self.M = np.array([
            [-1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, -1/self.sqrt2, 0, 0, 0, 0],   # Surge (Fx)
            [1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 0, 0, 0, 0],     # Sway (Fy)
            [d/2, -d/2, -d/2, d/2, 0, 0, 0, 0],                                       # Yaw (τ)
            [1, -1, 1, -1, 0, 0, 0, 0],                                               # Yaw moment contribution
            [0, 0, 0, 0, d/2, -d/2, -d/2, d/2],                                       # Roll (τ_roll)
            [0, 0, 0, 0, 1, -1, 1, -1],                                               # Roll moment contribution
            [0, 0, 0, 0, d/2, -d/2, -d/2, d/2],                                       # Pitch (τ_pitch)
            [0, 0, 0, 0, 1, 1, -1, -1],                                               # Pitch moment contribution
            [0, 0, 0, 0, 1, 1, 1, 1],                                                 # Heave (Fz)
        ])

        # Compute the pseudo-inverse of the transformation matrix
        self.M_pseudo_inv = np.linalg.pinv(self.M)

    def compute_thruster_forces(self, fx=0.0, fy=0.0, fz=0.0, tau_yaw=0.0, tau_roll=0.0, tau_pitch=0.0):
        # Input vector for forces and torques
        input_vector = np.array([fx, fy, 0, tau_yaw, 0, tau_roll, 0, tau_pitch, fz])

        # Compute thrust forces
        F = self.M_pseudo_inv.dot(input_vector)

        # Return thrust forces directly as an array
        return F

# Example usage
controller = ThrusterController(d=1.0)
thrust_values = controller.compute_thruster_forces(fx=0.0, fy=0.0, fz=0.0, tau_yaw=1.0, tau_roll=1.0, tau_pitch=0.0)

# Print all thruster values
print("Nilai Thruster:")
for i, value in enumerate(thrust_values):
    print(f"F{i+1}: {value}")

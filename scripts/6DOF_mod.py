import numpy as np

# Variabel d adalah jarak dari tengah kapal ke titik di mana gaya dorongan diberikan
d = 1.0  # Contoh nilai, sesuaikan dengan konfigurasi kapal Anda
sqrt2 = np.sqrt(2)

# Matriks transformasi yang dimodifikasi untuk 8 thruster
M = np.array([
    [-1/sqrt2, 1/sqrt2, 1/sqrt2, -1/sqrt2, -1/sqrt2, 1/sqrt2, 1/sqrt2, -1/sqrt2],  # Surge (maju/mundur)
    [1/sqrt2, 1/sqrt2, 1/sqrt2, 1/sqrt2, -1/sqrt2, -1/sqrt2, -1/sqrt2, -1/sqrt2],   # Sway (kanan/kiri)
    [d/2, -d/2, -d/2, d/2, -d/2, d/2, d/2, -d/2],                                   # Yaw (putaran)
    [1, -1, 1, -1, 1, -1, 1, -1],                                                  # Roll (miring ke samping)
    [1, -1, 1, -1, -1, 1, -1, 1],                                                  # Pitch (miring ke depan/belakang)
    [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]                                        # Heave (naik/turun)
])

# Inputan gaya dan momen
fx = 1   # Gaya pada sumbu x (surge)
fy = 0    # Gaya pada sumbu y (sway)
fz = 1  # Gaya pada sumbu z (heave)
tau_yaw = 0 # Momen yaw
tau_roll = 0  # Momen roll
tau_pitch = 0 # Momen pitch

# Vektor input
input_vector = np.array([fx, fy, fz, tau_yaw, tau_roll, tau_pitch])

# Menghitung pseudo-invers dari Matriks M
M_pseudo_inv = np.linalg.pinv(M)

# Menghitung gaya dorong dari thruster
F = M_pseudo_inv.dot(input_vector)

# Output nilai thruster
print("Nilai Thruster:")
for i in range(len(F)):
    if i < 4:
        print(f"F{i+1}: {1500 - (F[i] * 500)}")
    else:
        print(f"F{i+1}: {1500 + (F[i] * 500)}")

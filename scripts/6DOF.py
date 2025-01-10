import numpy as np

# Variabel d adalah jarak dari tengah kapal ke titik di mana gaya dorongan diberikan
d = 1.0  # Contoh nilai, sesuaikan dengan konfigurasi kapal Anda
sqrt2 = np.sqrt(2)

# Matriks transformasi untuk 8 thruster
M = np.array([
    [-1/sqrt2, 1/sqrt2, 1/sqrt2, -1/sqrt2, 0, 0, 0, 0],   # Surge (maju/mundur)
    [1/sqrt2, 1/sqrt2, 1/sqrt2, 1/sqrt2, 0, 0, 0, 0],     # Sway (kanan/kiri)
    [d/2, -d/2, -d/2, d/2, 0, 0, 0, 0],                   # Yaw (putaran)
    [1, -1, 1, -1, 0, 0, 0, 0],                           # Kontribusi untuk momen yaw
    [0, 0, 0, 0, d/2, -d/2, -d/2, d/2],           # Roll (miring ke samping)
    [0, 0, 0, 0, 1, -1, 1, -1],                   # Kontribusi untuk momen roll
    [0, 0, 0, 0, d/2, -d/2, -d/2, d/2],           # Pitch (miring ke depan/belakang)
    [0, 0, 0, 0, 1, 1, -1, -1],                   # Kontribusi untuk momen pitch
    [0, 0, 0, 0, 1, 1, 1, 1],                     # Heave (naik/turun)

])

# Inputan gaya dan momen
fx = 0.0
fy = 1.0
fz = 0.0
tau_yaw = 0
tau_roll = 0.0
tau_pitch = 1.0

# Vektor input
input_vector = np.array([fx, fy, 0, tau_yaw, 0, fz, 0, tau_pitch, tau_roll])

# Menghitung pseudo-invers dari Matriks M
M_pseudo_inv = np.linalg.pinv(M)

# Menghitung gaya dorong dari thruster
F = M_pseudo_inv.dot(input_vector)

# thrust 5 = 7
# thrust 6 = 6
# thrust 7 = 5
# thrust 8 = 8

# Depth Semua Sama
# Roll  5 7 sama 6 8 sama
# Pitch 5 6 sama 7 8 sama

# Output nilai thruster
print("Nilai Thruster:")
for i in range(len(F)):
    if i < 4:
        print(f"F{i+1}: {1500 - (F[i]*500)}")
    elif i == 4 or i == 6:
        print(i+1)
        print(f"F{i+1}: {1500 + (F[i]*500)}")
    elif i == 5 or i == 7:
        print(i+1)
        print(f"F{i+1}: {1500 - (F[i]*500)}")

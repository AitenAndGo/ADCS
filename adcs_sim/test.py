from sim import simulate
from config import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # potrzebne dla 3D

# Run main sim
times, positions, velocities = simulate(SIMULATION_TIME, SIMULATION_TIMESTEP)

# Tworzenie wykresu
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')


# Wykres Ziemi jako kula
R_EARTH = 6378.12  # km
u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
x = R_EARTH * np.outer(np.cos(u), np.sin(v))
y = R_EARTH * np.outer(np.sin(u), np.sin(v))
z = R_EARTH * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x, y, z, color='blue')

# Wykres orbity
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Orbita satelity', color='red')

# Ustawienia osi
ax.set_title("Orbita satelity w 3D")
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.legend()
ax.set_box_aspect([1, 1, 1])  # Równe proporcje osi
plt.show()
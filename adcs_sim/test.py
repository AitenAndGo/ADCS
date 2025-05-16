from sim import simulate
from config import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # potrzebne dla 3D
import numpy as np
from utils import quaternion_2_euler

# Run main sim
# simulate now returns both ECI and body magnetic fields
times, positions, velocities, quaternions, omegas, magnetic_fields_eci_raw, magnetic_fields_body_raw = simulate(SIMULATION_TIME, SIMULATION_TIMESTEP)

# Convert lists to numpy arrays for easier manipulation
times = np.array(times)
positions = np.array(positions)
velocities = np.array(velocities)
quaternions = np.array(quaternions)
omegas = np.array(omegas)
magnetic_fields_eci = np.array(magnetic_fields_eci_raw)
magnetic_fields_body = np.array(magnetic_fields_body_raw)


# 1. Create 3D plot of orbit
fig_orbit = plt.figure(figsize=(10, 8))
ax_orbit = fig_orbit.add_subplot(111, projection='3d')

# Earth as a sphere
# Use R_EARTH from config (which is in km) directly for consistency with position units
u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
x_earth = R_EARTH * np.outer(np.cos(u), np.sin(v))
y_earth = R_EARTH * np.outer(np.sin(u), np.sin(v))
z_earth = R_EARTH * np.outer(np.ones(np.size(u)), np.cos(v))
ax_orbit.plot_surface(x_earth, y_earth, z_earth, color='blue', alpha=0.3)

# Orbit plot (positions are in km from sim.py output)
ax_orbit.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='satelite orbit', color='red')

ax_orbit.set_title("Satelite Orbit in ECI Frame")
ax_orbit.set_xlabel("X (km)")
ax_orbit.set_ylabel("Y (km)")
ax_orbit.set_zlabel("Z (km)")
ax_orbit.legend()


# 2. Plot velocities
plt.figure(figsize=(12, 6))
plt.plot(times, velocities[:, 0], label='Vx (km/s)') # Units from sim.py
plt.plot(times, velocities[:, 1], label='Vy (km/s)')
plt.plot(times, velocities[:, 2], label='Vz (km/s)')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/s)')
plt.title('Satellite Velocity Components (ECI Frame)')
plt.legend()
plt.grid(True)

# 3. Convert quaternions to Euler angles and plot
euler_angles = np.array([quaternion_2_euler(q) for q in quaternions])
plt.figure(figsize=(12, 6))
plt.plot(times, np.degrees(euler_angles[:, 0]), label='Roll (φ)')
plt.plot(times, np.degrees(euler_angles[:, 1]), label='Pitch (θ)')
plt.plot(times, np.degrees(euler_angles[:, 2]), label='Yaw (ψ)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Satellite Attitude (Euler Angles)')
plt.legend()
plt.grid(True)

# 4. Plot angular velocities
plt.figure(figsize=(12, 6))
plt.plot(times, np.degrees(omegas[:, 0]), label='ωx (deg/s)')
plt.plot(times, np.degrees(omegas[:, 1]), label='ωy (deg/s)')
plt.plot(times, np.degrees(omegas[:, 2]), label='ωz (deg/s)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.title('Satellite Angular Velocity Components (Body Frame)')
plt.legend()
plt.grid(True)

# 5. Plot Magnetic Field in ECI frame 
plt.figure(figsize=(12, 6))
plt.plot(times, magnetic_fields_eci[:, 0], label='Bx (ECI)')
plt.plot(times, magnetic_fields_eci[:, 1], label='By (ECI)')
plt.plot(times, magnetic_fields_eci[:, 2], label='Bz (ECI)')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Field (Tesla)')
plt.title('Magnetic Field Components (ECI Frame)')
plt.legend()
plt.grid(True)

# 6. Plot Magnetic Field in Body frame
plt.figure(figsize=(12, 6))
plt.plot(times, magnetic_fields_body[:, 0], label='Bx (Body)')
plt.plot(times, magnetic_fields_body[:, 1], label='By (Body)')
plt.plot(times, magnetic_fields_body[:, 2], label='Bz (Body)')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Field (Tesla)')
plt.title('Magnetic Field Components (Body Frame)')
plt.legend()
plt.grid(True)

plt.show()

# --- Launch Windows Visualization App ---
import subprocess
import os # For path manipulation if needed, though direct path often works
import pandas as pd

print("\nAttempting to launch Windows visualization application...")

# IMPORTANT: Replace this with the correct path to your .exe from WSL's perspective
# Example: "/mnt/c/Users/YourWindowsUser/Documents/visualizationApp/your_app_name.exe"
# If the visualizationApp folder is in the root of C:, it might be "/mnt/c/visualizationApp/your_app_name.exe"
# Get the user's home directory from environment variable
import os
from pathlib import Path


# Change this path to the correct path to your folder with the SateliteSim.exe file
# Base path to the application folder
APP_BASE_PATH = Path("/mnt/c/Users/Barto/UNITY/SateliteSim/Build/v02")
windows_app_path = str(APP_BASE_PATH / "SateliteSim.exe")

# StreamingAssets folder is relative to the app location
StreamingAssetsPath = str(APP_BASE_PATH / "SateliteSim_Data/StreamingAssets/")

# Create StreamingAssets directory if it doesn't exist
os.makedirs(StreamingAssetsPath, exist_ok=True)

# Orientacja
orientation_df = pd.DataFrame(quaternions, columns=["w", "x", "y", "z"])
orientation_df.insert(0, "time", times)
orientation_df.to_csv(f"{StreamingAssetsPath}orientation_data.csv", index=False)

# Pozycja
position_df = pd.DataFrame(positions, columns=["x", "y", "z"])
position_df.insert(0, "time", times)
position_df.to_csv(f"{StreamingAssetsPath}translation_data.csv", index=False)


try:
    print(f"Executing: {windows_app_path}")
    # Popen launches the application without waiting for it to complete.
    # This is generally suitable for GUI applications.
    subprocess.Popen(windows_app_path)
    print(f"Launched '{windows_app_path}'. Check for its window.")
    print("Python script will now exit if all plots are closed.")

except FileNotFoundError:
    print(f"Error: The application at '{windows_app_path}' was not found.")
    print("Please ensure the path is correct and the application exists at that location from WSL's perspective.")
    print("Typical WSL path for C drive is /mnt/c/...")
except Exception as e:
    print(f"An error occurred while trying to launch the application: {e}")
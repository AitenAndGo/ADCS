# ==============================================
# main.py
# Main entry point for ADCS simulation and visualization
# ----------------------------------------------
# - Runs the satellite attitude and orbit simulation
# - Plots results: orbit, attitude, angular velocity, control torques, sensor data
# - Exports orientation and position data for external visualization
# - Optionally launches a Windows visualization app (Unity-based)
# ==============================================

from sim import simulate
from config import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting
import numpy as np
from utils import quaternion_2_euler

# --- Run the main simulation ---
# simulate() returns a dictionary with all relevant simulation data
sim_data = simulate(SIMULATION_TIME, SIMULATION_TIMESTEP)

# --- Unpack simulation data ---
times = sim_data["times"]
positions = sim_data["positions"]
velocities = sim_data["velocities"]
quaternions = sim_data["quaternions"]
omegas = sim_data["omegas"]
magnetic_fields_body = sim_data["magnetic_fields_body"]
control_torques = sim_data["control_torques"]
noisy_gyros = sim_data["noisy_gyros"]
noisy_mags = sim_data["noisy_mags"]
filtered_gyros = sim_data["filtered_gyros"]
filtered_mags = sim_data["filtered_mags"]

# Convert lists to numpy arrays for easier manipulation
# (ensures compatibility with plotting and data export)
times = np.array(times)
positions = np.array(positions)
velocities = np.array(velocities)
quaternions = np.array(quaternions)
omegas = np.array(omegas)
magnetic_fields_body = np.array(magnetic_fields_body)

# --- 1. 3D Plot of Orbit ---
fig_orbit = plt.figure(figsize=(10, 8))
ax_orbit = fig_orbit.add_subplot(111, projection='3d')

# Plot Earth as a sphere (for reference)
# Use R_EARTH from config (in km) for consistency with position units
u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
x_earth = R_EARTH * np.outer(np.cos(u), np.sin(v))
y_earth = R_EARTH * np.outer(np.sin(u), np.sin(v))
z_earth = R_EARTH * np.outer(np.ones(np.size(u)), np.cos(v))
ax_orbit.plot_surface(x_earth, y_earth, z_earth, color='blue', alpha=0.3)

# Plot satellite orbit trajectory
ax_orbit.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='satelite orbit', color='red')

ax_orbit.set_title("Satelite Orbit in ECI Frame")
ax_orbit.set_xlabel("X (km)")
ax_orbit.set_ylabel("Y (km)")
ax_orbit.set_zlabel("Z (km)")
ax_orbit.legend()

# --- 2. Plot Satellite Attitude (True Euler Angles) ---
euler_true = np.array([quaternion_2_euler(q) for q in quaternions])

plt.figure(figsize=(12, 8))
# Roll
plt.subplot(3, 1, 1)
plt.plot(times, np.degrees(euler_true[:, 0]), label='True Roll')
plt.ylabel('Roll (deg)')
plt.legend()
plt.grid(True)
# Pitch
plt.subplot(3, 1, 2)
plt.plot(times, np.degrees(euler_true[:, 1]), label='True Pitch')
plt.ylabel('Pitch (deg)')
plt.legend()
plt.grid(True)
# Yaw
plt.subplot(3, 1, 3)
plt.plot(times, np.degrees(euler_true[:, 2]), label='True Yaw')
plt.ylabel('Yaw (deg)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)
plt.suptitle('Satellite Attitude: True Euler Angles')

# --- 4. Plot Angular Velocities (shows damping) ---
plt.figure(figsize=(12, 6))
plt.plot(times, np.degrees(omegas[:, 0]), label='ωx (deg/s)')
plt.plot(times, np.degrees(omegas[:, 1]), label='ωy (deg/s)')
plt.plot(times, np.degrees(omegas[:, 2]), label='ωz (deg/s)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.title('Satellite Angular Velocity (Damped by B-dot)')
plt.legend()
plt.grid(True)

# --- 5. Plot Control Torques ---
plt.figure(figsize=(12, 6))
plt.plot(times, control_torques[:, 0], label='Tx')
plt.plot(times, control_torques[:, 1], label='Ty')
plt.plot(times, control_torques[:, 2], label='Tz')
plt.xlabel('Time (s)')
plt.ylabel('Torque (N·m)')
plt.title('B-dot Control Torques')
plt.legend()
plt.grid(True)

# --- 6. Plot Commanded Magnetic Moments ---
commanded_moments = sim_data["commanded_moments"]
plt.figure(figsize=(12, 6))
plt.plot(times[:], commanded_moments[:, 0], label='mx')
plt.plot(times[:], commanded_moments[:, 1], label='my')
plt.plot(times[:], commanded_moments[:, 2], label='mz')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Moment (A·m²)')
plt.title('Commanded Magnetic Moment from B-dot Controller')
plt.legend()
plt.grid(True)

# --- 7. Overlay: Angular Velocities (True, Measured, Filtered) ---
plt.figure(figsize=(12, 8))
for i, axis in enumerate(['x', 'y', 'z']):
    plt.subplot(3, 1, i+1)
    plt.plot(times[:], np.degrees(omegas[:, i]), label=f'True ω{axis}')
    plt.plot(times[:-1], np.degrees(noisy_gyros[:, i]), label=f'Measured ω{axis}', linestyle='--')
    plt.plot(times[:-1], np.degrees(filtered_gyros[:, i]), label=f'Filtered ω{axis}', linestyle=':')
    plt.ylabel(f'ω{axis} (deg/s)')
    plt.legend()
    plt.grid(True)
plt.xlabel('Time (s)')
plt.suptitle('Angular Velocity: True, Measured, and Filtered (Gyroscope)')

# --- 8. Overlay: Magnetometer (True, Measured, Filtered for B-dot) ---
plt.figure(figsize=(12, 8))
for i, axis in enumerate(['x', 'y', 'z']):
    plt.subplot(3, 1, i+1)
    plt.plot(times[:-1], magnetic_fields_body[:-1, i]*1e6, label=f'True B{axis} (μT)')
    plt.plot(times[:-1], noisy_mags[:, i]*1e6, label=f'Measured B{axis} (μT)', linestyle='--')
    plt.plot(times[:-1], filtered_mags[:, i]*1e6, label=f'Filtered B{axis} (μT)', linestyle=':')
    plt.ylabel(f'B{axis} (μT)')
    plt.legend()
    plt.grid(True)
plt.xlabel('Time (s)')
plt.suptitle('Magnetometer: True, Measured, and Filtered (for B-dot)')

# --- Show all plots ---
plt.show()

# --- Export Data for Windows Visualization App (Unity) ---
import subprocess
import os # For path manipulation if needed, though direct path often works
import pandas as pd

print("\nAttempting to launch Windows visualization application...")

# IMPORTANT: Replace this with the correct path to your .exe
import os
from pathlib import Path

# Change this path to the correct path to your folder with the SateliteSim.exe file
# Base path to the application folder
APP_BASE_PATH = Path("../v04")
windows_app_path = str(APP_BASE_PATH / "SateliteSim.exe")

# StreamingAssets folder is relative to the app location
StreamingAssetsPath = str(APP_BASE_PATH / "SateliteSim_Data/StreamingAssets/")

# Create StreamingAssets directory if it doesn't exist
os.makedirs(StreamingAssetsPath, exist_ok=True)

# --- Export orientation (quaternion) data to CSV ---
orientation_df = pd.DataFrame(quaternions, columns=["w", "x", "y", "z"])
orientation_df.insert(0, "time", times)
orientation_df.to_csv( f"{StreamingAssetsPath}/orientation_data.csv", index=False)

# --- Export position (translation) data to CSV ---
position_df = pd.DataFrame(positions, columns=["x", "y", "z"])
position_df.insert(0, "time", times)
position_df.to_csv(f"{StreamingAssetsPath}/translation_data.csv", index=False)

# --- Launch the Unity-based visualization app (if available) ---
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

from sim import simulate
from config import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # potrzebne dla 3D
import numpy as np
from utils import quaternion_2_euler, quaternion_multiply, normalize_quaternion
from sensors import read_gyroscope, read_magnetometer
from controller import bdot_feedback_loop

# Run main sim
# simulate now returns both ECI and body magnetic fields
sim_data = simulate(SIMULATION_TIME, SIMULATION_TIMESTEP)

# Unpack simulation data
times = sim_data["times"]
positions = sim_data["positions"]
velocities = sim_data["velocities"]
quaternions = sim_data["quaternions"]
omegas = sim_data["omegas"]
magnetic_fields_body = sim_data["magnetic_fields_body"]
estimated_quaternions = sim_data["estimated_quaternions"]
control_torques = sim_data["control_torques"]

# Convert lists to numpy arrays for easier manipulation
times = np.array(times)
positions = np.array(positions)
velocities = np.array(velocities)
quaternions = np.array(quaternions)
omegas = np.array(omegas)
magnetic_fields_body = np.array(magnetic_fields_body)
estimated_quaternions = np.array(estimated_quaternions)
control_torques = np.array(control_torques)

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

# 2. Plot satellite attitude (True vs. Estimated)
euler_true = np.array([quaternion_2_euler(q) for q in quaternions])
euler_est = np.array([quaternion_2_euler(q) for q in estimated_quaternions])

plt.figure(figsize=(12, 8))
# Roll
plt.subplot(3, 1, 1)
plt.plot(times, np.degrees(euler_true[:, 0]), label='True Roll', color='black')
plt.plot(times, np.degrees(euler_est[:, 0]), label='Estimated Roll', linestyle='--', color='red')
plt.ylabel('Roll (deg)')
plt.legend()
plt.grid(True)
# Pitch
plt.subplot(3, 1, 2)
plt.plot(times, np.degrees(euler_true[:, 1]), label='True Pitch', color='black')
plt.plot(times, np.degrees(euler_est[:, 1]), label='Estimated Pitch', linestyle='--', color='green')
plt.ylabel('Pitch (deg)')
plt.legend()
plt.grid(True)
# Yaw
plt.subplot(3, 1, 3)
plt.plot(times, np.degrees(euler_true[:, 2]), label='True Yaw', color='black')
plt.plot(times, np.degrees(euler_est[:, 2]), label='Estimated Yaw', linestyle='--', color='blue')
plt.ylabel('Yaw (deg)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)
plt.suptitle('Attitude Estimation: True vs. Filtered')

# 3. Plot Attitude Estimation Error
q_conj = quaternions.copy()
q_conj[:, 1:] *= -1
error_quats = np.array([quaternion_multiply(q_est, q_true_conj) for q_est, q_true_conj in zip(estimated_quaternions, q_conj)])
error_angles = np.array([2 * np.arccos(np.clip(q[0], -1, 1)) for q in error_quats])

plt.figure(figsize=(12, 6))
plt.plot(times, np.degrees(error_angles), label='Attitude Error')
plt.xlabel('Time (s)')
plt.ylabel('Error (degrees)')
plt.title('Attitude Estimation Error (Angle between True and Estimated)')
plt.legend()
plt.grid(True)

# 4. Plot Angular Velocities (shows damping)
plt.figure(figsize=(12, 6))
plt.plot(times, np.degrees(omegas[:, 0]), label='ωx (deg/s)')
plt.plot(times, np.degrees(omegas[:, 1]), label='ωy (deg/s)')
plt.plot(times, np.degrees(omegas[:, 2]), label='ωz (deg/s)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.title('Satellite Angular Velocity (Damped by B-dot)')
plt.legend()
plt.grid(True)

# 5. Plot Control Torques
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
plt.plot(times, commanded_moments[:-1, 0], label='mx')
plt.plot(times, commanded_moments[:-1, 1], label='my')
plt.plot(times, commanded_moments[:-1, 2], label='mz')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Moment (A·m²)')
plt.title('Commanded Magnetic Moment from B-dot Controller')
plt.legend()
plt.grid(True)

# moje--- Simulate noisy sensor readings ---
gyro_state = {}
mag_state = {}
noisy_gyros = []
noisy_mags = []
gyro_biases = []
mag_biases = []

for i in range(len(times)):
    # True values
    omega_true = omegas[i]  # [rad/s] (body frame)
    B_body_true = magnetic_fields_body[i]  # [Tesla] (body frame)
    dt = SIMULATION_TIMESTEP
    # Gyroscope
    omega_meas, gyro_state = read_gyroscope(omega_true, dt, gyro_state)
    noisy_gyros.append(omega_meas)
    gyro_biases.append(gyro_state['gyro_bias'].copy())
    # Magnetometer
    B_meas, mag_state = read_magnetometer(B_body_true, dt, mag_state)
    noisy_mags.append(B_meas)
    mag_biases.append(mag_state['mag_bias'].copy())

noisy_gyros = np.array(noisy_gyros)
noisy_mags = np.array(noisy_mags)
gyro_biases = np.array(gyro_biases)
mag_biases = np.array(mag_biases)

# --- Plot Gyroscope Readings ---
plt.figure(figsize=(12, 6))
plt.plot(times, np.degrees(omegas[:, 0]), label='True ωx (deg/s)', color='black', linestyle='--')
plt.plot(times, np.degrees(noisy_gyros[:, 0]), label='Measured ωx (deg/s)', color='red', alpha=0.7)
plt.plot(times, np.degrees(gyro_biases[:, 0]), label='Gyro Bias x (deg/s)', color='red', linestyle=':')
plt.plot(times, np.degrees(omegas[:, 1]), label='True ωy (deg/s)', color='black', linestyle='--')
plt.plot(times, np.degrees(noisy_gyros[:, 1]), label='Measured ωy (deg/s)', color='green', alpha=0.7)
plt.plot(times, np.degrees(gyro_biases[:, 1]), label='Gyro Bias y (deg/s)', color='green', linestyle=':')
plt.plot(times, np.degrees(omegas[:, 2]), label='True ωz (deg/s)', color='black', linestyle='--')
plt.plot(times, np.degrees(noisy_gyros[:, 2]), label='Measured ωz (deg/s)', color='blue', alpha=0.7)
plt.plot(times, np.degrees(gyro_biases[:, 2]), label='Gyro Bias z (deg/s)', color='blue', linestyle=':')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.title('Gyroscope: True vs Measured Readings (with Bias)')
plt.legend(ncol=2)
plt.grid(True)

# --- Plot Magnetometer Readings ---
plt.figure(figsize=(12, 6))
plt.plot(times, magnetic_fields_body[:, 0]*1e6, label='True Bx (μT)', color='black', linestyle='--')
plt.plot(times, noisy_mags[:, 0]*1e6, label='Measured Bx (μT)', color='red', alpha=0.7)
plt.plot(times, mag_biases[:, 0]*1e6, label='Mag Bias x (μT)', color='red', linestyle=':')
plt.plot(times, magnetic_fields_body[:, 1]*1e6, label='True By (μT)', color='black', linestyle='--')
plt.plot(times, noisy_mags[:, 1]*1e6, label='Measured By (μT)', color='green', alpha=0.7)
plt.plot(times, mag_biases[:, 1]*1e6, label='Mag Bias y (μT)', color='green', linestyle=':')
plt.plot(times, magnetic_fields_body[:, 2]*1e6, label='True Bz (μT)', color='black', linestyle='--')
plt.plot(times, noisy_mags[:, 2]*1e6, label='Measured Bz (μT)', color='blue', alpha=0.7)
plt.plot(times, mag_biases[:, 2]*1e6, label='Mag Bias z (μT)', color='blue', linestyle=':')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Field (μT)')
plt.title('Magnetometer: True vs Measured Readings (with Bias)')
plt.legend(ncol=2)
plt.grid(True)

# --- Test bdot_feedback_loop function ---
print("\n=== Testing bdot_feedback_loop function ===")

# Test with simple sinusoidal magnetic field
test_times = np.linspace(0, 10, 100)
test_B_filtered = []
test_m_out = []

for t in test_times:
    # Create a simple sinusoidal magnetic field for testing
    B_test = np.array([1e-5 * np.sin(t), 1e-5 * np.cos(t), 1e-5])
    test_B_filtered.append(B_test)
    
    # Apply B-dot feedback loop
    m_out = bdot_feedback_loop(B_test)
    test_m_out.append(m_out)

test_B_filtered = np.array(test_B_filtered)
test_m_out = np.array(test_m_out)

# Plot test results
plt.figure(figsize=(12, 8))

# Plot input B_filtered
plt.subplot(2, 1, 1)
plt.plot(test_times, test_B_filtered[:, 0]*1e6, label='Bx (μT)', color='red')
plt.plot(test_times, test_B_filtered[:, 1]*1e6, label='By (μT)', color='green')
plt.plot(test_times, test_B_filtered[:, 2]*1e6, label='Bz (μT)', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Field (μT)')
plt.title('Test Input: Filtered Magnetic Field')
plt.legend()
plt.grid(True)

# Plot output m_out
plt.subplot(2, 1, 2)
plt.plot(test_times, test_m_out[:, 0], label='mx (A·m²)', color='red')
plt.plot(test_times, test_m_out[:, 1], label='my (A·m²)', color='green')
plt.plot(test_times, test_m_out[:, 2], label='mz (A·m²)', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Magnetic Moment (A·m²)')
plt.title('Test Output: B-dot Control Commands')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.suptitle('bdot_feedback_loop Function Test', y=1.02)

print("✓ bdot_feedback_loop test completed!")
print(f"  - Input: {len(test_B_filtered)} magnetic field samples")
print(f"  - Output: {len(test_m_out)} control commands")
print(f"  - First command: {test_m_out[0]}")
print(f"  - Last command: {test_m_out[-1]}")

#tu sie konczy moje
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

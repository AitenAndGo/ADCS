# sensors.py
# Simulated onboard sensor data
# ----------------------------------------------
# - Simulates readings from gyroscopes and magnetometers
# - Adds realistic noise, bias, and drift
# - Used for testing attitude estimation and control
# ==============================================


import numpy as np
from config import MAG_DISTURBANCE_GAIN


# =============================
# Sensor noise, bias, and drift parameters (typical for CubeSat-class sensors)
# =============================
# Gyroscope
GYRO_NOISE_STD = 0.001   # [rad/s] white noise std dev


# Magnetometer
MAG_NOISE_STD = 5e-7     # [Tesla] white noise std dev (50 nT)

def read_gyroscope(omega_true, dt, state):
    """
    Simulate gyroscope reading with noise, bias, and drift.
    Args:
        omega_true: true angular velocity [wx, wy, wz] (rad/s)
        dt: timestep (s)
        state: dict holding persistent bias for the gyroscope
    Returns:
        omega_meas: measured angular velocity (rad/s)
        state: updated state dict
    """
    if 'gyro_bias' not in state:
        # Initialize bias as a random vector
        state['gyro_bias'] = np.random.normal(0, 1, 3)
    # Bias random walk (drift)
    state['gyro_bias'] += np.random.normal(0, 1 * np.sqrt(dt), 3)
    # White noise
    noise = np.random.normal(0, GYRO_NOISE_STD, 3)
    omega_meas = omega_true + state['gyro_bias'] + noise
    return omega_meas, state


def read_magnetometer(magnetic_field_body, m_out, dt, state):
    """
    Simulate magnetometer reading with noise, bias, drift, and magnetorquer disturbance.
    Args:
        magnetic_field_body: true magnetic field in body frame [Bx, By, Bz] (Tesla)
        m_out: commanded magnetic moment from controller [mx, my, mz] (A·m²)
        dt: timestep (s)
        state: dict holding persistent bias for the magnetometer
    Returns:
        B_meas: measured magnetic field (Tesla)
        state: updated state dict
    """
    # Add disturbance from magnetorquers
    B_disturbance = MAG_DISTURBANCE_GAIN * m_out
    B_total_true = magnetic_field_body + B_disturbance      
    
    # White noise
    noise = np.random.normal(0, MAG_NOISE_STD, 3)
    B_meas = B_total_true + noise
    return B_meas, state


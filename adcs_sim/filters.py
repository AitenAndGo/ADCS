# ==============================================
# filters.py
# Attitude estimation filters (optional)
# ----------------------------------------------
# - Complementary, Madgwick, or Kalman filters
# - Combines sensor data to estimate satellite attitude
# ==============================================
import numpy as np
from utils import quaternion_to_rotation_matrix, normalize_quaternion, quaternion_multiply, euler_2_quaternion
from sensors import read_magnetometer, read_gyroscope

# ==============================================
# Complementary Filter for Attitude Estimation
# ==============================================

def complementary_filter_update(q_est, omega_meas, B_meas, magnetic_field_eci, dt, alpha=0.98):
    """
    Update attitude estimate and derive filtered sensor values.
    Args:
        q_est: current attitude estimate quaternion [q0, q1, q2, q3]
        omega_meas: measured angular velocity [wx, wy, wz] (rad/s)
        B_meas: measured magnetic field in body frame [Bx, By, Bz] (Tesla)
        magnetic_field_eci: reference magnetic field in inertial frame [Bx, By, Bz] (Tesla)
        dt: timestep (s)
        alpha: filter blending factor (0 < alpha < 1), higher = trust gyro more
    Returns:
        A dictionary containing:
        'q_est': updated attitude estimate quaternion
        'omega_filtered': filtered angular velocity (rad/s)
        'B_filtered': filtered magnetic field in body frame (Tesla)
    """
    # --- 1. Gyro integration (predict step) ---
    omega = np.array(omega_meas)
    omega_norm = np.linalg.norm(omega)
    if omega_norm > 1e-8:
        theta = omega_norm * dt
        axis = omega / omega_norm
        dq = np.hstack([
            np.cos(theta/2),
            axis * np.sin(theta/2)
        ])
    else:
        dq = np.array([1.0, 0.0, 0.0, 0.0])
    q_gyro = normalize_quaternion(quaternion_multiply(q_est, dq))

    # --- 2. Magnetometer correction (measurement step) ---
    # Estimate yaw from measured and reference magnetic field
    # (Assume B_ref is in inertial frame, B_meas in body frame)
    # Project both to horizontal plane (ignore Z)
    B_meas_xy = np.array([B_meas[0], B_meas[1], 0])
    B_ref_xy = np.array([magnetic_field_eci[0], magnetic_field_eci[1], 0])
    yaw_meas = np.arctan2(B_meas_xy[1], B_meas_xy[0])
    yaw_ref = np.arctan2(B_ref_xy[1], B_ref_xy[0])
    yaw_err = yaw_meas - yaw_ref
    # Correction quaternion (about Z axis)
    dq_corr = euler_2_quaternion(0, 0, -yaw_err)
    q_mag = normalize_quaternion(quaternion_multiply(q_gyro, dq_corr))

    # --- 3. Complementary filter blend ---
    dot = np.dot(q_gyro, q_mag)
    if dot < 0.0:
        q_mag = -q_mag
        dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        q_new = normalize_quaternion((1 - alpha) * q_gyro + alpha * q_mag)
    else:
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        s0 = np.sin((1 - alpha) * theta_0) / sin_theta_0
        s1 = np.sin(alpha * theta_0) / sin_theta_0
        q_new = normalize_quaternion(s0 * q_gyro + s1 * q_mag)
    
    # --- 4. Derive Filtered Outputs ---
    # The filtered magnetic field is our best estimate of the true field,
    # found by rotating the reference ECI field by our new attitude estimate.
    R_est = quaternion_to_rotation_matrix(q_new)
    B_filtered = R_est @ magnetic_field_eci
    
    # For a simple complementary filter, the "filtered" omega is the raw measurement,
    # as the filter does not estimate gyroscope bias.
    omega_filtered = omega_meas

    return {
        'q_est': q_new,
        'omega_filtered': omega_filtered,
        'B_filtered': B_filtered
    }

# Example usage in a simulation loop:
# q_est = np.array([1,0,0,0])
# for each timestep:
#     q_est = complementary_filter_update(q_est, omega_meas, B_meas, B_ref, dt)

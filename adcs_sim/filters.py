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

def kalman_filter_update(q_est, omega_meas, B_meas, magnetic_field_eci, dt, state_dict=None):
    """
    Extended Kalman Filter for attitude estimation with gyro bias estimation.
    
    State vector: [q0, q1, q2, q3, bx, by, bz] where:
    - q0, q1, q2, q3: attitude quaternion
    - bx, by, bz: gyroscope bias (rad/s)
    
    Args:
        q_est: current attitude estimate quaternion [q0, q1, q2, q3]
        omega_meas: measured angular velocity [wx, wy, wz] (rad/s)
        B_meas: measured magnetic field in body frame [Bx, By, Bz] (Tesla)
        magnetic_field_eci: reference magnetic field in inertial frame [Bx, By, Bz] (Tesla)
        dt: timestep (s)
        state_dict: dictionary containing filter state (P, bias_est)
    Returns:
        A dictionary containing:
        'q_est': updated attitude estimate quaternion
        'omega_filtered': filtered angular velocity (rad/s)
        'B_filtered': filtered magnetic field in body frame (Tesla)
        'bias_est': estimated gyroscope bias (rad/s)
        'P': updated covariance matrix
    """
    
    # Initialize state if not provided
    if state_dict is None:
        state_dict = {
            'P': np.eye(7) * 0.1,  # Initial covariance matrix
            'bias_est': np.array([0.0, 0.0, 0.0])  # Initial bias estimate
        }
    
    P = state_dict['P']
    bias_est = state_dict['bias_est']
    
    # --- 1. Predict Step (Time Update) ---
    
    # Correct angular velocity with bias estimate
    omega_corrected = omega_meas - bias_est
    
    # Quaternion integration
    omega_norm = np.linalg.norm(omega_corrected)
    if omega_norm > 1e-8:
        theta = omega_norm * dt
        axis = omega_corrected / omega_norm
        dq = np.hstack([
            np.cos(theta/2),
            axis * np.sin(theta/2)
        ])
    else:
        dq = np.array([1.0, 0.0, 0.0, 0.0])
    
    q_pred = normalize_quaternion(quaternion_multiply(q_est, dq))
    
    # State transition matrix F (simplified - assumes constant bias)
    F = np.eye(7)
    F[:4, :4] = np.eye(4)  # Quaternion part
    F[4:, 4:] = np.eye(3)  # Bias part (assumed constant)
    
    # Process noise covariance Q
    Q = np.eye(7) * 1e-6
    Q[:4, :4] *= 1e-4  # Quaternion process noise
    Q[4:, 4:] *= 1e-8  # Bias process noise
    
    # Predict covariance
    P_pred = F @ P @ F.T + Q
    
    # --- 2. Update Step (Measurement Update) ---
    
    # Measurement vector: [Bx_meas, By_meas, Bz_meas]
    z = B_meas
    
    # Predicted measurement (rotate ECI field to body frame)
    R_pred = quaternion_to_rotation_matrix(q_pred)
    B_pred = R_pred @ magnetic_field_eci
    
    # Measurement residual
    y = z - B_pred
    
    # Measurement matrix H (Jacobian of measurement model)
    # H relates changes in state to changes in measurement
    H = np.zeros((3, 7))
    
    # Partial derivatives of B_pred with respect to quaternion
    # This is a simplified linearization
    B_eci = magnetic_field_eci
    H[:3, :4] = np.array([
        [0, 2*B_eci[0], 2*B_eci[1], 2*B_eci[2]],
        [0, -2*B_eci[1], 2*B_eci[0], 0],
        [0, -2*B_eci[2], 0, 2*B_eci[0]]
    ]) * 0.5  # Simplified quaternion derivative
    
    # No direct measurement of bias
    H[:3, 4:] = np.zeros((3, 3))
    
    # Measurement noise covariance R
    R = np.eye(3) * 1e-10  # Magnetometer noise (Tesla^2)
    
    # Kalman gain
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    # State update
    state_update = K @ y
    q_update = state_update[:4]
    bias_update = state_update[4:]
    
    # Update quaternion (additive update for small corrections)
    q_new = normalize_quaternion(q_pred + q_update)
    
    # Update bias estimate
    bias_new = bias_est + bias_update
    
    # Update covariance
    I = np.eye(7)
    P_new = (I - K @ H) @ P_pred
    
    # --- 3. Derive Filtered Outputs ---
    
    # Filtered angular velocity (corrected for bias)
    omega_filtered = omega_meas - bias_new
    
    # Filtered magnetic field (best estimate)
    R_est = quaternion_to_rotation_matrix(q_new)
    B_filtered = R_est @ magnetic_field_eci
    
    # Update state dictionary
    state_dict['P'] = P_new
    state_dict['bias_est'] = bias_new
    
    return {
        'q_est': q_new,
        'omega_filtered': omega_filtered,
        'B_filtered': B_filtered,
        'bias_est': bias_new,
        'P': P_new
    }

# ==============================================
# Legacy Complementary Filter (kept for reference)
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
        'B_filtered_complementary': B_filtered_complementary
    }

# Example usage in a simulation loop:
# q_est = np.array([1,0,0,0])
# for each timestep:
#     q_est = complementary_filter_update(q_est, omega_meas, B_meas, B_ref, dt)

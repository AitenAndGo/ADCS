# ==============================================
# filters.py
# Attitude estimation filters (optional)
# ----------------------------------------------
# This module provides algorithms for estimating the satellite's attitude
# by combining sensor data from gyroscopes and magnetometers.
#
# - Implements a simple complementary filter for sensor fusion
# - Placeholder for advanced filters (e.g., Madgwick, Kalman)
# - Used to provide filtered estimates of angular velocity and magnetic field
# ==============================================


# ==============================================
# Complementary Filter for Attitude Estimation
# ==============================================

def complementary_filter_update(prev_filtered_gyro, prev_filtered_mag, omega_meas, B_meas, alpha=0.9):
    """
    Updates the filtered estimates of angular velocity and magnetic field using a complementary filter.
    The complementary filter blends the previous filtered value with the new sensor measurement
    to reduce noise while maintaining responsiveness.

    Args:
        prev_filtered_gyro (np.ndarray): Previous filtered angular velocity (rad/s)
        prev_filtered_mag (np.ndarray): Previous filtered magnetic field (Tesla)
        omega_meas (np.ndarray): New gyroscope measurement (rad/s)
        B_meas (np.ndarray): New magnetometer measurement (Tesla)
        alpha (float, optional): Filter coefficient (0 < alpha < 1). Higher alpha favors previous value (more smoothing).

    Returns:
        B_filtered (np.ndarray): Updated filtered magnetic field (Tesla)
        omega_filtered (np.ndarray): Updated filtered angular velocity (rad/s)
    """
    B_filtered = alpha * prev_filtered_mag + (1 - alpha) * B_meas
    omega_filtered = alpha * prev_filtered_gyro + (1 - alpha) * omega_meas
    return B_filtered, omega_filtered


# ==============================================
# utils.py
# Utility functions and helpers
# ----------------------------------------------
# - Quaternion math, vector normalization, conversions
# - Coordinate frame transformations
# - Time helpers and general-purpose tools
# ==============================================
import numpy as np

def euler_2_quaternion(phi, theta, psi):
    """
    Converts Euler angles (in radians) to a quaternion.

    Parameters:
    - phi: Roll angle (rotation around x-axis)
    - theta: Pitch angle (rotation around y-axis)
    - psi: Yaw angle (rotation around z-axis)

    Returns:
    - quaternion: [q0, q1, q2, q3], where q0 is the scalar part
    """
    q0 = np.cos(phi/2) * np.cos(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    q1 = np.sin(phi/2) * np.cos(theta/2) * np.cos(psi/2) - np.cos(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    q2 = np.cos(phi/2) * np.sin(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.cos(theta/2) * np.sin(psi/2)
    q3 = np.cos(phi/2) * np.cos(theta/2) * np.sin(psi/2) - np.sin(phi/2) * np.sin(theta/2) * np.cos(psi/2)

    quaternion = [q0, q1, q2, q3]

    return quaternion


def quaternion_2_euler(quaternion):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw) in radians.
    Assumes ZYX rotation sequence (psi, theta, phi).

    Parameters:
    - quaternion: list or NumPy array [q0, q1, q2, q3], where q0 is the scalar part.

    Returns:
    - euler: NumPy array [phi, theta, psi] corresponding to [roll, pitch, yaw] in radians.
    """
    q0, q1, q2, q3 = quaternion

    # Roll (phi, x-axis rotation)
    phi = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))

    # Pitch (theta, y-axis rotation)
    # Clip argument to arcsin to be within [-1, 1] for numerical stability
    asin_input = np.clip(2 * (q0 * q2 - q3 * q1), -1.0, 1.0)
    theta = np.arcsin(asin_input)

    # Yaw (psi, z-axis rotation)
    psi = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))

    return np.array([phi, theta, psi])

def rk4_step(f, y, dt, *args):
    """Generic RK4 integrator"""
    k1 = f(y, *args)
    k2 = f(y + 0.5 * dt * k1, *args)
    k3 = f(y + 0.5 * dt * k2, *args)
    k4 = f(y + dt * k3, *args)
    return y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

def quaternion_to_rotation_matrix(quaternion):
    """
    Convert a quaternion to a rotation matrix.
    
    Parameters:
    - quaternion: [q0, q1, q2, q3] where q0 is the scalar component
    
    Returns:
    - R: 3x3 rotation matrix
    """
    q0, q1, q2, q3 = quaternion
    
    R = np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])
    
    return R

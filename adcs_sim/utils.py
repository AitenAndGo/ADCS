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


def quaternion_2_euler(q0, q1, q2, q3):
    """
    Converts a quaternion to Euler angles (in radians).

    Parameters:
    - q0, q1, q2, q3: Quaternion components (q0 is the scalar part)

    Returns:
    - euler: [phi, theta, psi] = [roll, pitch, yaw]
    """
    # Roll (x-axis rotation)
    phi = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))

    # Pitch (y-axis rotation)
    theta = np.arcsin(2 * (q0 * q2 - q3 * q1))

    # Yaw (z-axis rotation)
    psi = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))

    euler = [phi, theta, psi]

    return euler

def rk4_step(f, y, dt, *args):
    """Generic RK4 integrator"""
    k1 = f(y, *args)
    k2 = f(y + 0.5 * dt * k1, *args)
    k3 = f(y + 0.5 * dt * k2, *args)
    k4 = f(y + dt * k3, *args)
    return y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

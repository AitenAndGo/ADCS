# ==============================================
# utils.py
# Utility functions and helpers
# ----------------------------------------------
# This module provides general-purpose utility functions for the ADCS simulation,
# including quaternion math, vector normalization, coordinate frame conversions,
# and numerical integration routines.
#
# - Quaternion math, vector normalization, conversions
# - Coordinate frame transformations
# - Time helpers and general-purpose tools
# ==============================================
import numpy as np

def euler_2_quaternion(phi, theta, psi):
    """
    Converts Euler angles (in radians) to a quaternion.

    Parameters:
        phi (float): Roll angle (rotation around x-axis)
        theta (float): Pitch angle (rotation around y-axis)
        psi (float): Yaw angle (rotation around z-axis)

    Returns:
        quaternion (list): [q0, q1, q2, q3], where q0 is the scalar part
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
        quaternion (list or np.ndarray): [q0, q1, q2, q3], where q0 is the scalar part.

    Returns:
        euler (np.ndarray): [phi, theta, psi] corresponding to [roll, pitch, yaw] in radians.
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
    """
    Generic Runge-Kutta 4th order (RK4) integrator for ODEs.

    Parameters:
        f (callable): Function that computes the derivative (dy/dt)
        y (np.ndarray): Current state vector
        dt (float): Time step
        *args: Additional arguments to pass to f
    Returns:
        y_new (np.ndarray): State vector after one RK4 step
    """
    k1 = f(y, *args)
    k2 = f(y + 0.5 * dt * k1, *args)
    k3 = f(y + 0.5 * dt * k2, *args)
    k4 = f(y + dt * k3, *args)
    return y + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def quaternion_to_rotation_matrix(quaternion):
    """
    Convert a quaternion to a 3x3 rotation matrix.
    The quaternion should be in [q0, q1, q2, q3] format, with q0 as the scalar part.
    
    Parameters:
        quaternion (list or np.ndarray): [q0, q1, q2, q3] where q0 is the scalar component
    Returns:
        R (np.ndarray): 3x3 rotation matrix
    """
    q0, q1, q2, q3 = quaternion
    
    R = np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])
    
    return R


def normalize_quaternion(q):
    """
    Normalize a quaternion to unit length.

    Parameters:
        q (list or np.ndarray): Quaternion [q0, q1, q2, q3]
    Returns:
        q_normalized (np.ndarray): Unit quaternion
    """
    q = np.array(q)
    return q / np.linalg.norm(q)


def quaternion_multiply(q1, q2):
    """
    Hamilton product of two quaternions.
    Computes q = q1 * q2.

    Parameters:
        q1 (list or np.ndarray): First quaternion [q0, q1, q2, q3] (scalar first)
        q2 (list or np.ndarray): Second quaternion [q0, q1, q2, q3] (scalar first)
    Returns:
        q (np.ndarray): Product quaternion [q0, q1, q2, q3]
    """
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])


# ==============================================
# attitude.py
# Attitude dynamics and kinematics model
# ----------------------------------------------
# - Uses quaternions for rotation representation
# - Simulates attitude propagation using Euler's equations
# - Computes angular velocity and orientation changes
# ==============================================

import numpy as np
from config import INERTIA, INERTIA_INV
from utils import rk4_step


def quaternion_derivative(q, omega):
    """
    Computes the time derivative of a quaternion given angular velocity.
    
    Parameters:
    - q: Quaternion [q0, q1, q2, q3] (unit quaternion representing orientation)
    - omega: Angular velocity vector [wx, wy, wz] in rad/s

    Returns:
    - dq/dt: Quaternion derivative based on current angular velocity
    """
    wx, wy, wz = omega

    # Quaternion kinematics matrix (based on Hamilton product rules)
    omega_mat = 0.5 * np.array([
        [0,   -wx, -wy, -wz],
        [wx,   0,   wz, -wy],
        [wy,  -wz,  0,   wx],
        [wz,   wy, -wx,  0]
    ])

    return omega_mat @ q  # dq/dt = 0.5 * Omega_matrix * q


def angular_acceleration(omega, torque, inertia, inertia_inv):
    """
    Computes angular acceleration using Euler's rotational equations.

    Parameters:
    - omega: Angular velocity vector [wx, wy, wz] in rad/s
    - torque: Applied torque vector [tx, ty, tz] in N·m
    - inertia: Inertia matrix (3x3) of the satellite
    - inertia_inv: Inverse of the inertia matrix

    Returns:
    - Angular acceleration vector [αx, αy, αz] in rad/s²
    """
    # Angular momentum: H = I * ω
    H = inertia @ omega

    # Cross product term: ω × H
    cross_term = np.cross(omega, H)

    # Euler’s equation: I * α = torque - ω × (I * ω)
    return inertia_inv @ (torque - cross_term)


def update_rotation(quaternion, omega, torque, dt):
    """
    Updates the satellite's orientation and angular velocity over a time step.

    Parameters:
    - quaternion: Current orientation quaternion [q0, q1, q2, q3]
    - omega: Current angular velocity vector [wx, wy, wz] in rad/s
    - torque: Control or environmental torque vector [tx, ty, tz] in N·m
    - dt: Time step in seconds

    Returns:
    - q_new: Updated and normalized quaternion after one step
    - omega_new: Updated angular velocity vector after one step
    """
    # Convert inputs to numpy arrays for consistency
    q = np.array(quaternion)
    omega = np.array(omega)

    # Integrate quaternion dynamics using Runge-Kutta 4th order method
    q_new = rk4_step(quaternion_derivative, q, dt, omega)

    # Normalize quaternion to avoid drift due to numerical errors
    q_new /= np.linalg.norm(q_new)

    # Integrate angular velocity dynamics using Runge-Kutta 4th order method
    omega_new = rk4_step(
        angular_acceleration, omega, dt, torque, INERTIA, INERTIA_INV
    )

    return q_new, omega_new
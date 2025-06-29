# ==============================================
# orbit.py
# Satellite orbit model
# ----------------------------------------------
# This module provides functions to simulate the satellite's orbit around Earth
# using the two-body problem (Keplerian motion) and numerical integration.
#
# - Calculates position and velocity relative to Earth
# - Supports simplified orbital dynamics or Keplerian motion
# - Can be extended for more complex perturbations or force models
# ==============================================

import numpy as np
from scipy.integrate import solve_ivp
from config import MU_EARTH, G, SATELITE_MASS, M_EARTH
from utils import rk4_step


def two_body_problem(state):
    """
    Computes the derivative of the state vector for two-body orbital motion.
    This function models the satellite's motion under the influence of Earth's gravity only
    (no perturbations, drag, or third-body effects).
    
    Parameters:
        state (array-like): State vector [x, y, z, vx, vy, vz]
            - x, y, z: Position components in km
            - vx, vy, vz: Velocity components in km/s
    Returns:
        dstate_dt (np.ndarray): Derivative of the state vector [vx, vy, vz, ax, ay, az]
            - vx, vy, vz: Velocity components
            - ax, ay, az: Acceleration components due to gravity
    """
    r = state[0:3]  # Position vector [x, y, z]
    v = state[3:6]  # Velocity vector [vx, vy, vz]
    norm_r = np.linalg.norm(r)  # Magnitude of the position vector (distance to Earth)
    
    # Calculate gravitational force (in Newtons) and acceleration
    Fgrav = (-G * SATELITE_MASS * M_EARTH / norm_r**2) * (r / norm_r)
    a = Fgrav / SATELITE_MASS  # Acceleration: F = ma, so a = F/m
    
    return np.concatenate((v, a))  # Concatenate velocity and acceleration to return as a vector


def simulate_orbit(initial_position, initial_velocity, times):
    """
    Simulates the satellite's orbit using the two-body problem and numerical integration.
    Uses scipy's solve_ivp to integrate the equations of motion over the specified time span.
    
    Parameters:
        initial_position (array-like): Initial position vector [x, y, z] in km
        initial_velocity (array-like): Initial velocity vector [vx, vy, vz] in km/s
        times (array-like): Time points at which to evaluate the orbit (in seconds)
    Returns:
        t (np.ndarray): Time points (seconds)
        rvec (np.ndarray): Position vectors [x, y, z] at each time step in km
        vvec (np.ndarray): Velocity vectors [vx, vy, vz] at each time step in km/s
    """
    # Combine initial position and velocity into the initial state vector
    initial_state = [initial_position[0], initial_position[1], initial_position[2],
                     initial_velocity[0], initial_velocity[1], initial_velocity[2]]

    print(initial_state)  # Debug: Print initial state vector

    # Solve the two-body orbital dynamics using the `solve_ivp` ODE solver
    solution = solve_ivp(
        fun=two_body_problem,           # The function that computes the derivatives
        method='RK45',                  # Runge-Kutta method for ODE integration
        t_span=(times[0], times[-1]),   # Time span from first to last time point
        y0=initial_state,               # Initial state vector [position, velocity]
        t_eval=times,                   # Time points at which to evaluate the solution
        rtol=1e-9,                      # Relative tolerance for integration
        atol=1e-9                       # Absolute tolerance for integration
    )

    # Extract position and velocity vectors from the solution
    rvec = solution.y[:3].T     # Position vectors (x, y, z) at each time step
    vvec = solution.y[3:6].T    # Velocity vectors (vx, vy, vz) at each time step

    return solution.t, rvec, vvec  # Return time, position, and velocity


def update_orbit(position, velocity, dt):
    """
    Updates the satellite's position and velocity using the Runge-Kutta 4th order (RK4) method.
    This function is typically called at each simulation time step for real-time propagation.
    
    Parameters:
        position (array-like): Current position vector [x, y, z] in km
        velocity (array-like): Current velocity vector [vx, vy, vz] in km/s
        dt (float): Time step (in seconds) for integration
    Returns:
        position_new (np.ndarray): Updated position vector after one time step in km
        velocity_new (np.ndarray): Updated velocity vector after one time step in km/s
    """
    state=[position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]]

    state_new = rk4_step(two_body_problem, state, dt)
    position_new = state_new[:3]
    velocity_new = state_new[3:]

    return position_new, velocity_new  # Return the updated position and velocity
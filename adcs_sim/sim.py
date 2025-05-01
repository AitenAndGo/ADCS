# ==============================================
# sim.py
# Main simulation loop for ADCS
# ----------------------------------------------
# - Integrates orbit, attitude, magnetic field, and sensor models
# - Applies control algorithms and filtering (if enabled)
# - Returns data for visualization
# ==============================================

import numpy as np
from config import INITIAL_POSITION, INITIAL_VELOCITY, INITIAL_ROTATION_QUATERNION, INITIAL_ANGULAR_VELOCITY, INITIAL_TORQUE
from orbit import simulate_orbit, update_orbit
from attitude import update_rotation

def calculate_orbit(duration, time_step):
    """
    Computes the satellite's full orbit over the specified duration.

    Parameters:
    - duration: Total simulation time in seconds
    - time_step: Time step in seconds

    Returns:
    - times: Array of time values
    - positions: Array of position vectors [x, y, z] over time
    - velocities: Array of velocity vectors [vx, vy, vz] over time
    """
    times = np.arange(0, duration + time_step, time_step)

    times, positions, velocities = simulate_orbit(
        INITIAL_POSITION,
        INITIAL_VELOCITY,
        times
    )

    return times, positions, velocities


def simulate(duration, time_step):
    """
    Runs the full ADCS simulation, including orbit and attitude propagation.

    Parameters:
    - duration: Total simulation time in seconds
    - time_step: Time step in seconds

    Returns:
    - times: Array of time values
    - positions: List of position vectors [x, y, z] for each time step
    - velocities: List of velocity vectors [vx, vy, vz] for each time step
    - quaternions: List of orientation quaternions for each time step
    - omegas: List of angular velocity vectors [wx, wy, wz] for each time step
    """
    times = np.arange(0, duration + time_step, time_step)

    # Lists to store simulation states over time
    positions = [INITIAL_POSITION]
    velocities = [INITIAL_VELOCITY]
    quaternions = [INITIAL_ROTATION_QUATERNION]
    omegas = [INITIAL_ANGULAR_VELOCITY]
    torque = [INITIAL_TORQUE]

    for t in times:
        # --- Translational Dynamics (Orbit Propagation) ---
        # Update satellite position and velocity
        newPos, newVel = update_orbit(positions[-1], velocities[-1], time_step)
        positions.append(newPos)
        velocities.append(newVel)

        # --- Rotational Dynamics (Attitude Propagation) ---
        # Update satellite orientation and angular velocity
        newRot, newOmega = update_rotation(quaternions[-1], omegas[-1], torque[-1], time_step)
        quaternions.append(newRot)
        omegas.append(newOmega)

    return times, positions, velocities, quaternions, omegas

# ==============================================
# sim.py
# Main ADCS simulation loop
# ----------------------------------------------
# - Integrates orbit and attitude dynamics
# - Calls sensor models and control algorithms (future)
# ==============================================

import numpy as np
from config import INITIAL_POSITION, INITIAL_VELOCITY, INITIAL_ROTATION_QUATERNION, INITIAL_ANGULAR_VELOCITY, INITIAL_TORQUE
from orbit import simulate_orbit, update_orbit
from attitude import update_rotation
from magnetic_field import get_magnetic_field_readings

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
    - positions: List of position vectors [x, y, z] for each time step (km)
    - velocities: List of velocity vectors [vx, vy, vz] for each time step (km/s)
    - quaternions: List of orientation quaternions for each time step
    - omegas: List of angular velocity vectors [wx, wy, wz] for each time step (rad/s)
    - magnetic_fields_eci: List of ECI magnetic field vectors [Bx, By, Bz] (Tesla)
    - magnetic_fields_body: List of body frame magnetic field vectors [Bx, By, Bz] (Tesla)
    """
    times = np.arange(0, duration + time_step, time_step)
    
    # Initialize state lists
    positions = [np.array(INITIAL_POSITION)]  # in km
    velocities = [np.array(INITIAL_VELOCITY)]  # in km/s
    quaternions = [np.array(INITIAL_ROTATION_QUATERNION)]
    omegas = [np.array(INITIAL_ANGULAR_VELOCITY)]
    torque = [np.array(INITIAL_TORQUE)] # Assuming torque might change, keep as list for now

    # Initialize lists for magnetic field data
    # Initial call with t=0 for the simulation epoch start
    initial_magnetic_field_eci, initial_magnetic_field_body = get_magnetic_field_readings(positions[0], quaternions[0], 0) 
    magnetic_fields_eci = [initial_magnetic_field_eci]
    magnetic_fields_body = [initial_magnetic_field_body]

    # --- Simulation Loop ---
    total_steps = len(times) - 1
    print("Starting simulation...") # Initial message
    for i, t in enumerate(times[:-1]): # Iterate up to the second to last time step
        current_pos_km = positions[-1]
        current_vel_km_s = velocities[-1]
        current_rot = quaternions[-1]
        current_omega = omegas[-1]
        current_torque = torque[-1] # Using the last torque, can be updated by a controller

        # --- Orbit Propagation ---
        # Ensure units are consistent (km and km/s for update_orbit)
        newPos_km, newVel_km_s = update_orbit(current_pos_km, current_vel_km_s, time_step)
        positions.append(newPos_km)
        velocities.append(newVel_km_s)

        # --- Attitude Propagation ---
        # Ensure units are consistent for update_rotation
        newRot, newOmega = update_rotation(current_rot, current_omega, current_torque, time_step)
        quaternions.append(newRot)
        omegas.append(newOmega)
        
        # --- Magnetic Field Readings ---
        # Calculate magnetic field in ECI and body frame using current simulation time `t`
        # Note: t is the time at the *start* of the current interval. 
        # For consistency, we use the newPos and newRot calculated for time t+time_step,
        # but the time for the IGRF model should correspond to the actual point in time.
        # Using times[i+1] which is the time at the end of the current step.
        mag_field_eci, mag_field_body = get_magnetic_field_readings(newPos_km, newRot, times[i+1])
        magnetic_fields_eci.append(mag_field_eci)
        magnetic_fields_body.append(mag_field_body)

        # Print progress (e.g., every 1% or every few steps)
        if total_steps > 0: # Avoid division by zero for very short simulations
            progress_percent = (i + 1) / total_steps * 100
            # Update progress every 100 steps or if it's a significant percentage jump or the last step
            if (i + 1) % 100 == 0 or (i + 1) == total_steps or progress_percent % 5 == 0:
                print(f"\rSimulation Progress: {progress_percent:.2f}%", end="")

    print("\nSimulation complete!") # Final message
    return (
        times, 
        positions, 
        velocities, 
        quaternions, 
        omegas, 
        magnetic_fields_eci,  # Ensure ECI field is returned
        magnetic_fields_body
    )

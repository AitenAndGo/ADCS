import numpy as np
from config import INITIAL_POSITION, INITIAL_VELOCITY, INITIAL_ROTATION_QUATERNION, INITIAL_ANGULAR_VELOCITY
from orbit import update_orbit
from attitude import update_rotation
from magnetic_field import get_magnetic_field_readings
from sensors import read_gyroscope, read_magnetometer
from filters import complementary_filter_update
from controller import get_control_torque, b_dot_controller

def simulate(duration, time_step):
    """
    Runs the full closed-loop ADCS simulation.
    """
    times = np.arange(0, duration + time_step, time_step)
    
    # Initialize true state lists
    positions = [np.array(INITIAL_POSITION)]
    velocities = [np.array(INITIAL_VELOCITY)]
    quaternions = [np.array(INITIAL_ROTATION_QUATERNION)]
    omegas = [np.array(INITIAL_ANGULAR_VELOCITY)]
    
    # Initialize sensor and filter states
    gyro_state, mag_state = {}, {}
    q_est = np.array(INITIAL_ROTATION_QUATERNION)
    m_command_prev = np.array([0, 0, 0])  # Initial command is zero
    
    # Initialize data logging lists
    magnetic_fields_eci = []
    magnetic_fields_body = []
    noisy_gyros = []
    noisy_mags = []
    filtered_gyros = []
    filtered_mags = []
    estimated_quaternions = [q_est.copy()]
    control_torques = [np.array([0, 0, 0])]
    commanded_moments = [m_command_prev.copy()]
    # Initialize previous filtered values as first noisy readings
    prev_filtered_gyro = None
    prev_filtered_mag = None
    bdot_state = {}

    # --- Simulation Loop ---
    print("Starting closed-loop simulation...")
    for i, t in enumerate(times[:-1]):
        current_pos = positions[-1]
        current_vel = velocities[-1]
        current_quat = quaternions[-1]
        current_omega = omegas[-1]
        
        # --- 1. Get True Magnetic Field ---
        B_eci_true, B_body_true = get_magnetic_field_readings(current_pos, current_quat, t)
        magnetic_fields_eci.append(B_eci_true)
        magnetic_fields_body.append(B_body_true)
        
        # --- 2. Sensor Simulation (uses command from previous step) ---
        omega_meas, gyro_state = read_gyroscope(current_omega, time_step, gyro_state)
        B_meas, mag_state = read_magnetometer(B_body_true, m_command_prev, time_step, mag_state)
        noisy_gyros.append(omega_meas)
        noisy_mags.append(B_meas)

        # --- 3. Simple Complementary Filter (filter noisy sensors) ---
        if i == 0:
            prev_filtered_gyro = omega_meas
            prev_filtered_mag = B_meas
        filter_output = complementary_filter_update(prev_filtered_gyro, prev_filtered_mag, omega_meas, B_meas)
        omega_filtered = filter_output['filtered_gyro']
        B_filtered = filter_output['filtered_mag']
        filtered_gyros.append(omega_filtered)
        filtered_mags.append(B_filtered)
        prev_filtered_gyro = omega_filtered
        prev_filtered_mag = B_filtered

        # --- 4. B-dot Control Law (use filtered magnetic field) ---
        m_out, bdot_state = b_dot_controller(B_filtered, omega_filtered, bdot_state, dt=time_step)
        commanded_moments.append(m_out.copy())
        m_command_prev = m_out.copy()

        # --- 5. Actuator Model (compute torque) ---
        current_torque = get_control_torque(m_out, B_body_true)
        control_torques.append(current_torque)

        # --- 6. Dynamics Propagation ---
        new_pos, new_vel = update_orbit(current_pos, current_vel, time_step)
        new_quat, new_omega = update_rotation(current_quat, current_omega, current_torque, time_step)
        
        positions.append(new_pos)
        velocities.append(new_vel)
        quaternions.append(new_quat)
        omegas.append(new_omega)
        
        # Print progress
        progress_percent = (i + 1) / (len(times) -1) * 100
        if (i + 1) % 100 == 0 or (i + 1) == len(times) - 1:
            print(f"\rSimulation Progress: {progress_percent:.2f}%", end="")

    # Final magnetic field reading for the last point
    B_eci_true, B_body_true = get_magnetic_field_readings(positions[-1], quaternions[-1], times[-1])
    magnetic_fields_eci.append(B_eci_true)
    magnetic_fields_body.append(B_body_true)

    print("\nSimulation complete!")
    return {
        "times": times, 
        "positions": np.array(positions), 
        "velocities": np.array(velocities), 
        "quaternions": np.array(quaternions), 
        "omegas": np.array(omegas), 
        "magnetic_fields_eci": np.array(magnetic_fields_eci),
        "magnetic_fields_body": np.array(magnetic_fields_body),
        "noisy_gyros": np.array(noisy_gyros),
        "noisy_mags": np.array(noisy_mags),
        "filtered_gyros": np.array(filtered_gyros),
        "filtered_mags": np.array(filtered_mags),
        "control_torques": np.array(control_torques),
        "commanded_moments": np.array(commanded_moments)
    }

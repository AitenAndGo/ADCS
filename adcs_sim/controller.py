# ==============================================
# controller.py
# Attitude control algorithms
# ----------------------------------------------
# - Implements B-dot control logic
# - Placeholder for additional controllers (e.g., PD, quaternion feedback)
# - Outputs control torques for actuators (e.g., magnetorquers)
# - Replace read_magnetic_field() with real I2C/SPI BNO086 read function (e.g., via pyserial, Adafruit_BNO08x, etc.).
# ==============================================

import time
import numpy as np
from utils import quaternion_to_rotation_matrix
from config import K_BDOT, SIMULATION_TIMESTEP
from filters import complementary_filter_update
from sensors import read_gyroscope, read_magnetometer
from magnetic_field import get_magnetic_field_readings
from config import INITIAL_ROTATION_QUATERNION, INITIAL_ANGULAR_VELOCITY, INITIAL_POSITION
# === B-dot Controller Parameters ===
       # [s] — control loop time step


# === B-dot Controller ===
def b_dot_controller(B_curr, state, k=K_BDOT, dt=SIMULATION_TIMESTEP):
    """
    Calculates the magnetic moment for B-dot control using internal state.
    Args:
        B_curr: current filtered magnetic field vector (Tesla)
        state: dict holding the controller's persistent state (e.g., B_prev)
        k: B-dot gain
        dt: timestep
    Returns:
        m_out: commanded magnetic moment (A·m²)
        state: updated state dict
    """
    B_prev = state.get('B_prev', None)

    if B_prev is None:
        # Cannot compute derivative on the first run
        m_out = np.array([0, 0, 0])
    else:
        B_dot = (B_curr - B_prev) / dt
        m_out = -k * B_dot
    
    # Update state for the next iteration
    state['B_prev'] = B_curr.copy()
    
    return m_out, state

def get_control_torque(m_out, B_filtered):
    """
    Calculates the torque produced by magnetorquers.
    Args:
        m_out: commanded magnetic moment (A·m²)
        B_filtered: filtered magnetic field in body frame (Tesla)
    Returns:
        torque: resulting control torque (N·m)
    """
    return np.cross(m_out, B_filtered)

# === Simple B-dot Feedback Loop ===
def bdot_feedback_loop(B_filtered):
    """
    Simple B-dot feedback loop that takes only B_filtered as input.
    Implements the B-dot algorithm: m_out = -K * dB/dt
    
    Args:
        B_filtered: filtered magnetic field vector from filters.py (Tesla)
    Returns:
        m_out: magnetic moment for magnetorquers (A·m²)
    """
    # Initialize controller state (persistent between calls)
    if not hasattr(bdot_feedback_loop, 'state'):
        bdot_feedback_loop.state = {}
    
    # Get current B_filtered
    B_curr = np.array(B_filtered)
    
    # Apply B-dot algorithm
    m_out, bdot_feedback_loop.state = b_dot_controller(B_curr, bdot_feedback_loop.state)
    
    return m_out

#nie sprawdziłem od tąd
# === Main Control Loop (Realistic Simulation) ===
def run_bdot_loop(duration=60, dt=SIMULATION_TIMESTEP):
    """
    Realistic B-dot control loop that simulates real-life scenario.
    This function mimics how the B-dot controller would work on actual hardware:
    1. Gets sensor readings (gyro + magnetometer)
    2. Runs attitude filter to get filtered magnetic field
    3. Applies B-dot control law
    4. Returns control commands
    
    Args:
        duration: simulation duration in seconds
        dt: time step in seconds
    Returns:
        dict containing simulation results
    """
    print("Starting realistic B-dot ADCS loop...\n")
    
    # Initialize states
    bdot_state = {}
    gyro_state, mag_state = {}, {}
    q_est = np.array(INITIAL_ROTATION_QUATERNION)
    m_command_prev = np.array([0, 0, 0])
    
    # Initialize data logging
    times = []
    B_filtered_list = []
    commanded_moments = []
    control_torques = []
    
    # Simulate satellite position (simplified - assuming constant position)
    position = np.array(INITIAL_POSITION)
    
    # Run the control loop
    for i in range(int(duration / dt)):
        t = i * dt
        times.append(t)
        
        # --- 1. Get True Magnetic Field (like real satellite would) ---
        B_eci_true, B_body_true = get_magnetic_field_readings(position, q_est, t)
        
        # --- 2. Simulate Sensor Readings (like real hardware) ---
        omega_meas, gyro_state = read_gyroscope(np.array(INITIAL_ANGULAR_VELOCITY), dt, gyro_state)
        B_meas, mag_state = read_magnetometer(B_body_true, m_command_prev, dt, mag_state)
        
        # --- 3. Run Attitude Filter (like real ADCS would) ---
        filter_output = complementary_filter_update(q_est, omega_meas, B_meas, B_eci_true, dt)
        q_est = filter_output['q_est']
        B_filtered = filter_output['B_filtered']
        B_filtered_list.append(B_filtered.copy())
        
        # --- 4. Apply B-dot Control Law (like real controller would) ---
        m_command, bdot_state = b_dot_controller(B_filtered, bdot_state, dt=dt)
        commanded_moments.append(m_command.copy())
        
        # --- 5. Calculate Control Torque ---
        torque = get_control_torque(m_command, B_filtered)
        control_torques.append(torque.copy())
        
        # Update for next iteration
        m_command_prev = m_command.copy()
        
        # Print progress every 10 seconds
        if i % int(10/dt) == 0:
            print(f"Time: {t:.1f}s, B_filtered: {B_filtered*1e6:.2f} μT, m: {m_command:.2e} A·m²")
    
    print(f"\nB-dot loop completed. Generated {len(commanded_moments)} control commands.")
    
    return {
        'times': np.array(times),
        'B_filtered': np.array(B_filtered_list),
        'commanded_moments': np.array(commanded_moments),
        'control_torques': np.array(control_torques)
    }

#dotąd

if __name__ == "__main__":
    run_bdot_loop()



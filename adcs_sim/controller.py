# ==============================================
# controller.py
# Attitude control algorithms
# ----------------------------------------------
# - Implements B-dot control logic
# - Placeholder for additional controllers (e.g., PD, quaternion feedback)
# - Outputs control torques for actuators (e.g., magnetorquers)
# - Replace read_magnetic_field() with real I2C/SPI BNO086 read function (e.g., via pyserial, Adafruit_BNO08x, etc.).
# ==============================================


import numpy as np

from config import K_BDOT

# === B-dot Controller (Cross-Product Form) ===
def b_dot_controller(B_filtered, omega_filtered, k=K_BDOT):
    """
    Calculates the magnetic moment for B-dot control using the cross-product form.
    Args:
        B_filtered: current filtered magnetic field vector (Tesla)
        omega_filtered: current angular velocity vector (rad/s)
        k: B-dot gain
    Returns:
        m_out: commanded magnetic moment (A·m²)
    """
    
    m_out = k * np.cross(omega_filtered, B_filtered)
    
    return m_out

def get_control_torque(m_out, B_body_true):
     
     current_torque = np.cross(m_out, B_body_true)
     return current_torque




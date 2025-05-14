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

# === B-dot Controller Parameters ===
K_BDOT = 1e-6   # [A·m²/(µT/s)] — B-dot gain (tune for your system)
DT = 0.1        # [s] — control loop time step

# === Simulated Sensor Reading ===
def read_magnetic_field():
    # Simulate Earth's magnetic field with small rotation-induced changes
    return np.random.normal(0, 30, 3)  # Replace with real magnetometer read

# === Simulated Magnetorquer Output ===
def apply_magnetic_moment(moment):
    # Replace with real actuator control logic (e.g., PWM to H-bridge)
    print(f"Applying magnetic moment (A·m²): {moment}")

# === B-dot Controller ===
def b_dot_controller(B_prev, B_curr, k=K_BDOT, dt=DT):
    B_dot = (B_curr - B_prev) / dt
    m = -k * B_dot
    return m

# === Main Control Loop ===
def run_bdot_loop():
    print("Starting pure B-dot ADCS loop...\n")
    B_prev = read_magnetic_field()
    
    while True:
        time.sleep(DT)
        B_curr = read_magnetic_field()

        # Compute magnetic dipole moment via B-dot
        m_out = b_dot_controller(B_prev, B_curr)

        # Apply the magnetic dipole to magnetorquers
        apply_magnetic_moment(m_out)

        # Update previous magnetic field
        B_prev = B_curr

# === Start the loop ===
if __name__ == "__main__":
    run_bdot_loop()



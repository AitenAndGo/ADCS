# ==============================================
# sim.py
# Main simulation loop for ADCS
# ----------------------------------------------
# - Integrates orbit, attitude, magnetic field, and sensor models
# - Applies control algorithms and filtering (if enabled)
# - Returns data for visualization
# ==============================================

import numpy as np
from config import INITIAL_POSITION, INITIAL_VELOCITY
from orbit import simulate_orbit

def simulate(duration, time_step):
    times = np.arange(0, duration + time_step, time_step)

    times, positions, velocities = simulate_orbit(
        INITIAL_POSITION,
        INITIAL_VELOCITY,
        times
    )

    return times, positions, velocities
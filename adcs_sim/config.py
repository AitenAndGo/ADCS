# ==============================================
# config.py
# Configuration parameters for the ADCS simulation
# ----------------------------------------------
# - Satellite properties: mass, inertia
# - Orbit parameters: altitude, inclination, simulation time
# - Controller settings: B-dot constants, filter configs
# ==============================================

import numpy as np


# --- Initial parameters ---

# Earth
G = 6.67428e-20             # N * km^2 / kg^2
M_EARTH = 5.97219e24        # kg
R_EARTH = 6378.137          # km (Mean equatorial radius)
MU_EARTH = 398600.4418      # km^3/s^2 (Standard Gravitational Parameter for Earth)

# Initial satellite state
SATELITE_MASS = 2.6                             # kg
ALTITUDE = 408                                  # km
INITIAL_POSITION = [R_EARTH + ALTITUDE, 0, 0]   # in km

# Orbit properties
INCLINATION_DEG = 51.6                              # degrees (Approximate ISS inclination)
INCLINATION_RAD = np.radians(INCLINATION_DEG)       # radians

SEMI_MAJOR_AXIS = np.linalg.norm(INITIAL_POSITION)  # km (For circular orbit, this is the radius)
v_mag = np.sqrt(MU_EARTH / SEMI_MAJOR_AXIS)

INITIAL_VELOCITY = np.array([0.0, v_mag * np.cos(INCLINATION_RAD),
                              v_mag * np.sin(INCLINATION_RAD)])         # km/s

# Simulation time
ORBIT_PERIOD = 2 * np.pi * np.sqrt(SEMI_MAJOR_AXIS**3 / MU_EARTH) # seconds

NUM_ORBITS = 12                                 # Number of orbits to simulate
SIMULATION_TIME = NUM_ORBITS * ORBIT_PERIOD     # Simulate for a few orbits (seconds)
SIMULATION_TIMESTEP = 10                        # Simulation time step (seconds) - Increased for efficiency

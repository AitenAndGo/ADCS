# ==============================================
# config.py
# Configuration parameters for the ADCS simulation
# ----------------------------------------------
# - Satellite properties: mass, inertia
# - Orbit parameters: altitude, inclination, simulation time
# - Controller settings: B-dot constants, filter configs
# ==============================================

import numpy as np
from utils import euler_2_quaternion
from datetime import datetime, timedelta, UTC


# --- Initial parameters ---

# Earth
G = 6.67428e-20             # N * km^2 / kg^2
M_EARTH = 5.97219e24        # kg
R_EARTH = 6378.137          # km (Mean equatorial radius)
MU_EARTH = 398600.4418      # km^3/s^2 (Standard Gravitational Parameter for Earth)
EARTH_RADIUS = 6371000      # Earth's radius in meters (used in IGRF)

# Initial satellite state
SATELITE_MASS = 2.6                             # kg

# Inertia matrix in kg·m²
SATELITE_INERTIA = np.array([
    [0.7, 0.0, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.6]
])

INERTIA = SATELITE_INERTIA
INERTIA_INV = np.linalg.inv(INERTIA)


ALTITUDE = 525                                  # km
INITIAL_POSITION = [R_EARTH + ALTITUDE, 0, 0]   # km (ECI frame)

# Satelite initail attitude and angular velocity
PHI0 = 0.8          # rad                                                                
THETA0 = 0.4        # rad
PSI0 = -0.6         # rad
INITIAL_ROTATION_QUATERNION = euler_2_quaternion(PHI0, THETA0, PSI0) # quaternion

I0 = 00.021         # rad/s
J0 = 00.013         # rad/s
K0 = 00.037         # rad/s
INITIAL_ANGULAR_VELOCITY = [I0, J0, K0] # rad/s (body frame)

# TORQUE
INITIAL_MAGNETORQUERS = [0, 0, 0]
INITIAL_TORQUE = INITIAL_MAGNETORQUERS # N·m (body frame)

# Orbit properties
INCLINATION_DEG = 51.6                              # degrees (Approximate ISS inclination)
INCLINATION_RAD = np.radians(INCLINATION_DEG)       # radians

SEMI_MAJOR_AXIS = np.linalg.norm(INITIAL_POSITION)  # km (For circular orbit, this is the radius)
v_mag = np.sqrt(MU_EARTH / SEMI_MAJOR_AXIS)

INITIAL_VELOCITY = np.array([0.0, v_mag * np.cos(INCLINATION_RAD),
                              v_mag * np.sin(INCLINATION_RAD)]) # km/s (ECI frame)

# Simulation time
ORBIT_PERIOD = 2 * np.pi * np.sqrt(SEMI_MAJOR_AXIS**3 / MU_EARTH) # seconds

NUM_ORBITS = 1                                  # Number of orbits to simulate
SIMULATION_TIME = NUM_ORBITS * ORBIT_PERIOD     # Simulate for a few orbits (seconds)
SIMULATION_TIMESTEP = 0.1                       # Simulation time step (seconds)
INITIAL_EPOCH = datetime(2024, 1, 1, 12, 0, 0, tzinfo=UTC) # Simulation start epoch

# Note: EARTH_RADIUS is defined twice with different units.
# R_EARTH (km) is used for orbit calculations based on MU_EARTH (km^3/s^2).
# EARTH_RADIUS (m) is likely intended for IGRF which often expects altitude in meters from Earth's surface.
# Ensure consistency in how these are used in orbit.py and magnetic_field.py.
# For IGRF, position_eci is passed in km to cartesian_to_geodetic, which then uses a_wgs84 (6378.137 km)
# and returns altitude in meters. So the EARTH_RADIUS (m) might be redundant if R_EARTH (km) is consistently used.

# Earth Parameters
EARTH_MU = 3.986004418e14  # Earth's gravitational parameter (m^3/s^2)

# Magnetic Field Parameters - IGRF is used in magnetic_field.py
# The dipole MAGNETIC_MOMENT is no longer the primary model if IGRF is active.
# MAGNETIC_MOMENT = np.array([0, 0, -7.96e22])  # Earth's magnetic dipole moment in A·m² (for dipole model)

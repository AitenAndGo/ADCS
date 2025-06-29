# ==============================================
# config.py
# Configuration parameters for the ADCS simulation
# ----------------------------------------------
# Contains all physical constants, satellite properties, orbit parameters,
# controller settings, and simulation timing for the ADCS (Attitude Determination and Control System) simulation.
#
# - Satellite properties: mass, inertia
# - Orbit parameters: altitude, inclination, simulation time
# - Controller settings: B-dot constants, filter configs
# ==============================================

import numpy as np
from utils import euler_2_quaternion
from datetime import datetime, timedelta, UTC

# --- Physical Constants ---

# Universal gravitational constant (N * km^2 / kg^2)
G = 6.67428e-20
# Mass of Earth (kg)
M_EARTH = 5.97219e24
# Mean equatorial radius of Earth (km)
R_EARTH = 6378.137
# Standard gravitational parameter for Earth (km^3/s^2)
MU_EARTH = 398600.4418
# Earth's radius in meters (used for IGRF magnetic field model)
EARTH_RADIUS = 6371000

# --- Satellite Properties ---

# Mass of the satellite (kg)
SATELITE_MASS = 2.6

# Inertia matrix of the satellite (kg·m²)
SATELITE_INERTIA = np.array([
    [0.7, 0.0, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.6]
])

# Alias for convenience
INERTIA = SATELITE_INERTIA
# Precompute inverse inertia matrix for dynamics calculations
INERTIA_INV = np.linalg.inv(INERTIA)

# --- Orbit Parameters ---

# Altitude of the satellite above Earth's surface (km)
ALTITUDE = 525
# Initial position in ECI frame (km)
INITIAL_POSITION = [R_EARTH + ALTITUDE, 0, 0]

# --- Initial Attitude and Angular Velocity ---

# Initial Euler angles (radians)
PHI0 = 0.8          # Roll
THETA0 = 0.4        # Pitch
PSI0 = -0.6         # Yaw
# Initial attitude as quaternion (for simulation)
INITIAL_ROTATION_QUATERNION = euler_2_quaternion(PHI0, THETA0, PSI0)

# Initial angular velocity components (rad/s, body frame)
I0 = 00.021         # x-axis
J0 = 00.013         # y-axis
K0 = 00.037         # z-axis
INITIAL_ANGULAR_VELOCITY = [I0, J0, K0]

# --- Initial Control Inputs ---

# Initial magnetorquer command (A·m², body frame)
INITIAL_MAGNETORQUERS = [0, 0, 0]
# Initial applied torque (N·m, body frame)
INITIAL_TORQUE = INITIAL_MAGNETORQUERS

# --- Orbit Geometry ---

# Orbital inclination (degrees, typical for ISS)
INCLINATION_DEG = 51.6
# Orbital inclination (radians)
INCLINATION_RAD = np.radians(INCLINATION_DEG)

# Semi-major axis (km) for circular orbit (equals orbital radius)
SEMI_MAJOR_AXIS = np.linalg.norm(INITIAL_POSITION)
# Magnitude of orbital velocity (km/s)
v_mag = np.sqrt(MU_EARTH / SEMI_MAJOR_AXIS)

# Initial velocity in ECI frame (km/s)
# x: 0, y: in-plane, z: out-of-plane (due to inclination)
INITIAL_VELOCITY = np.array([0.0, v_mag * np.cos(INCLINATION_RAD),
                              v_mag * np.sin(INCLINATION_RAD)])

# --- Simulation Timing ---

# Orbital period (seconds) for given semi-major axis
ORBIT_PERIOD = 2 * np.pi * np.sqrt(SEMI_MAJOR_AXIS**3 / MU_EARTH)

# Number of orbits to simulate
NUM_ORBITS = 10
# Total simulation time (seconds)
SIMULATION_TIME = NUM_ORBITS * ORBIT_PERIOD
# Simulation time step (seconds)
SIMULATION_TIMESTEP = 1
# Start epoch for simulation (UTC)
INITIAL_EPOCH = datetime(2024, 1, 1, 12, 0, 0, tzinfo=UTC)

# --- Notes on Earth Radius Usage ---
# R_EARTH (km) is used for orbital calculations (consistent with MU_EARTH units).
# EARTH_RADIUS (m) is used for IGRF magnetic field model, which expects altitude in meters.
# Ensure correct units are used in orbit.py and magnetic_field.py.
#
# For IGRF, position_eci is passed in km to cartesian_to_geodetic, which uses a_wgs84 (6378.137 km)
# and returns altitude in meters. EARTH_RADIUS (m) may be redundant if R_EARTH (km) is used consistently.

# --- Earth Gravitational Parameter (SI units) ---
# Used for some calculations requiring meters (not km)
EARTH_MU = 3.986004418e14  # m^3/s^2

# --- Magnetic Field Model Parameters ---
# IGRF is used in magnetic_field.py for realistic geomagnetic field.
# The dipole MAGNETIC_MOMENT is commented out, as IGRF is preferred.
# MAGNETIC_MOMENT = np.array([0, 0, -7.96e22])  # A·m² (for dipole model)

# --- B-dot Controller Gain ---
# B-dot gain (A·m²/(T/s)), tune for your system
K_BDOT = 67200

# --- Magnetorquer Disturbance on Magnetometer ---
# Models the magnetic field created by the magnetorquers as seen by the magnetometer.
# Simplified linear model: B_disturbance = C * m_command
# C is a scalar gain (T / A·m²)
MAG_DISTURBANCE_GAIN = 1e-7

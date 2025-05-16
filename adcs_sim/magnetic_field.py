# ==============================================
# magnetic_field.py
# Earth's magnetic field model using IGRF
# ----------------------------------------------
# - Computes magnetic field vector at satellite position
# - Uses IGRF-13 model for accurate field calculations
# - Transforms between coordinate frames
# ==============================================

import numpy as np
from datetime import datetime, UTC, timedelta
from geomag import geomag
from utils import quaternion_to_rotation_matrix
from config import INITIAL_EPOCH

# Initialize IGRF model
gm = geomag.GeoMag()

def cartesian_to_geodetic(position_eci):
    """
    Convert Cartesian ECI coordinates to geodetic coordinates (lat, lon, alt).
    
    Parameters:
    - position_eci: [x, y, z] position vector in ECI frame (km)
    
    Returns:
    - latitude: degrees
    - longitude: degrees
    - altitude: meters above sea level
    """
    x, y, z = position_eci
    
    # WGS84 ellipsoid parameters
    a = 6378.137  # Earth's semi-major axis in km (consistent with R_EARTH in config for orbit)
    f = 1/298.257223563  # Flattening
    b = a*(1-f)  # Semi-minor axis
    e2 = 2*f - f*f  # Square of eccentricity
    
    # Calculate longitude
    longitude = np.degrees(np.arctan2(y, x))
    
    # Iterative calculation of latitude and altitude
    p = np.sqrt(x*x + y*y)
    latitude = np.arctan2(z, p*(1-e2))
    
    for _ in range(5):  # Usually converges in 2-3 iterations
        N_val = a / np.sqrt(1 - e2*np.sin(latitude)**2)
        h = p/np.cos(latitude) - N_val
        latitude = np.arctan2(z, p*(1-e2*N_val/(N_val+h)))
    
    latitude = np.degrees(latitude)
    altitude = h * 1000  # Convert to meters
    
    return latitude, longitude, altitude

def calculate_magnetic_field_igrf(position_eci, sim_time_seconds):
    """
    Calculate Earth's magnetic field using the IGRF-13 model.
    
    Parameters:
    - position_eci: [x, y, z] position vector in ECI frame (km)
    - sim_time_seconds: Simulation time in seconds from INITIAL_EPOCH
    
    Returns:
    - magnetic_field_eci: [Bx, By, Bz] magnetic field vector in ECI frame (Tesla)
    """
    # Convert ECI coordinates to geodetic
    lat, lon, alt = cartesian_to_geodetic(position_eci)
    
    # Calculate current simulation datetime from epoch and elapsed seconds
    current_simulation_datetime = INITIAL_EPOCH + timedelta(seconds=sim_time_seconds)
    
    # Calculate magnetic field components using IGRF
    mag_data = gm.GeoMag(lat, lon, alt, current_simulation_datetime.date())
    
    # Convert from nT to Tesla
    # IGRF gives field in North-East-Down (NED) coordinates
    Bn_nT = mag_data.bx  # North component in nT
    Be_nT = mag_data.by  # East component in nT
    Bd_nT = mag_data.bz  # Down component in nT (Note: geomag library might use bz for down)
    
    Bn = Bn_nT * 1e-9  # Convert to Tesla
    Be = Be_nT * 1e-9  # Convert to Tesla
    Bd = Bd_nT * 1e-9  # Convert to Tesla
    
    # Convert from NED to ECI
    lon_rad = np.radians(lon)
    lat_rad = np.radians(lat)
    
    cos_lat = np.cos(lat_rad)
    sin_lat = np.sin(lat_rad)
    cos_lon = np.cos(lon_rad)
    sin_lon = np.sin(lon_rad)
    
    # Rotation matrix from NED to ECI (simplified, more advanced would include Earth rotation effects accurately)
    R_ned_to_eci = np.array([
        [-cos_lon*sin_lat, -sin_lon, -cos_lon*cos_lat],
        [-sin_lon*sin_lat,  cos_lon, -sin_lon*cos_lat],
        [cos_lat,           0,       -sin_lat]
    ])
    
    B_ned = np.array([Bn, Be, Bd])
    B_eci = R_ned_to_eci @ B_ned
    
    return B_eci

def get_magnetic_field_readings(position, quaternion, sim_time_seconds):
    """
    Calculate magnetic field readings in the satellite's body frame.
    
    Parameters:
    - position: [x, y, z] position vector in ECI frame (km)
    - quaternion: [q0, q1, q2, q3] rotation quaternion from ECI to body frame
    - sim_time_seconds: Simulation time in seconds from INITIAL_EPOCH
    
    Returns:
    - magnetic_field_eci: [Bx, By, Bz] magnetic field vector in ECI frame (Tesla)
    - magnetic_field_body: [Bx, By, Bz] magnetic field vector in body frame (Tesla)
    """
    # Get magnetic field in ECI frame using IGRF model
    magnetic_field_eci = calculate_magnetic_field_igrf(position, sim_time_seconds)
    
    # Get rotation matrix from quaternion using the utility function
    R = quaternion_to_rotation_matrix(quaternion)
    
    # Transform magnetic field from ECI to body frame
    magnetic_field_body = R @ magnetic_field_eci
    
    return magnetic_field_eci, magnetic_field_body

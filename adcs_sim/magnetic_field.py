# ==============================================
# magnetic_field.py
# Earth's magnetic field model
# ----------------------------------------------
# - Computes magnetic field vector at satellite position
# - Can use IGRF model or simplified dipole approximation
# install geomag for igrf
# ==============================================
from geomag import GeoMag
from datetime import datetime

# Initialize geomagnetic model
gm = GeoMag()

# Example location: latitude, longitude, altitude (meters)
latitude = 51.0     # degrees (e.g., Warsaw)
longitude = 0.0     # degrees
altitude = 400000   # altitude in meters (~400 km LEO satellite)

# Get magnetic field at current time and position
mag_data = gm.GeoMag(latitude, longitude, altitude)

# Output the magnetic field components (nT)
print(f"Magnetic Field at ({latitude}°, {longitude}°, {altitude} m):")
print(f"  X (North)    = {mag_data.x:.2f} nT")
print(f"  Y (East)     = {mag_data.y:.2f} nT")
print(f"  Z (Down)     = {mag_data.z:.2f} nT")
print(f"  Total field  = {mag_data.t:.2f} nT")


# If you’re interested, I can help simulate the Earth's magnetic field over a full orbit (e.g., using TLE data and sgp4) and visualize the field strength or direction. Just let me know!

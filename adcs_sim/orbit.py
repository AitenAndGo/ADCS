# ==============================================
# orbit.py
# Satellite orbit model
# ----------------------------------------------
# - Calculates position and velocity relative to Earth
# - Supports simplified orbital dynamics or Keplerian motion
# ==============================================

import numpy as np
from scipy.integrate import solve_ivp
from config import MU_EARTH, G, SATELITE_MASS, M_EARTH


def two_body_problem(t, state):
    """
    Computes the derivative of the state vector for two-body orbital motion.

    Parameters:
    - t: Time (seconds)
    - state: State vector [x, y, z, vx, vy, vz]

    Returns:
    - Derivative of the state vector: [vx, vy, vz, ax, ay, az]
    """

    r = state[0:3]
    v = state[3:6]
    norm_r = np.linalg.norm(r)
    Fgrav = (-G * SATELITE_MASS * M_EARTH  / norm_r**2) * (r / norm_r)
    a = Fgrav / SATELITE_MASS

    return np.concatenate((v, a))


def simulate_orbit(initial_position, initial_velocity, times):
    """
    Simulates the satellite's orbit using the two-body problem.

    Parameters:
    - initial_position: Initial position vector [x, y, z] in km
    - initial_velocity: Initial velocity vector [vx, vy, vz] in km/s
    - time: Time (in seconds) at which to evaluate the orbit

    Returns:
    - position: Position vector [x, y, z] in km
    - velocity: Velocity vector [vx, vy, vz] in km/s
    """

    initial_state = [initial_position[0], initial_position[1], initial_position[2],
                     initial_velocity[0], initial_velocity[1], initial_velocity[2]]

    print(initial_state)

    solution = solve_ivp(
        fun=two_body_problem,
        method='RK45',
        t_span=(times[0], times[-1]),
        y0=initial_state,
        t_eval=times,
        rtol=1e-9,
        atol=1e-9
    )

    print(solution.y)

    rvec = solution.y[:3].T
    vvec = solution.y[3:6].T

    return solution.t, rvec, vvec
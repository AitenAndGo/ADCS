# Attitude Determination and Control System (ADCS) Simulation

Author: Bartosz Polak, Tymoteusz Walczak
Date: June 2025
Repository: https://github.com/AitenAndGo/ADCS

## Abstract

This project implements a complete Attitude Determination and Control System (ADCS) simulation for a satellite in low Earth orbit. The simulation models closed-loop behavior, incorporating orbital propagation, rotational dynamics, realistic sensor models, signal filtering, and control actuation using the B-dot algorithm. The system is written in Python and produces outputs suitable for further analysis and visualization, including a standalone Unity-based 3D visualization tool for enhanced presentation of the satellite's motion.

## Features

- Numerical integration of orbital and rotational dynamics
- Simulation of noisy sensor readings (gyroscope and magnetometer)
- Implementation of a complementary filter for sensor fusion
- B-dot control algorithm for magnetic attitude stabilization
- Generation of analysis plots for attitude, control torques, and sensor data
- Data export to CSV format for external visualization
- Optional launching of a Unity-based 3D visualization application

## Repository Structure

ADCS/
├── main.py                 # Main entry point: simulation + plotting + data export
├── sim.py                  # Core simulation engine (closed-loop)
├── config.py               # Initial conditions and constants
├── /controller/            # B-dot controller and torque computation
├── /filters/               # Complementary filtering implementation
├── /sensors/               # Sensor models with noise
├── /orbit/, /attitude/     # Physics-based models for orbit and attitude
├── requirements.txt        # Python dependencies
└── SateliteSim.exe         # Unity-based visualization (external)




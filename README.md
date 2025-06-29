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

## Installation Instructions

### Prerequisites
- Python 3.9 or higher
- Git (for cloning the repository)
- Unity visualization requires Windows OS (optional)

### Step-by-Step Setup

1. Clone the repository:
```
git clone https://github.com/AitenAndGo/ADCS.git
cd ADCS
```

2. Create a virtual environment (optional but recommended):
```
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install required Python packages:
```
pip install -r requirements.txt
```

4. Download the Unity visualization executable:
- Download from: [Unity Build (Google Drive)](https://drive.google.com/file/d/1VWN5XnW7O_ditpmI6GVgmJEPzqd2TMXW/view?usp=drive_link)
- Extract the contents of the archive into the root ADCS/ directory.
- Ensure the executable file SateliteSim.exe is located at:

## Running the Simulation

To start the simulation and generate plots:
```
python main.py
```
This will perform the following actions:
- Run the complete ADCS simulation
- Generate plots of orbit trajectory, Euler angles, angular velocities, control torques, and sensor signals
- Export orientation and position data to .csv files in the Unity visualization directory
- Attempt to launch the Unity visualization tool (if on Windows)

## Simulation Description

The simulation includes the following major components:

- Orbit Propagation: Integrates satellite position and velocity using a simplified two-body model.
- Attitude Propagation: Uses quaternion-based integration for rotational motion.
- Sensor Simulation: Adds realistic noise to gyroscope and magnetometer data.
- Complementary Filtering: Combines sensor data to estimate angular velocity and magnetic field.
- Control Law: Implements the B-dot controller to damp rotational motion using magnetorquers.
- Data Export: Outputs orientation and position data in CSV format for visualization purposes.

## Unity Visualization

If run on Windows with the Unity executable available, main.py will automatically:
- Export trajectory and attitude data to the Unity app's StreamingAssets folder
- Launch SateliteSim.exe for interactive 3D visualization
Ensure Unity files are correctly extracted and present in the project root.

## Acknowledgements

This project was developed as part of an academic exploration of spacecraft attitude control. The author acknowledges guidance and resources provided by the university research community and open-source developers in the aerospace and simulation fields.

License

This project is released under the MIT License. See [MIT License](LICENSE) file for details.

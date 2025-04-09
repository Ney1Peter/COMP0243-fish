# COMP0243-fish
soft robotic fish

# 🐟 Fish Simulation in PyBullet

This project simulates the movement of a fish in a virtual aquatic environment using the [PyBullet](https://github.com/bulletphysics/bullet3) physics engine. It evaluates propulsion behavior, tail swing dynamics, water resistance forces, and trajectory stability in a controlled simulation environment.

fish-simulation/
├── fish_simulation.py        # Main simulation script
├── requirements.txt          # Dependencies list
└── README.md                 # Project description


## 📂 Project Features

- 3D fish model loaded via URDF
- Simulation of water flow using external forces
- Dynamic gravity manipulation
- Tail swing modeled using sine functions
- Real-time head angle correction and propulsion control
- Energy consumption tracking and trajectory deviation analysis

## 📦 Requirements

This project requires the following Python packages:

```txt
pybullet>=3.2.5
numpy>=1.21.0


You can install them using:
pip install -r requirements.txt

## How to run
git clone https://github.com/your_username/fish-simulation.git
cd fish-simulation

### Run the simulation script:
python load_fish.py

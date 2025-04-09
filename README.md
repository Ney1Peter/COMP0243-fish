# COMP0243-soft robotic

# Fish Simulation in PyBullet

This project simulates the movement of a fish in a virtual aquatic environment using the [PyBullet](https://github.com/bulletphysics/bullet3) physics engine. It evaluates propulsion behavior, tail swing dynamics, water resistance forces, and trajectory stability in a controlled simulation environment.

## Project Features

- 3D fish model loaded via URDF
- Simulation of water flow using external forces
- Dynamic gravity manipulation
- Tail swing modeled using sine functions
- Real-time head angle correction and propulsion control
- Energy consumption tracking and trajectory deviation analysis

## Requirements

This project requires the following Python packages:

```plaintext
pybullet>=3.2.5
numpy>=1.21.0
```

You can install them using:
```plaintext
pip install -r requirements.txt
```

## How to run
```plaintext
git clone https://github.com/Ney1Peter/COMP0243-fish.git
```

Run the simulation script:
```plaintext
python load_fish.py
```

Run the adaptive ststem:
```plaintext
python adaptive.py
```

Run the evaluation script:
```plaintext
python generate_1.py
python generate_2.py
python generate_3.py
python generate_4.py
```

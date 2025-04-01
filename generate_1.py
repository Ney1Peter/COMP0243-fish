#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Define data, each row represents one experiment result
data = [
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.0,
        "intrinsic_propulsion_force (N)": 12.944489738407286,
        "tail_amplitude (rad)": 0.06472244869203643,
        "arrival_time (s)": 36.86165237426758,
        "propulsion_energy (N·s)": 9.02747805677338,
        "avg_deviation (m)": 0.20336938492476514
    },
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 12.926193112676746,
        "tail_amplitude (rad)": 0.06463096556338373,
        "arrival_time (s)": 41.31096625328064,
        "propulsion_energy (N·s)": 10.109629604008116,
        "avg_deviation (m)": 0.20372922403444646
    },
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.5,
        "intrinsic_propulsion_force (N)": 12.940836265142016,
        "tail_amplitude (rad)": 0.06470418132571008,
        "arrival_time (s)": 37.954965591430664,
        "propulsion_energy (N·s)": 9.555241527645324,
        "avg_deviation (m)": 0.2456937990459284
    },
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.8,
        "intrinsic_propulsion_force (N)": 13.004637423304436,
        "tail_amplitude (rad)": 0.06502318711652218,
        "arrival_time (s)": 38.045111417770386,
        "propulsion_energy (N·s)": 9.227181899017227,
        "avg_deviation (m)": 0.23181334423915975
    },
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 1.0,
        "intrinsic_propulsion_force (N)": 12.949832286089,
        "tail_amplitude (rad)": 0.064749161430445,
        "arrival_time (s)": 35.89835453033447,
        "propulsion_energy (N·s)": 8.863622435516334,
        "avg_deviation (m)": 0.21594525156623376
    }
]

# Construct DataFrame using pandas and print the table
df = pd.DataFrame(data)
print("Experiment Data Table:")
print(df)

# Figure 1: Arrival time and propulsion energy vs. water flow frequency (different colors in one plot)
plt.figure(figsize=(8,6))
plt.plot(df["water_flow_frequency (Hz)"], df["arrival_time (s)"], marker='o', linestyle='-', color="blue", label="Arrival Time (s)")
plt.plot(df["water_flow_frequency (Hz)"], df["propulsion_energy (N·s)"], marker='s', linestyle='-', color="red", label="Propulsion Energy (N·s)")
plt.xlabel("Water Flow Frequency (Hz)")
plt.ylabel("Value")
plt.title("Arrival Time & Propulsion Energy in different Water Flow Frequency\n(Water Flow Amplitude = 0.5 N)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("Arrival time and energy in different frequency.png")  # Save image (optional)
plt.show()

# Figure 2: Average trajectory deviation vs. water flow frequency
plt.figure(figsize=(8,6))
plt.plot(df["water_flow_frequency (Hz)"], df["avg_deviation (m)"], marker='o', linestyle='-', color="green")
plt.xlabel("Water Flow Frequency (Hz)")
plt.ylabel("Average Trajectory Deviation (m)")
plt.title("Average Trajectory Deviation in different Water Flow Frequency\n(Water Flow Amplitude = 0.5 N)")
plt.grid(True)
plt.tight_layout()
plt.savefig("Avg deviation in different frequency.png")  # Save image (optional)
plt.show()

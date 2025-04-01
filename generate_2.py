#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Define data, each row represents one experiment result
data = [
    {
        "water_flow_amplitude (N)": 0.0,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 12.996147402300927,
        "tail_amplitude (rad)": 0.06498073701150464,
        "arrival_time (s)": 36.271326780319214,
        "propulsion_energy (N·s)": 8.98258174041047,
        "avg_deviation (m)": 0.20247867541670078
    },
    {
        "water_flow_amplitude (N)": 0.2,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 12.940498144980541,
        "tail_amplitude (rad)": 0.06470249072490271,
        "arrival_time (s)": 38.013190507888794,
        "propulsion_energy (N·s)": 9.531789391835483,
        "avg_deviation (m)": 0.18374233090264158
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
        "water_flow_amplitude (N)": 0.8,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 12.988099338486656,
        "tail_amplitude (rad)": 0.06494049669243328,
        "arrival_time (s)": 41.87571358680725,
        "propulsion_energy (N·s)": 10.240999839175567,
        "avg_deviation (m)": 0.23192767320022978
    },
    {
        "water_flow_amplitude (N)": 1.0,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 12.95631752688189,
        "tail_amplitude (rad)": 0.06478158763440944,
        "arrival_time (s)": 42.1204309463501,
        "propulsion_energy (N·s)": 10.520818066878006,
        "avg_deviation (m)": 0.24658027724525552
    }
]

# Construct the DataFrame table
df = pd.DataFrame(data)
print("Experiment Data Table:")
print(df)

# Figure 1: Arrival time and propulsion energy vs. water flow resistance (plot two curves in the same figure)
plt.figure(figsize=(8,6))
plt.plot(df["water_flow_amplitude (N)"], df["arrival_time (s)"], marker='o', linestyle='-', color='blue', label="Arrival Time (s)")
plt.plot(df["water_flow_amplitude (N)"], df["propulsion_energy (N·s)"], marker='s', linestyle='-', color='red', label="Propulsion Energy (N·s)")
plt.xlabel("Water Flow Amplitude (N)")
plt.ylabel("Value")
plt.title("Arrival Time & Propulsion Energy in different Water Flow Amplitude\n(Water Flow Frequency = 0.3 Hz)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("arrival_time_and_energy_vs_amplitude.png")
plt.show()

# Figure 2: Average trajectory deviation vs. water flow resistance
plt.figure(figsize=(8,6))
plt.plot(df["water_flow_amplitude (N)"], df["avg_deviation (m)"], marker='o', linestyle='-', color='green')
plt.xlabel("Water Flow Amplitude (N)")
plt.ylabel("Average Trajectory Deviation (m)")
plt.title("Average Trajectory Deviation in different Water Flow Amplitude\n(Water Flow Frequency = 0.3 Hz)")
plt.grid(True)
plt.tight_layout()
plt.savefig("avg_deviation_vs_amplitude.png")
plt.show()

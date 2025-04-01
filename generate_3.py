#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Define data, each row represents one experiment result
data = [
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 6.4783004310266,
        "tail_amplitude (rad)": 0.032391502155133,
        "arrival_time (s)": 65.71479201316833,
        "propulsion_energy (N·s)": 8.058362140716902,
        "avg_deviation (m)": 0.24314137479052703
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
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 25.949928974773062,
        "tail_amplitude (rad)": 0.1297496448738653,
        "arrival_time (s)": 21.296332836151123,
        "propulsion_energy (N·s)": 10.525827854790535,
        "avg_deviation (m)": 0.1876734573396961
    },
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 38.97666512083853,
        "tail_amplitude (rad)": 0.19488332560419266,
        "arrival_time (s)": 17.854430675506592,
        "propulsion_energy (N·s)": 13.3074930616342,
        "avg_deviation (m)": 0.1695197087288668
    },
    {
        "water_flow_amplitude (N)": 0.5,
        "water_flow_frequency (Hz)": 0.3,
        "intrinsic_propulsion_force (N)": 65.00900310255666,
        "tail_amplitude (rad)": 0.3250450155127833,
        "arrival_time (s)": 13.681216478347778,
        "propulsion_energy (N·s)": 16.05353020392603,
        "avg_deviation (m)": 0.1852481459363188
    }
]

# Construct the DataFrame using pandas
df = pd.DataFrame(data)
print("Experiment Data Table:")
print(df)

# Figure 1: Arrival time and propulsion energy vs. tail amplitude (shared x-axis: tail amplitude in radians)
plt.figure(figsize=(8,6))
plt.plot(df["tail_amplitude (rad)"], df["arrival_time (s)"], marker='o', linestyle='-', color='blue', label="Arrival Time (s)")
plt.plot(df["tail_amplitude (rad)"], df["propulsion_energy (N·s)"], marker='s', linestyle='-', color='red', label="Propulsion Energy (N·s)")
plt.xlabel("Tail Amplitude (rad)")
plt.ylabel("Value")
plt.title("Arrival Time & Propulsion Energy in different Tail Amplitude\n(water_flow_amplitude = 0.5 N, water_flow_frequency = 0.3 Hz)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("arrival_time_and_energy_vs_tail_amplitude.png")
plt.show()

# Figure 2: Line chart of average trajectory deviation vs. tail amplitude
plt.figure(figsize=(8,6))
plt.plot(df["tail_amplitude (rad)"], df["avg_deviation (m)"], marker='o', linestyle='-', color='green')
plt.xlabel("Tail Amplitude (rad)")
plt.ylabel("Average Trajectory Deviation (m)")
plt.title("Average Trajectory Deviation in different Tail Amplitude\n(water_flow_amplitude = 0.5 N, water_flow_frequency = 0.3 Hz)")
plt.grid(True)
plt.tight_layout()
plt.savefig("avg_deviation_vs_tail_amplitude.png")
plt.show()

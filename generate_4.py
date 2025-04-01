#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Define experimental data
data = [
    {
        "water_flow_frequency (Hz)": 0.0,
        "arrival_time_before (s)": 36.86165237426758,
        "arrival_time_after (s)": 24.23546051979065,
        "avg_deviation_before (m)": 0.20336938492476514,
        "avg_deviation_after (m)": 0.23852193442749323
    },
    {
        "water_flow_frequency (Hz)": 0.3,
        "arrival_time_before (s)": 41.31096625328064,
        "arrival_time_after (s)": 23.443081855773926,
        "avg_deviation_before (m)": 0.20372922403444646,
        "avg_deviation_after (m)": 0.24448012404111177
    },
    {
        "water_flow_frequency (Hz)": 0.5,
        "arrival_time_before (s)": 37.954965591430664,
        "arrival_time_after (s)": 25.463353157043457,
        "avg_deviation_before (m)": 0.2456937990459284,
        "avg_deviation_after (m)": 0.278347702643297
    },
    {
        "water_flow_frequency (Hz)": 0.8,
        "arrival_time_before (s)": 38.045111417770386,
        "arrival_time_after (s)": 26.017811059951782,
        "avg_deviation_before (m)": 0.23181334423915975,
        "avg_deviation_after (m)": 0.2569105601357977
    },
    {
        "water_flow_frequency (Hz)": 1.0,
        "arrival_time_before (s)": 35.89835453033447,
        "arrival_time_after (s)": 23.966724157333374,
        "avg_deviation_before (m)": 0.21594525156623376,
        "avg_deviation_after (m)": 0.23643400962087194
    }
]

# Construct the DataFrame table
df = pd.DataFrame(data)
print("Experiment Data Table:")
print(df)

# Figure 1: Arrival time vs. water flow frequency
plt.figure(figsize=(8,6))
plt.plot(df["water_flow_frequency (Hz)"], df["arrival_time_before (s)"], marker='o', linestyle='-', color='blue', label="Adaptive Before")
plt.plot(df["water_flow_frequency (Hz)"], df["arrival_time_after (s)"], marker='s', linestyle='-', color='red', label="Adaptive After")
plt.xlabel("Water Flow Frequency (Hz)")
plt.ylabel("Arrival Time (s)")
plt.title("Arrival Time vs Water Flow Frequency\n(Water Flow Amplitude = 0.5 N)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("arrival_time_vs_frequency.png")
plt.show()

# Figure 2: Average trajectory deviation vs. water flow frequency
plt.figure(figsize=(8,6))
plt.plot(df["water_flow_frequency (Hz)"], df["avg_deviation_before (m)"], marker='o', linestyle='-', color='blue', label="Adaptive Before")
plt.plot(df["water_flow_frequency (Hz)"], df["avg_deviation_after (m)"], marker='s', linestyle='-', color='red', label="Adaptive After")
plt.xlabel("Water Flow Frequency (Hz)")
plt.ylabel("Average Trajectory Deviation (m)")
plt.title("Average Trajectory Deviation vs Water Flow Frequency\n(Water Flow Amplitude = 0.5 N)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("avg_deviation_vs_frequency.png")
plt.show()

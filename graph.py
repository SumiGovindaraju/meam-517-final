#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd

controller_df = pd.read_csv("controller_data.csv")
sim_df = pd.read_csv("sim_data.csv")

def plot_col(col):
    plt.figure(figsize=(10, 6))

    plt.plot(controller_df['t'], controller_df[col], label='MPC', marker='o')
    plt.plot(sim_df['t'], sim_df[col], label='Baseline', marker='o')

    plt.title(col)
    plt.xlabel("Time (s)")
    plt.ylabel("Value")  # Changed ylabel to a generic label

    plt.legend()  # Show legend with labels 'MPC' and 'Baseline'

    plt.show()

plot_col("mean_percent_occupancy")
plot_col("mean_waiting_time")
plot_col("vehicle_speed")
plot_col("emissions")

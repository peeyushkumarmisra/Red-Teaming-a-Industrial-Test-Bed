import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# CONFIG
CSV_PATH = '/workspaces/thesis/experiment_data.csv'

def plot_analysis(experiment_name='exp'):
    if not os.path.exists(CSV_PATH):
        print(f"Error: CSV {CSV_PATH} not found")
        return
    # Getting CSV 
    df = pd.read_csv(CSV_PATH)
    df['Time_s'] = df['TimeStep'] * 0.01
    t_end = df['Time_s'].max()

    # Plotting
    fig, axes = plt.subplots(2, 1, figsize = (14, 16), sharex = True)
    ax1, ax2 = axes
    ax1.set_title(f'Shadow Controller IDS — {experiment_name.upper()}')
    ax2.set_xlabel('Time (seconds)')

    # Plot 1 - Torques
    ax1.plot(df['Time_s'], df['tau_actual_norm'], 
             label = 'Actual Commanded Torque (from controller)', 
             color = '#1f77b4', linewidth = 1.2)
    ax1.plot(df['Time_s'], df['tau_planned_norm'], 
             label = 'Expected Torque from Robot Model (inverse dynamics)',
             color = '#ff7f0e', linewidth = 1.2, alpha = 0.8)

    ax1.set_ylabel('Torque (Nm)')
    ax1.set_ylim(top=500)
    ax1.legend(loc = 'upper left', fontsize = 9)
    ax1.grid(True, linestyle = '--', alpha = 0.5)

    # Plot 2 - EWMA Internals
    ax2.plot(df['Time_s'], df['EWMA_Fast'], 
             label = 'Fast EWMA (attack-sensitive tracker)', 
             color = '#ff7f0e', linewidth = 1)
    ax2.plot(df['Time_s'], df['EWMA_Slow'], 
             label = 'Slow EWMA (long-term baseline)', 
             color = '#2ca02c', linewidth = 1)
    ax2.plot(df['Time_s'], df['EWMA_Delta'], 
             label = 'Deviation between Fast and Slow EWMA', 
             color = '#9467bd', linewidth = 1.5)
    ax2.plot(df['Time_s'], df['Threshold'], 
             label = 'Dynamic Detection Threshold', 
             color = '#d62728', linewidth = 2, linestyle = '--')

    ax2.set_ylabel('EWMA Values')
    ax2.set_ylim(top=500)
    ax2.legend(loc = 'upper left', fontsize = 9)
    ax2.grid(True, linestyle='--', alpha = 0.5)

    # Shading Plots
    for ax in [ax1, ax2]:
        # Payload Context
        ax.fill_between(df['Time_s'], 0, 1, where=(df['Context'] == 1),
                        label='Payload Attached',
                        color="#4292d4", alpha=0.15,
                        transform=ax.get_xaxis_transform())
        # Attack Time
        attack_mask = df['Attack_Flag'] == 1
        if attack_mask.any():
            ax.fill_between(df['Time_s'], 0, 1, where=attack_mask,
                            label='Attack Detected',
                            color="#f13838", alpha=0.3,
                            transform=ax.get_xaxis_transform())
        # Friction degradation
        ax.axvspan(100, t_end, label='Friction Degradation',
                   color="#ffa500", alpha=0.12)

    # Saving
    plt.tight_layout()
    save_path = f'/workspaces/thesis/{experiment_name}.png'
    plt.savefig(save_path, dpi = 300, bbox_inches = 'tight')
    print(f"Saved plot to {save_path}")
    plt.show()

if __name__ == '__main__':
    exp = sys.argv[1] if len(sys.argv) > 1 else 'exp1'
    plot_analysis(exp)
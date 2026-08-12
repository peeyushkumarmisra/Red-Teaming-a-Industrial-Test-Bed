import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_full_analysis():
    csv_path = '/workspaces/thesis/experiment_data.csv'
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found")
        return

    df = pd.read_csv(csv_path)
    df['Time_s'] = df['TimeStep'] * 0.01  # 100 Hz = 0.01s

    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    ax1, ax2, ax3, ax4, ax5 = axes

    # --- Plot 1: Torques ---
    ax1.plot(df['Time_s'], df['tau_cmd_norm'], label='||τ_cmd|| (Controller Plan)', color='#1f77b4', linewidth=1.2)
    ax1.plot(df['Time_s'], df['tau_id_norm'], label='||τ_ID(q,qd,qdd)|| (Observed Physics)', color='#ff7f0e', linewidth=1.2, alpha=0.8)
    ax1.set_ylabel('Torque Norm (Nm)')
    ax1.set_title('Shadow Controller IDS — Full Internal Analysis')
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, linestyle='--', alpha=0.5)

    # --- Plot 2: Residuals ---
    ax2.plot(df['Time_s'], df['residual_raw'], label='||r_obs|| raw (momentum observer)', color='#d62728', linewidth=0.8, alpha=0.6)
    ax2.plot(df['Time_s'], df['residual_clipped'], label='||r_obs|| clipped (to EWMA)', color='#1f77b4', linewidth=1.2)
    ax2.set_ylabel('Residual (Nm)')
    ax2.legend(loc='upper left', fontsize=9)
    ax2.grid(True, linestyle='--', alpha=0.5)

    # --- Plot 3: EWMA Internals ---
    ax3.plot(df['Time_s'], df['EWMA_Fast'], label='Fast EWMA', color='#ff7f0e', linewidth=1)
    ax3.plot(df['Time_s'], df['EWMA_Slow'], label='Slow EWMA', color='#2ca02c', linewidth=1)
    ax3.plot(df['Time_s'], df['EWMA_Delta'], label='Delta = Fast - Slow', color='#9467bd', linewidth=1.5)
    ax3.plot(df['Time_s'], df['Threshold'], label='Dynamic Threshold', color='#d62728', linestyle='--', linewidth=2)
    ax3.set_ylabel('EWMA Values')
    ax3.legend(loc='upper left', fontsize=9)
    ax3.grid(True, linestyle='--', alpha=0.5)

    # --- Plot 4: Momentum Observer State ---
    ax4.plot(df['Time_s'], df['p_norm'], label='||p|| = ||M(q)·qd||', color='#17becf', linewidth=1)
    ax4.plot(df['Time_s'], df['r_norm'], label='||r|| = ||K·(p - p_obs)||', color='#e377c2', linewidth=1)
    ax4.set_ylabel('Momentum Observer')
    ax4.legend(loc='upper left', fontsize=9)
    ax4.grid(True, linestyle='--', alpha=0.5)

    # --- Plot 5: Context & Attack ---
    ax5.fill_between(df['Time_s'], 0, 1, where=(df['Context'] == 1), color='green', alpha=0.15, label='Payload Attached')
    attack_mask = df['Attack_Flag'] == 1
    if attack_mask.any():
        ax5.fill_between(df['Time_s'], 0, 1, where=attack_mask, color='red', alpha=0.3, label='Attack Detected')
    ax5.plot(df['Time_s'], df['qd_norm'] / df['qd_norm'].max(), label='Normalized Speed', color='#7f7f7f', linewidth=0.8)
    ax5.set_ylabel('Context / Motion')
    ax5.set_xlabel('Time (seconds)')
    ax5.set_ylim(-0.1, 1.2)
    ax5.legend(loc='upper left', fontsize=9)
    ax5.grid(True, linestyle='--', alpha=0.5)

    plt.tight_layout()
    save_path = '/workspaces/thesis/1exp.png'
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Saved full analysis to {save_path}")
    plt.show()

if __name__ == '__main__':
    plot_full_analysis()
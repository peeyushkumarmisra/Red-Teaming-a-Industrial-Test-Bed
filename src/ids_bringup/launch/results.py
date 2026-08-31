import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# CONFIG
CSV_PATH = '/workspaces/thesis/experiment_data.csv'

# Calculates the Confusion Matrix and performance metrics
def evaluate_metrics(df):  
    # Slice data for t >= 30s to strictly evaluate after the burn-in phase
    eval_df = df[df['Time_s'] >= 30.0].copy()
    
    y_true = eval_df['Ground_Truth']
    y_pred = eval_df['Attack_Flag'] # Alarm from Dual-EWMA tripwire
    
    # Confusion Matrix
    TP = ((y_true == 1) & (y_pred == 1)).sum()
    FP = ((y_true == 0) & (y_pred == 1)).sum()
    TN = ((y_true == 0) & (y_pred == 0)).sum()
    FN = ((y_true == 1) & (y_pred == 0)).sum()
    
    # Key Metrics
    TPR = TP / (TP + FN) if (TP + FN) > 0 else 0.0
    FPR = FP / (FP + TN) if (FP + TN) > 0 else 0.0
    Precision = TP / (TP + FP) if (TP + FP) > 0 else 0.0
    F1 = 2 * (Precision * TPR) / (Precision + TPR) if (Precision + TPR) > 0 else 0.0
    
    # Detection Latency (Finding rising edges of Ground_Truth to isolate attacks)
    eval_df['Attack_Start'] = (eval_df['Ground_Truth'].diff() == 1)
    attack_starts = eval_df[eval_df['Attack_Start']]['Time_s'].values
    
    latencies = []
    for t_start in attack_starts:
        # Look at a 3-second window after the attack starts
        window = eval_df[(eval_df['Time_s'] >= t_start) & (eval_df['Time_s'] <= t_start + 3.0)]
        alarms = window[window['Attack_Flag'] == 1]
        if not alarms.empty:
            t_alarm = alarms['Time_s'].iloc[0]
            latencies.append(t_alarm - t_start)
            
    avg_latency = np.mean(latencies) if latencies else 0.0
    
    # Output to Console
    print("\n" + "="*45)
    print(" EXPERIMENTAL RESULTS & METRICS")
    print("="*45)
    print(f"True Positives (TP):  {TP}")
    print(f"False Positives (FP): {FP}")
    print(f"True Negatives (TN):  {TN}")
    print(f"False Negatives (FN): {FN}")
    print("-" * 45)
    print(f"Recall / TPR:         {TPR:.4f} ({TPR*100:.2f}%)")
    print(f"False Positive Rate:  {FPR:.4f} ({FPR*100:.2f}%)")
    print(f"Precision:            {Precision:.4f} ({Precision*100:.2f}%)")
    print(f"F1-Score:             {F1:.4f}")
    print(f"Avg Detect Latency:   {avg_latency:.4f} seconds")
    print("="*45 + "\n")
    
    return df

def plot_analysis(experiment_name='exp_final'):
    if not os.path.exists(CSV_PATH):
        print(f"Error: CSV {CSV_PATH} not found. Please run the simulation first.")
        return
        
    df = pd.read_csv(CSV_PATH)
    df['Time_s'] = df['TimeStep'] * 0.01
    t_end = df['Time_s'].max()

    # Calculate and print metrics
    df = evaluate_metrics(df)

    # Plotting
    fig, axes = plt.subplots(2, 1, figsize=(14, 16), sharex=True)
    ax1, ax2 = axes
    ax1.set_title(f'Shadow Controller IDS — {experiment_name.upper()}')
    ax2.set_xlabel('Time (seconds)')

    # Plot 1 - Torques
    ax1.plot(df['Time_s'], df['tau_actual_norm'], 
             label='Actual Commanded Torque (from controller)', 
             color='#1f77b4', linewidth=1.2)
    ax1.plot(df['Time_s'], df['tau_planned_norm'], 
             label='Expected Torque from Robot Model (inverse dynamics)',
             color='#ff7f0e', linewidth=1.2, alpha=0.8)

    ax1.set_ylabel('Torque (Nm)')
    ax1.set_ylim(top=500)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, linestyle='--', alpha=0.5)

    # Plot 2 - EWMA Internals
    ax2.plot(df['Time_s'], df['EWMA_Fast'], 
             label='Fast EWMA (attack-sensitive tracker)', 
             color='#ff7f0e', linewidth=1)
    ax2.plot(df['Time_s'], df['EWMA_Slow'], 
             label='Slow EWMA (long-term baseline)', 
             color='#2ca02c', linewidth=1)
    ax2.plot(df['Time_s'], df['EWMA_Delta'], 
             label='Absolute Deviation |Fast - Slow|', 
             color='#9467bd', linewidth=1.5)
    
    # Dynamic threshold (Should now float alongside the green line)
    ax2.plot(df['Time_s'], df['Threshold'], 
             label='Dynamic Detection Threshold', 
             color='#d62728', linewidth=2, linestyle='--')

    ax2.set_ylabel('EWMA Values')
    ax2.set_ylim(top=500)
    ax2.legend(loc='upper left', fontsize=9)
    ax2.grid(True, linestyle='--', alpha=0.5)

    # Shading Plots
    for ax in [ax1, ax2]:
            # 1. Ground Truth (When the attack is actually happening)
            truth_mask = df['Ground_Truth'] == 1
            if truth_mask.any():
                ax.fill_between(df['Time_s'], 0, 1, where=truth_mask,
                                label='Ground Truth Attack Window', color="#f13838", alpha=0.2,
                                transform=ax.get_xaxis_transform())
                                
            # 2. IDS Detection (When your algorithm triggers the alarm)
            alarm_mask = df['Attack_Flag'] == 1
            if alarm_mask.any():
                ax.fill_between(df['Time_s'], 0, 1, where=alarm_mask,
                                label='IDS Detection Alarm', color="#8c564b", alpha=0.4, hatch='//',
                                transform=ax.get_xaxis_transform())
                                
            # 3. Friction degradation
            ax.axvspan(100, t_end, label='Friction Degradation Active', color="#ffa500", alpha=0.12)

    plt.tight_layout()
    save_path = f'/workspaces/thesis/{experiment_name}.png'
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Saved plot to {save_path}")
    plt.show()

if __name__ == '__main__':
    exp = sys.argv[1] if len(sys.argv) > 1 else 'exp_final'
    plot_analysis(exp)
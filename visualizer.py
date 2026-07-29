import pandas as pd
import matplotlib.pyplot as plt

def main():
    # 1. Load the data directly from CSV
    # Ensure 'experiment_data.csv' is in the same directory as this script
    df = pd.read_csv('experiment_data.csv')

    # 2. Set up the plotting environment for a Python script
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('IDS Telemetry Analysis (Standalone View)', fontsize=16, fontweight='bold')

    # --- Plot 1: Raw EKF Residual ---
    ax1.plot(df['TimeStep'], df['Residual_Norm'], color='blue', alpha=0.7, label='Raw EKF Residual')
    ax1.set_ylabel('Residual Norm')
    ax1.set_title('Physical Truth Gap (EKF Innovation)')
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right')

    # --- Plot 2: EWMA Statistic vs Active Threshold ---
    ax2.plot(df['TimeStep'], df['EWMA_Norm'], color='purple', linewidth=2, label='EWMA Smoothed Statistic')
    ax2.plot(df['TimeStep'], df['Active_Threshold'], color='red', linestyle='--', linewidth=2, label='Active Threshold')
    ax2.set_ylabel('EWMA & Threshold')
    ax2.set_title('Statistical Tripwire Evaluation')
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right')

    # --- Plot 3: Attack Detection Flag ---
    ax3.fill_between(df['TimeStep'], 0, df['Attack_Flag'], color='darkred', alpha=0.5, label='Attack Detected (1 = True)')
    ax3.set_ylabel('Alert State')
    ax3.set_xlabel('Time Step (ms)')
    ax3.set_title('IDS Alarm Status')
    ax3.set_yticks([0, 1])
    ax3.grid(True, linestyle='--', alpha=0.5)
    ax3.legend(loc='upper right')

    # 3. Format layout and display window (blocks script execution until you close the window)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
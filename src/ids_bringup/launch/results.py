import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# CONFIG
CSV_PATH = '/workspaces/thesis/experiment_data.csv'

def evaluate_metrics(df):
    """Calculates the Confusion Matrix, performance metrics, and latency per attack."""
    # Slice data for t >= 30s to strictly evaluate after the burn-in phase
    eval_df = df[df['Time_s'] >= 30.0].copy()
    
    y_true = eval_df['Ground_Truth']
    y_pred = eval_df['Attack_Flag']  # Alarm from Dual-EWMA tripwire
    
    # Confusion Matrix
    TP = int(((y_true == 1) & (y_pred == 1)).sum())
    FP = int(((y_true == 0) & (y_pred == 1)).sum())
    TN = int(((y_true == 0) & (y_pred == 0)).sum())
    FN = int(((y_true == 1) & (y_pred == 0)).sum())
    
    # Key Metrics
    TPR = TP / (TP + FN) if (TP + FN) > 0 else 0.0
    FPR = FP / (FP + TN) if (FP + TN) > 0 else 0.0
    Precision = TP / (TP + FP) if (TP + FP) > 0 else 0.0
    F1 = 2 * (Precision * TPR) / (Precision + TPR) if (Precision + TPR) > 0 else 0.0
    
    # Latency Detection (Finding rising edges of Ground_Truth)
    eval_df['Attack_Start'] = (eval_df['Ground_Truth'].diff() == 1)
    attack_starts = eval_df[eval_df['Attack_Start']]['Time_s'].values
    
    latencies = []
    attack_labels = []
    for idx, t_start in enumerate(attack_starts, 1):
        window = eval_df[(eval_df['Time_s'] >= t_start) & (eval_df['Time_s'] <= t_start + 3.0)]
        alarms = window[window['Attack_Flag'] == 1]
        if not alarms.empty:
            t_alarm = alarms['Time_s'].iloc[0]
            lat = (t_alarm - t_start) * 1000.0  # Convert to ms
            latencies.append(lat)
            attack_labels.append(f"Atk {idx}")
            
    avg_latency_s = (np.mean(latencies) / 1000.0) if latencies else 0.0
    
    # Output to Console
    print("\n" + "="*50)
    print(" EXPERIMENTAL RESULTS & METRICS")
    print("="*50)
    print(f"True Positives (TP):  {TP}")
    print(f"False Positives (FP): {FP}")
    print(f"True Negatives (TN):  {TN}")
    print(f"False Negatives (FN): {FN}")
    print("-" * 50)
    print(f"Recall / TPR:         {TPR:.4f} ({TPR*100:.2f}%)")
    print(f"False Positive Rate:  {FPR:.4f} ({FPR*100:.2f}%)")
    print(f"Precision:            {Precision:.4f} ({Precision*100:.2f}%)")
    print(f"F1-Score:             {F1:.4f}")
    print(f"Avg Detect Latency:   {avg_latency_s:.4f} seconds")
    print("="*50 + "\n")
    
    metrics = {
        'TP': TP, 'FP': FP, 'TN': TN, 'FN': FN,
        'TPR': TPR, 'FPR': FPR, 'Precision': Precision,
        'F1': F1, 'Latency_s': avg_latency_s,
        'latencies_ms': latencies,
        'attack_labels': attack_labels
    }
    
    return df, metrics

def plot_analysis(experiment_name='exp_final'):
    if not os.path.exists(CSV_PATH):
        print(f"Error: CSV {CSV_PATH} not found. Please run the simulation first.")
        return
        
    df = pd.read_csv(CSV_PATH)
    df['Time_s'] = df['TimeStep'] * 0.01
    t_end = df['Time_s'].max()

    df, m = evaluate_metrics(df)

    # Figure Layout: Top = EWMA, Bottom = [CM, Latency Bar, Text Metrics]
    fig = plt.figure(figsize=(15, 12))
    gs = GridSpec(2, 3, height_ratios=[1.8, 1.0], width_ratios=[1.1, 0.9, 1.0], hspace=0.32, wspace=0.3)

    # -------------------------------------------------------------
    # 1. TOP PANEL: EWMA Timeline Plot
    # -------------------------------------------------------------
    ax_ewma = fig.add_subplot(gs[0, :])
    ax_ewma.set_title(f'Shadow Controller IDS Timeline — {experiment_name.upper()}', fontsize=12, fontweight='bold', pad=10)
    
    ax_ewma.plot(df['Time_s'], df['EWMA_Fast'], 
                 label='Fast EWMA (attack-sensitive tracker)', 
                 color='#ff7f0e', linewidth=1)
    ax_ewma.plot(df['Time_s'], df['EWMA_Slow'], 
                 label='Slow EWMA (long-term baseline)', 
                 color='#2ca02c', linewidth=1)
    ax_ewma.plot(df['Time_s'], df['EWMA_Delta'], 
                 label='Absolute Deviation |Fast - Slow|', 
                 color='#9467bd', linewidth=1.5)
    ax_ewma.plot(df['Time_s'], df['Threshold'], 
                 label='Dynamic Detection Threshold', 
                 color='#d62728', linewidth=2, linestyle='--')

    ax_ewma.set_xlabel('Time (seconds)')
    ax_ewma.set_ylabel('EWMA Values')
    ax_ewma.set_ylim(top=500)
    ax_ewma.grid(True, linestyle='--', alpha=0.5)

    # Context, Attack, and Friction Overlays
    context_mask = df['Context'] == 1
    if context_mask.any():
        ax_ewma.fill_between(df['Time_s'], 0, 1, where=context_mask,
                             label='Payload Active (Context = 1)', color="#17becf", alpha=0.08,
                             transform=ax_ewma.get_xaxis_transform())

    truth_mask = df['Ground_Truth'] == 1
    if truth_mask.any():
        ax_ewma.fill_between(df['Time_s'], 0, 1, where=truth_mask,
                             label='Ground Truth Attack Window', color="#f13838", alpha=0.2,
                             transform=ax_ewma.get_xaxis_transform())
                        
    alarm_mask = df['Attack_Flag'] == 1
    if alarm_mask.any():
        ax_ewma.fill_between(df['Time_s'], 0, 1, where=alarm_mask,
                             label='IDS Detection Alarm', color="#8c564b", alpha=0.4, hatch='//',
                             transform=ax_ewma.get_xaxis_transform())
                        
    ax_ewma.axvspan(100, t_end, label='Friction Degradation Active', color="#ffa500", alpha=0.12)
    ax_ewma.legend(loc='upper left', fontsize=8.5, ncol=2)

    # -------------------------------------------------------------
    # 2. BOTTOM LEFT: Confusion Matrix Heatmap
    # -------------------------------------------------------------
    ax_cm = fig.add_subplot(gs[1, 0])
    cm_matrix = np.array([[m['TN'], m['FP']],
                          [m['FN'], m['TP']]])
    
    im = ax_cm.imshow(cm_matrix, interpolation='nearest', cmap=plt.cm.Blues)
    #plt.colorbar(im, ax=ax_cm, fraction=0.046, pad=0.04)
    
    ax_cm.set(xticks=np.arange(2),
              yticks=np.arange(2),
              xticklabels=['Pred Normal (0)', 'Pred Attack (1)'],
              yticklabels=['Actual Normal (0)', 'Actual Attack (1)'],
              title='Confusion Matrix ($t \\geq 30$s)')
    
    labels = [['TN', 'FP'], ['FN', 'TP']]
    thresh = cm_matrix.max() / 2.
    for i in range(2):
        for j in range(2):
            val = cm_matrix[i, j]
            color = "white" if val > thresh else "black"
            ax_cm.text(j, i, f"{labels[i][j]}\n{val:,}",
                       ha="center", va="center", color=color, fontsize=14, fontweight='bold')

    # -------------------------------------------------------------
    # 3. BOTTOM MIDDLE: Detection Latency Bar Chart
    # -------------------------------------------------------------
    ax_lat = fig.add_subplot(gs[1, 1])
    if m['latencies_ms']:
        bars = ax_lat.bar(m['attack_labels'], m['latencies_ms'], color='#3470a3', width=0.55, edgecolor='black', linewidth=0.8)
        ax_lat.set_ylabel('Latency (ms)')
        ax_lat.set_title('Detection Latency per Attack')
        ax_lat.grid(axis='y', linestyle='--', alpha=0.6)
        
        for bar in bars:
            height = bar.get_height()
            ax_lat.annotate(f'{height:.1f}ms',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontsize=9, fontweight='bold')
    else:
        ax_lat.text(0.5, 0.5, 'No Alarms Detected', ha='center', va='center')
        ax_lat.set_title('Detection Latency')

    # -------------------------------------------------------------
    # 4. BOTTOM RIGHT: Clean Performance Metric Summary
    # -------------------------------------------------------------
    ax_metrics = fig.add_subplot(gs[1, 2])
    ax_metrics.axis('off')
    
    summary_text = (
        "SYSTEM PERFORMANCE METRICS\n"
        "-----------------------------------------\n\n"
        f"• Recall (TPR):          {m['TPR']*100:.2f}%\n\n"
        f"• False Positive Rate: {m['FPR']*100:.2f}%\n\n"
        f"• Precision:             {m['Precision']*100:.2f}%\n\n"
        f"• Composite F1:          {m['F1']:.4f}\n\n"
        f"• Mean Latency:          {m['Latency_s']*1000:.2f} ms ({m['Latency_s']:.4f}s)"
    )
    
    ax_metrics.text(0.05, 0.5, summary_text,
                    fontsize=10.5, va='center', ha='left', family='monospace',
                    bbox=dict(boxstyle='round,pad=0.8', facecolor='#f8f9fa', edgecolor='#cccccc', alpha=0.9))

    save_path = f'/workspaces/thesis/{experiment_name}.png'
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Saved layout plot to {save_path}")
    plt.show()

if __name__ == '__main__':
    exp = sys.argv[1] if len(sys.argv) > 1 else 'exp_final'
    plot_analysis(exp)
"""
Time-of-Flight Sensor Measurement Analysis and Visualization

This file analyzes combined angle and distance measurement data from CSV files
and generates heatmaps to visualize measurement validity and variance across
different angles and distances.

Key features:
- Reads measurement CSV files organized by nominal distance and angle
- Calculates validity ratio (percentage of measurements with valid status codes)
- Computes variance across selected sensor zones for valid measurements
- Generates heatmaps showing variance and validity distributions
- Creates line graphs to analyze trends in performance across distances
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
import re

# --- CONFIGURATION ---
DATA_FOLDER = 'distance_and_angle_measurements/data'
# Define which zones (0-15) to include in the calculation
# SELECTED_ZONES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
# SELECTED_ZONES = [0, 1, 2, 3, 4, 7, 8, 11, 12, 13, 14, 15]
# SELECTED_ZONES = [0, 3, 12, 15]
SELECTED_ZONES = [5, 6, 9, 10] 
VALID_STATUSES = [5, 9, 10]

def calculate_sensor_stats():
    """Analyzes CSV files and returns a summary DataFrame."""
    results = []
    file_pattern = re.compile(r"(\d+)cm_(\d+)deg\.csv")

    if not os.path.exists(DATA_FOLDER):
        print(f"Error: Folder '{DATA_FOLDER}' not found.")
        return pd.DataFrame()

    for filename in os.listdir(DATA_FOLDER):
        match = file_pattern.match(filename)
        if match:
            # Extract and convert cm to mm
            dist_nominal_mm = int(match.group(1)) * 10 
            angle_nominal = int(match.group(2))
            
            df = pd.read_csv(os.path.join(DATA_FOLDER, filename))
            
            dist_cols = [col for col in df.columns if 'dist' in col.lower()]
            stat_cols = [col for col in df.columns if 'status' in col.lower()]
            
            target_dist_cols = [dist_cols[i] for i in SELECTED_ZONES]
            target_stat_cols = [stat_cols[i] for i in SELECTED_ZONES]
            
            # 1. Calculate Validity Ratio
            valid_mask = df[target_stat_cols].isin(VALID_STATUSES)
            validity_ratio = (valid_mask.sum().sum() / df[target_stat_cols].size) * 100
            
            # 2. Calculate Variance per zone for valid readings
            zone_variances = []
            for d_col, s_col in zip(target_dist_cols, target_stat_cols):
                valid_vals = df.loc[df[s_col].isin(VALID_STATUSES), d_col]
                if len(valid_vals) > 1:
                    zone_variances.append(valid_vals.var())
            
            # Combine variances (Mean Variance)
            combined_var = np.mean(zone_variances) if zone_variances else np.nan

            results.append({
                'Distance_mm': dist_nominal_mm,
                'Angle_deg': angle_nominal,
                'Variance': combined_var,
                'Validity': validity_ratio
            })

    return pd.DataFrame(results).sort_values(['Distance_mm', 'Angle_deg'])

def plot_sensor_results(summary_df):
    """Generates heatmaps and line graphs from the summary data."""
    if summary_df.empty:
        print("No data to plot.")
        return

    # --- 1. HEATMAPS ---
    noise_pivot = summary_df.pivot(index="Distance_mm", columns="Angle_deg", values="Variance")
    valid_pivot = summary_df.pivot(index="Distance_mm", columns="Angle_deg", values="Validity")

    fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    fig1.suptitle(f'ToF Sensor Heatmap (Zones: {SELECTED_ZONES})', fontsize=16)

    sns.heatmap(noise_pivot, annot=True, fmt=".2f", cmap="YlOrRd", ax=ax1)
    ax1.set_title('Noise Level (Variance in $mm^2$)')
    ax1.set_ylabel('Nominal Distance (mm)')

    sns.heatmap(valid_pivot, annot=True, fmt=".1f", cmap="RdYlGn", ax=ax2)
    ax2.set_title('Validity Ratio (%)')
    ax2.set_ylabel('Nominal Distance (mm)')

    # --- 2. LINE GRAPHS (Trend Analysis) ---
    fig2, (ax3, ax4) = plt.subplots(1, 2, figsize=(16, 6))
    sns.lineplot(data=summary_df, x='Distance_mm', y='Variance', hue='Angle_deg', marker='o', ax=ax3)
    ax3.set_title('Variance vs. Distance')
    ax3.set_ylabel('Variance ($mm^2$)')

    sns.lineplot(data=summary_df, x='Distance_mm', y='Validity', hue='Angle_deg', marker='o', ax=ax4)
    ax4.set_title('Validity vs. Distance')
    ax4.set_ylabel('Validity (%)')

    plt.show()

if __name__ == "__main__":
    df_stats = calculate_sensor_stats()
    print(df_stats)
    plot_sensor_results(df_stats)
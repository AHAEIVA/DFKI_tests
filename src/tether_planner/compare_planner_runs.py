#!/usr/bin/env python3

import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import trimesh
import math
from scipy.spatial.transform import Rotation as R

def calculate_path_length(points):
    """Calculates the total length of a path defined by a list of 3D points."""
    length = 0.0
    if len(points) < 2:
        return 0.0
    points = np.array(points)
    diffs = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    length = np.sum(segment_lengths)
    return length

def find_first_exceedance(timestamps, lengths, max_length):
    """Finds the first timestamp where length exceeds max_length."""
    for ts, length in zip(timestamps, lengths):
        if length > max_length:
            return ts
    return None # Or np.inf or similar if never exceeded

# Removed placeholder calculate_coverage function

def add_recovery_time_to_plot(ax, results1, results2, args):
    """Adds recovery time data points to the coverage plot."""
    # Recovery Time as Coverage
    max_recovery_time = args.l_max * 2  # Maximum possible recovery time
    last_time1 = results1['coverage_df']['time_elapsed'].iloc[-1] if not results1['coverage_df'].empty else 0
    last_time2 = results2['coverage_df']['time_elapsed'].iloc[-1] if not results2['coverage_df'].empty else 0
    recovery_coverage1 = ((results1['final_recovery_time'] * (1)) / max_recovery_time) * 100
    recovery_coverage2 = ((results2['final_recovery_time'] * (1)) / max_recovery_time) * 100

    # Plot the recovery time as a line segment
    ax.plot([last_time1, last_time1 + (results1['final_recovery_time'])], [results1['coverage_df']['coverage_percent'].iloc[-1], results1['coverage_df']['coverage_percent'].iloc[-1]], color='blue', linestyle='--', linewidth=2, label="CPP")
    ax.plot([last_time2, last_time2 + (results2['final_recovery_time'])], [results2['coverage_df']['coverage_percent'].iloc[-1], results2['coverage_df']['coverage_percent'].iloc[-1]], color='orange', linestyle='--', linewidth=2, label="REACT Recovery")
    
    # Add annotation for recovery time
    ax.annotate("Recovery Time", xy=(last_time1 + (results1['final_recovery_time']/2), results1['coverage_df']['coverage_percent'].iloc[-1] + 2), xycoords='data', ha='center', fontsize=12, color='blue')
    ax.annotate("Recovery Time", xy=(last_time2 + (results2['final_recovery_time']/2), results2['coverage_df']['coverage_percent'].iloc[-1] - 4), xycoords='data', ha='center', fontsize=12, color='orange')

    # Extend inspection time with horizontal line
    ax.hlines(y=results1['coverage_df']['coverage_percent'].iloc[-1], xmin=last_time1, xmax=last_time1 + (results1['final_recovery_time']), color='blue', linestyle='-', linewidth=2)
    ax.hlines(y=results2['coverage_df']['coverage_percent'].iloc[-1], xmin=last_time2, xmax=last_time2 + (results2['final_recovery_time']), color='orange', linestyle='-', linewidth=2)

def process_planner_data(data_dir, planner_name, l_max, env_mesh):
    """Loads data, calculates metrics for a single planner run."""
    print(f"\n--- Processing Planner: {planner_name} ---")
    print(f"Data directory: {data_dir}")

    # --- Load Data ---
    try:
        odom_file = next(os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.endswith('_odom.csv'))
        orient_file = next(os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.endswith('_orientation.csv'))
        tether_file = next(os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.endswith('_tether.csv'))
        coverage_file = os.path.join(data_dir, 'coverage_data.csv') # Path for coverage file
        print(f"Found Odom file: {odom_file}")
        print(f"Found Orient file: {orient_file}")
        print(f"Found Tether file: {tether_file}")
        print(f"Looking for Coverage file: {coverage_file}")
    except StopIteration:
        print(f"Error: Could not find required CSV files in {data_dir}")
        return None
    except Exception as e:
        print(f"Error finding files in {data_dir}: {e}")
        return None

    try:
        odom_df = pd.read_csv(odom_file)
        orient_df = pd.read_csv(orient_file)
        tether_df = pd.read_csv(tether_file)
        # Load coverage data
        if os.path.exists(coverage_file):
            coverage_df = pd.read_csv(coverage_file)
            print(f"Loaded Coverage file: {coverage_file}")
        else:
            print(f"Error: Coverage data file not found - {coverage_file}")
            print("Please run simulate_trajectory.py first for this data directory to generate coverage_data.csv")
            return None
    except Exception as e:
        print(f"Error reading CSV files: {e}")
        return None

    # Convert timestamps if they are not already numeric seconds (excluding coverage_df as it should have time_elapsed)
    for df in [odom_df, orient_df, tether_df]:
        if 'timestamp' in df.columns and not pd.api.types.is_numeric_dtype(df['timestamp']):
             # Attempt conversion assuming a common format, adjust if needed
             try:
                 df['timestamp'] = pd.to_datetime(df['timestamp']).astype(np.int64) // 10**9
             except Exception as e:
                 print(f"Error converting timestamp column: {e}. Ensure it's numeric seconds.")
                 return None

    # --- Calculate Metrics ---
    # Total Time
    odom_df = odom_df.sort_values('timestamp')
    start_time = odom_df['timestamp'].min()
    end_time = odom_df['timestamp'].max()
    total_time = end_time - start_time
    print(f"Total trajectory time: {total_time:.2f} s")

    # Tether Length vs. Time
    tether_lengths_data = []
    tether_df = tether_df.sort_values(['timestamp', 'point_index'])
    unique_tether_ts = tether_df['timestamp'].unique()
    for ts in unique_tether_ts:
        points = tether_df[tether_df['timestamp'] == ts][['x', 'y', 'z']].values
        length = calculate_path_length(points)
        tether_lengths_data.append({'timestamp': ts, 'tether_length': length})

    tether_length_df = pd.DataFrame(tether_lengths_data)
    tether_length_df['time_elapsed'] = tether_length_df['timestamp'] - start_time
    print(f"Calculated tether length for {len(tether_length_df)} points.")


    # Time Max Tether Exceeded
    time_exceeded = find_first_exceedance(
        tether_length_df['timestamp'],
        tether_length_df['tether_length'],
        l_max
    )
    time_exceeded_elapsed = (time_exceeded - start_time) if time_exceeded is not None else None
    print(f"Time max tether length ({l_max}m) first exceeded: {time_exceeded_elapsed:.2f} s" if time_exceeded_elapsed is not None else "Max tether length never exceeded.")

    # Coverage vs. Time
    # Coverage vs. Time (Loaded from file)
    # Ensure coverage_df has the expected columns
    if not all(col in coverage_df.columns for col in ['time_elapsed', 'coverage_percent']):
        print(f"Error: coverage_data.csv in {data_dir} is missing required columns ('time_elapsed', 'coverage_percent').")
        return None
    coverage_df = coverage_df.sort_values('time_elapsed')
    final_coverage = coverage_df['coverage_percent'].iloc[-1] if not coverage_df.empty else 0
    print(f"Final coverage (from file): {final_coverage:.2f}%")

    # Get final tether length
    final_tether_length = tether_length_df['tether_length'].iloc[-1] if not tether_length_df.empty else 0
    print(f"Final tether length: {final_tether_length:.2f} m")

    # Calculate final recovery time (tether length * 10)
    final_recovery_time = final_tether_length * 10
    print(f"Final recovery time: {final_recovery_time:.2f} s")

    # --- Consolidate Results ---
    results = {
        'planner_name': "CPP",
        'total_time_s': total_time,
        'time_max_tether_exceeded_s': time_exceeded_elapsed,
        'final_coverage_percent': final_coverage,
        'tether_length_df': tether_length_df, # Includes 'time_elapsed' and 'tether_length'
        'coverage_df': coverage_df, # Includes 'time_elapsed' and 'coverage_percent'
        'final_recovery_time': final_recovery_time
    }
    return results


def main():
    parser = argparse.ArgumentParser(description="Compare planner runs based on CSV data.")
    parser.add_argument('--data-dir1', required=True, help="Directory containing CSV data for planner 1.")
    parser.add_argument('--name1', required=True, help="Name of planner 1.")
    parser.add_argument('--data-dir2', required=True, help="Directory containing CSV data for planner 2.")
    parser.add_argument('--name2', required=True, help="Name of planner 2.")
    parser.add_argument('--l-max', type=float, required=True, help="Maximum allowed tether length.")
    parser.add_argument('--stl-path', required=True, help="Path to the environment STL file.")
    parser.add_argument('--output-dir', default='results/comparison', help="Directory to save results.")
    args = parser.parse_args()

    # --- Setup ---
    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Output directory: {args.output_dir}")

    # Load Environment STL
    try:
        env_mesh = trimesh.load_mesh(args.stl_path)
        print(f"Loaded environment STL: {args.stl_path}, Area: {env_mesh.area:.2f}")
    except Exception as e:
        print(f"Error loading environment STL {args.stl_path}: {e}")
        return

    # --- Process Data ---
    results1 = process_planner_data(args.data_dir1, args.name1, args.l_max, env_mesh)
    results2 = process_planner_data(args.data_dir2, "REACT", args.l_max, env_mesh)

    if results1 is None or results2 is None:
        print("Processing failed for one or both planners. Exiting.")
        return

    # --- Generate Table ---
    summary_data = [
        {'Planner': "CPP",
         'Total Time (s)': f"{results1['total_time_s']:.2f}",
         'Time Max Tether Exceeded (s)': f"{results1['time_max_tether_exceeded_s']:.2f}" if results1['time_max_tether_exceeded_s'] is not None else "N/A",
         'Final Coverage (%)': f"{results1['final_coverage_percent']:.2f}"},
        {'Planner': "REACT",
         'Total Time (s)': f"{results2['total_time_s']:.2f}",
         'Time Max Tether Exceeded (s)': f"{results2['time_max_tether_exceeded_s']:.2f}" if results2['time_max_tether_exceeded_s'] is not None else "N/A",
         'Final Coverage (%)': f"{results2['final_coverage_percent']:.2f}"}
    ]
    summary_df = pd.DataFrame(summary_data)
    table_path = os.path.join(args.output_dir, 'comparison_metrics.csv')
    summary_df.to_csv(table_path, index=False)
    print(f"\nComparison table saved to: {table_path}")
    print(summary_df.to_string(index=False))

    # --- Generate Plots ---
    # plt.style.use('seaborn-v0_8-darkgrid') # Use a nice style (Removed for compatibility)

    # Coverage Plot
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(results1['coverage_df']['time_elapsed'].values, results1['coverage_df']['coverage_percent'].values, label=results1['planner_name'], linewidth=5.0) # Doubled linewidth
    ax1.plot(results2['coverage_df']['time_elapsed'].values, results2['coverage_df']['coverage_percent'].values, label=results2['planner_name'], linewidth=5.0) # Doubled linewidth
    ax1.set_xlabel("Time Elapsed (s)", fontsize=40) # Doubled font size
    ax1.set_ylabel("Coverage (%)", fontsize=40) # Doubled font size
    # ax1.set_title("Coverage vs. Time", fontsize=24) # Removed title
    ax1.legend(fontsize=32) # Doubled font size
    ax1.tick_params(axis='both', which='major', labelsize=42) # Doubled tick label size (21 * 2)
    ax1.grid(True)
    
    add_recovery_time_to_plot(ax1, results1, results2, args)

    coverage_plot_path = os.path.join(args.output_dir, 'coverage_vs_time.pdf') # Changed extension to pdf
    fig1.savefig(coverage_plot_path, format='pdf', bbox_inches='tight', pad_inches=0) # Added tight bbox and zero padding
    print(f"Coverage plot saved to: {coverage_plot_path}")

    # Tether Length Plot
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.plot(results1['tether_length_df']['time_elapsed'].values, results1['tether_length_df']['tether_length'].values, label=results1['planner_name'], linewidth=5.0) # Doubled linewidth
    ax2.plot(results2['tether_length_df']['time_elapsed'].values, results2['tether_length_df']['tether_length'].values, label=results2['planner_name'], linewidth=5.0) # Doubled linewidth
    ax2.axhline(y=args.l_max, color='r', linestyle='--', label='$L_{max}$', linewidth=4.0) # Changed label to only L_max
    ax2.set_xlabel("Time Elapsed (s)", fontsize=40) # Doubled font size
    ax2.set_ylabel("Tether Length (m)", fontsize=40) # Doubled font size
    # ax2.set_title("Tether Length vs. Time", fontsize=24) # Removed title
    ax2.legend(fontsize=32) # Doubled font size
    ax2.tick_params(axis='both', which='major', labelsize=42) # Doubled tick label size (21 * 2)
    ax2.grid(True)
    tether_plot_path = os.path.join(args.output_dir, 'tether_length_vs_time.pdf') # Changed extension to pdf
    fig2.savefig(tether_plot_path, format='pdf', bbox_inches='tight', pad_inches=0) # Added tight bbox and zero padding
    print(f"Tether length plot saved to: {tether_plot_path}")

    # plt.show() # Optionally display plots interactively

    print("\nComparison script finished.")

if __name__ == "__main__":
    main()

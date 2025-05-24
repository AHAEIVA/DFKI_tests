#!/usr/bin/env python3

import csv # Import csv module
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection # Import Poly3DCollection
from stl import mesh # STL library import uncommented
import sys
import os

# --- Configuration ---
L_MAX = 9.0
ODOM_FILE_REL = 'results/odom_subset.csv' # Use the subset CSV file
TETHER_PATH_FILE_REL = 'results/tether_path_subset.csv' # Use the subset CSV file
STL_FILE_REL = 'models/pipe_simple.stl' # Use pipe_simple.stl from models directory
STL_SCALE = 0.1 # Scale factor from tether_planner_main.cpp (scale_factor)
STL_TRANSLATION = np.array([0.0, 0.0, 4.5]) # Translation matching point cloud/waypoints in tether_planner_main.cpp (translation)
# Placeholder for the baseline data file
# BASELINE_ODOM_FILE_REL = 'results/normal_coverage_planner_odom.txt'

# Get absolute paths
script_dir = os.path.dirname(os.path.abspath(__file__))
odom_file_abs = os.path.join(script_dir, ODOM_FILE_REL)
tether_path_file_abs = os.path.join(script_dir, TETHER_PATH_FILE_REL)
stl_file_abs = os.path.join(script_dir, STL_FILE_REL) # STL path added

print(f"Analyzing Entanglement-Aware Planner Data from CSV Subsets")
print(f"Odometry file: {odom_file_abs}")
print(f"Tether path file: {tether_path_file_abs}")
print(f"STL file: {stl_file_abs}") # Print STL file path
print(f"Max Tether Length (L_max): {L_MAX}")

# --- Helper Functions ---

# Removed parse_yaml_stream function

def calculate_path_length(points):
    """Calculates the total length of a path defined by a list of 3D points."""
    length = 0.0
    if len(points) < 2:
        return 0.0
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i+1]
        dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
        length += dist
    return length

def get_timestamp(msg):
    """Extracts timestamp in seconds from ROS message header."""
    if 'header' in msg and 'stamp' in msg['header']:
        return msg['header']['stamp']['secs'] + msg['header']['stamp']['nsecs'] * 1e-9
    return None # Or raise an error

# --- Main Analysis Logic ---

# 1. Parse Odometry Data (Entanglement-Aware) from CSV
print("\nParsing Odometry data from CSV...")
rov_trajectory = [] # List of (timestamp, x, y, z)
try:
    with open(odom_file_abs, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader) # Skip header
        for row in reader:
            try:
                # Expecting timestamp, x, y, z
                if len(row) == 4:
                    ts = float(row[0])
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[3])
                    rov_trajectory.append((ts, x, y, z))
                else:
                    print(f"Warning: Skipping malformed odometry row: {row}", file=sys.stderr)
            except ValueError as e:
                 print(f"Warning: Skipping odometry row due to conversion error ({e}): {row}", file=sys.stderr)
except FileNotFoundError:
    print(f"Error: Odometry CSV file not found - {odom_file_abs}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred reading {odom_file_abs}: {e}", file=sys.stderr)
    sys.exit(1)

if not rov_trajectory:
    print("Error: No valid ROV trajectory data parsed from CSV.", file=sys.stderr)
    sys.exit(1)

# rov_trajectory is likely already sorted by timestamp from preprocessing, but sorting again doesn't hurt
rov_trajectory.sort()
print(f"Parsed {len(rov_trajectory)} valid ROV trajectory points from CSV.")


# 2. Parse Tether Path Data (Entanglement-Aware) from CSV
print("\nParsing Tether Path data from CSV...")
tether_paths = {} # Dict: {timestamp: [(x, y, z), ...]}
try:
    with open(tether_path_file_abs, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader) # Skip header
        current_ts = None
        current_points = []
        for row in reader:
            try:
                 # Expecting timestamp, point_index, x, y, z
                if len(row) == 5:
                    ts = float(row[0])
                    # point_index = int(row[1]) # Not strictly needed for length calc
                    x = float(row[2])
                    y = float(row[3])
                    z = float(row[4])

                    # Group points by timestamp
                    if ts != current_ts:
                        if current_points: # Store previous path if exists
                            tether_paths[current_ts] = current_points
                        current_ts = ts
                        current_points = []
                    current_points.append((x, y, z))

                else:
                    print(f"Warning: Skipping malformed tether path row: {row}", file=sys.stderr)
            except ValueError as e:
                 print(f"Warning: Skipping tether path row due to conversion error ({e}): {row}", file=sys.stderr)

        # Store the last path segment
        if current_points and current_ts is not None:
             tether_paths[current_ts] = current_points

except FileNotFoundError:
    print(f"Error: Tether path CSV file not found - {tether_path_file_abs}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred reading {tether_path_file_abs}: {e}", file=sys.stderr)
    sys.exit(1)

if not tether_paths:
    print("Error: No valid tether path data parsed from CSV.", file=sys.stderr)
    sys.exit(1)

print(f"Parsed {len(tether_paths)} unique tether path timestamps from CSV.")

# 3. Calculate Tether Length and Exceedance Time
print("\nCalculating tether lengths and exceedance time...")
tether_lengths = {} # {timestamp: length}
exceedance_duration = 0.0
last_ts = None
sorted_tether_timestamps = sorted(tether_paths.keys())

for i, ts in enumerate(sorted_tether_timestamps):
    path_points = tether_paths[ts]
    current_length = calculate_path_length(path_points)
    tether_lengths[ts] = current_length

    # Calculate time delta for exceedance
    if current_length > L_MAX:
        if i > 0: # Need a previous point to calculate duration
            prev_ts = sorted_tether_timestamps[i-1]
            time_delta = ts - prev_ts
            # Basic assumption: if length > L_max now, it was likely exceeding since the last measurement
            exceedance_duration += time_delta
        # else: # First point is already exceeding - cannot determine duration before it
           # pass

print(f"\n--- Entanglement-Aware Planner Results ---")
print(f"Total time tether length exceeded {L_MAX}m: {exceedance_duration:.4f} seconds")

# 4. Plotting (Entanglement-Aware Trajectory)
print("\nGenerating plot...")
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Subsample trajectory for plotting (take every 10th point)
plot_step = 10
rov_trajectory_subset = rov_trajectory[::plot_step]
print(f"Subsampling trajectory for plotting (1 in {plot_step}), {len(rov_trajectory_subset)} points.")

# Extract trajectory points for plotting
times, x_coords, y_coords, z_coords = zip(*rov_trajectory_subset)
ax.plot(x_coords, y_coords, z_coords, label=f'Entanglement-Aware ROV Trajectory (1/{plot_step})', color='blue', marker='.', linestyle='-', markersize=1) # Added markers

# --- Placeholder for Baseline Planner Analysis ---
# print("\n--- Baseline Planner Results ---")
# 1. Parse Baseline Odometry Data
# baseline_rov_trajectory = parse_odom_data(BASELINE_ODOM_FILE_REL)
# 2. Parse Baseline Tether Path Data (if available, format?)
# baseline_tether_paths = parse_tether_data(...)
# 3. Calculate Baseline Exceedance Time
# baseline_exceedance_duration = calculate_exceedance(baseline_tether_paths, L_MAX)
# print(f"Baseline total time tether length exceeded {L_MAX}m: {baseline_exceedance_duration:.4f} seconds")
# 4. Add Baseline Trajectory to Plot
# baseline_times, baseline_x, baseline_y, baseline_z = zip(*baseline_rov_trajectory)
# ax.plot(baseline_x, baseline_y, baseline_z, label='Baseline ROV Trajectory', color='red', linestyle='--')
# --- End Placeholder ---

ax.set_xlabel('X coordinate (m)')
ax.set_ylabel('Y coordinate (m)')
ax.set_zlabel('Z coordinate (m)')
ax.set_title('ROV Trajectory') # Updated title
ax.legend()
ax.grid(True)
# Optional: Set axis limits if needed
# ax.set_xlim([...])
# ax.set_ylim([...])
# ax.set_zlim([...])

# 5. Load and Plot STL Model
print(f"\nLoading STL model from {stl_file_abs}...")
try:
    # Load the STL mesh
    your_mesh = mesh.Mesh.from_file(stl_file_abs)

    # Scale and translate the mesh
    vertices = your_mesh.vectors * STL_SCALE + STL_TRANSLATION

    # Create the Poly3DCollection
    mesh_collection = Poly3DCollection(vertices)
    mesh_collection.set_edgecolor('k') # Optional: set edge color
    mesh_collection.set_facecolor('cyan') # Changed face color
    mesh_collection.set_alpha(0.7) # Increased transparency

    # Add the mesh to the first plot
    print("Adding STL model to trajectory plot (ax)...")
    ax.add_collection3d(mesh_collection)

    # Auto-scale axes to fit the mesh and trajectory data
    all_x = np.concatenate([x_coords, vertices[:,:,0].flatten()])
    all_y = np.concatenate([y_coords, vertices[:,:,1].flatten()])
    all_z = np.concatenate([z_coords, vertices[:,:,2].flatten()])
    ax.set_xlim(all_x.min(), all_x.max())
    ax.set_ylim(all_y.min(), all_y.max())
    ax.set_zlim(all_z.min(), all_z.max())
    print("Adjusted trajectory plot limits for STL.")

except FileNotFoundError:
    print(f"Error: STL file not found at {stl_file_abs}", file=sys.stderr)
    your_mesh = None # Ensure variable exists but is None
except Exception as e:
    print(f"Error loading or processing STL file: {e}", file=sys.stderr)
    your_mesh = None # Ensure variable exists but is None


# 6. Create Second Plot: Tether Snapshots
print("\nGenerating tether snapshots plot...")
fig2 = plt.figure(figsize=(10, 8))
ax2 = fig2.add_subplot(111, projection='3d')

# Plot STL model again on the second figure (if loaded successfully)
if your_mesh is not None:
    print("Adding STL model to tether snapshots plot (ax2)...")
    # Recreate collection for the second axes (important!)
    mesh_collection2 = Poly3DCollection(vertices) # Use same scaled/translated vertices
    mesh_collection2.set_edgecolor('k')
    mesh_collection2.set_facecolor('cyan') # Changed face color
    mesh_collection2.set_alpha(0.7) # Increased transparency
    ax2.add_collection3d(mesh_collection2)
    print("Added STL to second plot.")
    # Note: Axis limits for the second plot are handled later after plotting snapshots

# Select 10 timestamps for tether snapshots
num_snapshots = 10
snapshot_indices = np.linspace(0, len(sorted_tether_timestamps) - 1, num_snapshots, dtype=int)
snapshot_timestamps = [sorted_tether_timestamps[i] for i in snapshot_indices]

print(f"Plotting tether snapshots for timestamps: {snapshot_timestamps}")

# Plot tether paths for selected timestamps
colors = plt.cm.viridis(np.linspace(0, 1, num_snapshots)) # Use a colormap

for i, ts in enumerate(snapshot_timestamps):
    if ts in tether_paths:
        path_points = np.array(tether_paths[ts])
        if path_points.shape[0] > 0: # Check if path has points
             ax2.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2],
                     label=f'Tether @ {ts:.2f}s', color=colors[i], marker='o', markersize=2, linestyle='-')
        else:
             print(f"Warning: Empty tether path for timestamp {ts}", file=sys.stderr)

ax2.set_xlabel('X coordinate (m)')
ax2.set_ylabel('Y coordinate (m)')
ax2.set_zlabel('Z coordinate (m)')
ax2.set_title('Tether Path Snapshots') # Updated title
ax2.legend(fontsize='small') # Adjust legend size if needed
ax2.grid(True)

# Try to set equal aspect ratio for the second plot, considering STL
try:
    print("Adjusting tether plot limits for STL and snapshots...")
    # Collect all points from snapshots and STL for limit calculation
    all_x2 = []
    all_y2 = []
    all_z2 = []
    if your_mesh is not None:
        all_x2.append(vertices[:,:,0].flatten())
        all_y2.append(vertices[:,:,1].flatten())
        all_z2.append(vertices[:,:,2].flatten())

    for ts in snapshot_timestamps:
        if ts in tether_paths:
            path_points = np.array(tether_paths[ts])
            if path_points.shape[0] > 0:
                all_x2.append(path_points[:, 0])
                all_y2.append(path_points[:, 1])
                all_z2.append(path_points[:, 2])

    if all_x2: # Check if we have any points at all
        all_x2 = np.concatenate(all_x2)
        all_y2 = np.concatenate(all_y2)
        all_z2 = np.concatenate(all_z2)

        # Calculate ranges and center
        x_min, x_max = all_x2.min(), all_x2.max()
        y_min, y_max = all_y2.min(), all_y2.max()
        z_min, z_max = all_z2.min(), all_z2.max()

        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        z_center = (z_min + z_max) / 2

        # Calculate the largest range
        max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
        plot_radius = max_range / 2.0

        # Set limits centered around the data with equal range
        ax2.set_xlim(x_center - plot_radius, x_center + plot_radius)
        ax2.set_ylim(y_center - plot_radius, y_center + plot_radius)
        ax2.set_zlim(z_center - plot_radius, z_center + plot_radius)
        print("Set equal aspect ratio limits for tether plot.")
    else:
        print("Warning: No points found to set limits for the second plot.", file=sys.stderr)

except Exception as e:
     print(f"Could not set equal aspect ratio for tether plot: {e}", file=sys.stderr)


plt.tight_layout()
plt.show() # Show both figures

print("\nScript finished.")

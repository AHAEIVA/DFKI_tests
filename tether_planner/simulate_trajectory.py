#!/usr/bin/env python3

# Explicitly set the backend *before* importing pyplot
import matplotlib
matplotlib.use('Qt5Agg') # Use Qt5Agg for better interactivity

import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection as Poly3DCollection_3d # Alias to avoid name clash
from matplotlib.collections import PolyCollection # Import for 2D polygons
from matplotlib.ticker import MaxNLocator # Import MaxNLocator
from stl import mesh
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R # Import Rotation
import sys
import os
import math # For FOV calculations
import argparse
import glob
import pandas as pd # Added pandas for saving coverage

# --- Configuration ---
# Input file paths are now determined by arguments
ENV_STL_FILE_REL = 'models/pipe_simple.stl' # Renamed for clarity
ENV_STL_SCALE = 0.1
ENV_STL_TRANSLATION = np.array([0.0, 0.0, 4.5])
ROV_STL_FILE_REL = 'models/saab_simple.stl' # Added ROV STL
ROV_STL_SCALE = 0.002 # Scaled down further 20x (total 1000x)
# Apply initial 90-degree yaw rotation to align ROV model's front with X-axis
ROV_INITIAL_ROT = R.from_euler('z', 90, degrees=True)

ANIMATION_INTERVAL_MS = 0.001 # Delay between frames (keep low for max speed)
FRAME_STEP = 5 # Process every Nth frame to speed up animation (Increased from 15)
AXIS_LENGTH = 1.0 # Length of the orientation axes to draw
# EULER_ANGLES_IN_DEGREES = True # No longer needed, using quaternions directly

# Camera Parameters
CAM_FOV_H_DEG = 80.0 # Horizontal Field of View (degrees)
CAM_FOV_V_DEG = 60.0 # Vertical Field of View (degrees)
FOV_VIS_RANGE = 3.0 # How far the visualized FOV cone extends (meters) & Max inspection distance
CAM_OFFSET_POS = np.array([0.0, 0.0, 0.0]) # Position relative to ROV frame
CAM_OFFSET_QUAT = R.from_euler('zyx', [0, 0, 0], degrees=True).as_quat() # Orientation relative to ROV frame (x-forward)

# Colors & Alphas for Reconstruction Effect
COLOR_UNINSPECTED = (0.5, 0.5, 0.5, 0.05) # Very transparent gray
COLOR_INSPECTED = (0.0, 0.8, 0.8, 0.5)   # Cyan, medium alpha
COLOR_CURRENTLY_VISIBLE = (0.0, 1.0, 0.0, 0.8) # Lime green, high alpha
COLOR_FOV_VOLUME = 'yellow'
ALPHA_FOV_VOLUME = 0.15 # Transparency for FOV volume

# Video Saving Parameters (Enabled for saving)
VIDEO_FPS = 30
VIDEO_DPI = 150
OUTPUT_VIDEO_FILENAME = 'simulation_animation.mp4' # Define output filename

# --- Argument Parsing ---
parser = argparse.ArgumentParser(description='Simulate ROV trajectory and tether path with reconstruction highlighting.')
parser.add_argument('--data-dir', type=str, required=True,
                    help='Directory containing the run data (odom_*.csv, orientation_*.csv, tether_path_*.csv)')
# parser.add_argument('--save-video', action='store_true', help='Save animation to MP4 file instead of displaying.') # Keep commented out
parser.add_argument('--save-final-frame', type=str, default=None,
                    help='Save only the final frame of the simulation to the specified PDF file path.')
parser.add_argument('--final-frame-only', action='store_true',
                    help='Only generate and save the final frame (requires --save-final-frame). Skips animation.')
args = parser.parse_args()

if args.final_frame_only and not args.save_final_frame:
    parser.error("--final-frame-only requires --save-final-frame to be set.")

# --- Setup ---
# Get absolute paths
script_dir = os.path.dirname(os.path.abspath(__file__))
data_dir_abs = os.path.join(script_dir, args.data_dir)
env_stl_file_abs = os.path.join(script_dir, ENV_STL_FILE_REL) # Renamed
rov_stl_file_abs = os.path.join(script_dir, ROV_STL_FILE_REL) # Added

# Find data files in the specified directory using more general patterns
odom_files = glob.glob(os.path.join(data_dir_abs, '*_odom.csv'))
orient_files = glob.glob(os.path.join(data_dir_abs, '*_orientation.csv'))
tether_files = glob.glob(os.path.join(data_dir_abs, '*_tether.csv')) # Match files like 'planner_tether.csv'

if not odom_files: print(f"Error: No *_odom.csv file found in {data_dir_abs}", file=sys.stderr); sys.exit(1)
if not orient_files: print(f"Error: No *_orientation.csv file found in {data_dir_abs}", file=sys.stderr); sys.exit(1)
if not tether_files: print(f"Error: No *_tether.csv file found in {data_dir_abs}", file=sys.stderr); sys.exit(1)

# Use the first found file for each type (assuming one per run directory)
odom_data_file_abs = odom_files[0]
orient_data_file_abs = orient_files[0] # Restored
tether_path_file_abs = tether_files[0]

print("Starting ROV Trajectory Simulation with Reconstruction Highlighting")
print(f"Data directory: {data_dir_abs}")
print(f"Odometry file (Position): {odom_data_file_abs}") # Updated print
print(f"Orientation file (Euler Degrees): {orient_data_file_abs}") # Corrected unit
print(f"Tether path file: {tether_path_file_abs}")
print(f"Environment STL file: {env_stl_file_abs}")
print(f"ROV STL file: {rov_stl_file_abs}")
print(f"Animating every {FRAME_STEP}th frame.")
print(f"Camera FOV (H/V): {CAM_FOV_H_DEG}° / {CAM_FOV_V_DEG}°")
print(f"Max Inspection Range: {FOV_VIS_RANGE} m")
# print("Displaying interactive plot.") # Commented out, saving instead
print(f"Saving animation to {OUTPUT_VIDEO_FILENAME} in data directory.")


# --- Helper Functions ---
def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0: return v
    return v / norm

def angle_between_vectors(v1, v2):
    v1_u = normalize_vector(v1); v2_u = normalize_vector(v2)
    dot_product = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.arccos(dot_product)

# --- Data Loading ---
# 1. Load Environment STL Model
print(f"\nLoading Environment STL model from {env_stl_file_abs}...")
try:
    env_mesh_obj = mesh.Mesh.from_file(env_stl_file_abs)
    env_mesh_vectors_scaled = env_mesh_obj.vectors * ENV_STL_SCALE
    env_stl_vertices = env_mesh_vectors_scaled + ENV_STL_TRANSLATION
    env_stl_normals = np.zeros_like(env_mesh_obj.normals)
    for i, facet in enumerate(env_stl_vertices):
        v0, v1, v2 = facet; normal = np.cross(v1 - v0, v2 - v0)
        norm = np.linalg.norm(normal)
        if norm > 1e-9: env_stl_normals[i] = normal / norm
    print(f"Environment STL model loaded and transformed. {len(env_stl_vertices)} triangles.")
except FileNotFoundError: print(f"Error: Environment STL file not found at {env_stl_file_abs}", file=sys.stderr); sys.exit(1)
except Exception as e: print(f"Error loading or processing Environment STL file: {e}", file=sys.stderr); sys.exit(1)

# 1b. Load ROV STL Model
print(f"\nLoading ROV STL model from {rov_stl_file_abs}...")
try:
    rov_mesh_obj = mesh.Mesh.from_file(rov_stl_file_abs)
    # Apply scaling
    rov_scaled_vertices = rov_mesh_obj.vectors * ROV_STL_SCALE
    # Apply initial rotation
    if 'ROV_INITIAL_ROT' in globals():
        print(f"Applying initial ROV rotation: {ROV_INITIAL_ROT.as_euler('zyx', degrees=True)}")
        # Reshape for rotation: (N_triangles * 3_vertices, 3_coords)
        flat_vertices = rov_scaled_vertices.reshape(-1, 3)
        rotated_flat = ROV_INITIAL_ROT.apply(flat_vertices)
        # Reshape back: (N_triangles, 3_vertices, 3_coords)
        rov_base_vertices = rotated_flat.reshape(rov_scaled_vertices.shape) # Store rotated vertices
    else:
        rov_base_vertices = rov_scaled_vertices # Use scaled if no rotation

    print(f"ROV STL model loaded, scaled, and initially rotated. {len(rov_base_vertices)} triangles.")
except FileNotFoundError: print(f"Error: ROV STL file not found at {rov_stl_file_abs}", file=sys.stderr); sys.exit(1)
except Exception as e: print(f"Error loading or processing ROV STL file: {e}", file=sys.stderr); sys.exit(1)


# 2. Load Odometry Data (Position Only)
print(f"\nParsing odometry data (Position) from {odom_data_file_abs}...")
all_rov_pos_data = [] # List of [ts_sec, x, y, z]
try:
    with open(odom_data_file_abs, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader)
        # Check if the header *starts* with the expected columns
        expected_header_start = ['timestamp', 'x', 'y', 'z']
        if len(header) < 4 or header[:4] != expected_header_start:
             print(f"Error: Odom CSV header '{header}' does not start with expected columns '{expected_header_start}'", file=sys.stderr); sys.exit(1)
        print(f"Odom header found: {header}. Reading first 4 columns.") # Info message
        for row_idx, row in enumerate(reader):
            try:
                # Read only the first 4 columns, ignore the rest
                if len(row) >= 4:
                    # ts, x, y, z
                    data_point = [float(val) for val in row[:4]] # Take only the first 4 values
                    all_rov_pos_data.append(data_point)
                else: print(f"Warning: Skipping malformed odom data row #{row_idx+1} (expected at least 4 columns, got {len(row)})", file=sys.stderr)
            except ValueError as e: print(f"Warning: Skipping odom data row #{row_idx+1} due to conversion error ({e}) in first 4 columns", file=sys.stderr)
except FileNotFoundError: print(f"Error: Odometry data CSV file not found - {odom_data_file_abs}", file=sys.stderr); sys.exit(1)
except Exception as e: print(f"An unexpected error occurred reading {odom_data_file_abs}: {e}", file=sys.stderr); sys.exit(1)
if not all_rov_pos_data: print("Error: No valid odometry position data parsed from CSV.", file=sys.stderr); sys.exit(1)
print(f"Parsed {len(all_rov_pos_data)} odometry position points.")

# 3. Load Orientation Data (Euler Degrees)
print(f"\nParsing orientation data (Euler Degrees) from {orient_data_file_abs}...")
orientation_data = {} # {timestamp_ns: [roll_deg, pitch_deg, yaw_deg]}
try:
    with open(orient_data_file_abs, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile); header = next(reader)
        # Expect header with degrees
        expected_header = ['timestamp', 'roll_deg', 'pitch_deg', 'yaw_deg']
        if header != expected_header: print(f"Error: Orientation CSV header '{header}' != '{expected_header}'", file=sys.stderr); sys.exit(1)
        for row_idx, row in enumerate(reader):
            try:
                if len(row) == 4:
                    ts_sec, roll_deg, pitch_deg, yaw_deg = [float(val) for val in row]
                    ts_ns = int(ts_sec * 1e9)
                    orientation_data[ts_ns] = [roll_deg, pitch_deg, yaw_deg] # Store degrees
                else: print(f"Warning: Skipping malformed orientation data row #{row_idx+1} (expected 4 columns, got {len(row)})", file=sys.stderr)
            except ValueError as e: print(f"Warning: Skipping orientation data row #{row_idx+1} due to conversion error ({e})", file=sys.stderr)
except FileNotFoundError: print(f"Error: Orientation data CSV file not found - {orient_data_file_abs}", file=sys.stderr); sys.exit(1)
except Exception as e: print(f"An unexpected error occurred reading {orient_data_file_abs}: {e}", file=sys.stderr); sys.exit(1)
if not orientation_data: print("Error: No valid orientation data parsed from CSV.", file=sys.stderr); sys.exit(1)
sorted_orientation_ts_ns = np.array(sorted(orientation_data.keys()))
print(f"Parsed {len(orientation_data)} unique orientation timestamps.")

# --- Helper Function for Timestamp Matching (Orientation) ---
def find_nearest_orientation_ts_ns(target_ts_ns):
    if not sorted_orientation_ts_ns.size: return None
    idx = np.searchsorted(sorted_orientation_ts_ns, target_ts_ns, side="left")
    if idx == 0: return sorted_orientation_ts_ns[0]
    if idx == len(sorted_orientation_ts_ns): return sorted_orientation_ts_ns[-1]
    ts_before = sorted_orientation_ts_ns[idx-1]; ts_after = sorted_orientation_ts_ns[idx]
    if abs(target_ts_ns - ts_before) < abs(target_ts_ns - ts_after): return ts_before
    else: return ts_after

# 4. Combine Odometry (Position) and Orientation Data
print("\nCombining position and orientation data...")
all_rov_data = [] # List of [ts_sec, x, y, z, qx, qy, qz, qw]
skipped_frames = 0
for pos_data in all_rov_pos_data:
    ts_sec, x, y, z = pos_data
    ts_ns = int(ts_sec * 1e9)
    nearest_orient_ts = find_nearest_orientation_ts_ns(ts_ns)

    if nearest_orient_ts is not None:
        roll_deg, pitch_deg, yaw_deg = orientation_data[nearest_orient_ts] # Get degrees
        try:
            # Convert Euler degrees to quaternion (SciPy expects [x, y, z, w])
            # Using 'xyz' order - adjust if your data uses a different convention
            rotation = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True) # Use degrees=True
            qx, qy, qz, qw = rotation.as_quat()
            all_rov_data.append([ts_sec, x, y, z, qx, qy, qz, qw])
        except Exception as e:
            print(f"Warning: Skipping frame at ts {ts_sec:.3f} due to rotation conversion error: {e}", file=sys.stderr)
            skipped_frames += 1
    else:
        print(f"Warning: Skipping frame at ts {ts_sec:.3f} - could not find matching orientation data.", file=sys.stderr)
        skipped_frames += 1

if not all_rov_data: print("Error: No combined ROV data available after matching position and orientation.", file=sys.stderr); sys.exit(1)
print(f"Combined {len(all_rov_data)} frames.")
if skipped_frames > 0: print(f"Warning: Skipped {skipped_frames} frames during combination.", file=sys.stderr)

# --- Calculate Start Time ---
start_time_sec = all_rov_data[0][0] if all_rov_data else 0.0
print(f"Simulation start time (for elapsed time calculation): {start_time_sec:.3f} s")

# Subsample for animation
rov_data_anim = all_rov_data[::FRAME_STEP]
num_anim_frames = len(rov_data_anim)
print(f"Subsampled to {num_anim_frames} frames for animation.")


# 5. Load Tether Path Data (Renumbered from 5 to 6)
print(f"\nParsing Tether Path data from {tether_path_file_abs}...")
tether_data = {} # {timestamp_ns: np.array([[x,y,z],...])}
try:
    with open(tether_path_file_abs, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile); header = next(reader)
        expected_header = ['timestamp', 'point_index', 'x', 'y', 'z']
        if header != expected_header: print(f"Error: Tether CSV header '{header}' != '{expected_header}'", file=sys.stderr); sys.exit(1)
        current_ts_sec = None; current_points = []
        for row_idx, row in enumerate(reader):
            try:
                if len(row) == 5:
                    ts_sec, _, x, y, z = [float(val) for val in row]
                    ts_ns = int(ts_sec * 1e9)
                    if ts_sec != current_ts_sec:
                        if current_points: tether_data[int(current_ts_sec * 1e9)] = np.array(current_points)
                        current_ts_sec = ts_sec; current_points = []
                    current_points.append([x, y, z])
                else: print(f"Warning: Skipping malformed tether path row #{row_idx+1}", file=sys.stderr)
            except ValueError as e: print(f"Warning: Skipping tether path row #{row_idx+1} due to conversion error ({e})", file=sys.stderr)
        if current_points and current_ts_sec is not None: tether_data[int(current_ts_sec * 1e9)] = np.array(current_points)
except FileNotFoundError: print(f"Error: Tether path CSV file not found - {tether_path_file_abs}", file=sys.stderr); sys.exit(1)
except Exception as e: print(f"An unexpected error occurred reading {tether_path_file_abs}: {e}", file=sys.stderr); sys.exit(1)
if not tether_data: print("Warning: No valid tether path data parsed from CSV. Tether will not be animated.", file=sys.stderr) # Changed to warning
sorted_tether_ts_ns = np.array(sorted(tether_data.keys()))
print(f"Parsed {len(tether_data)} unique tether path timestamps.")

# --- Helper Function for Timestamp Matching (Tether, using nanoseconds) ---
def find_nearest_tether_ts_ns(target_ts_ns):
    if not sorted_tether_ts_ns.size: return None # Handle empty tether data
    idx = np.searchsorted(sorted_tether_ts_ns, target_ts_ns, side="left")
    if idx == 0: return sorted_tether_ts_ns[0]
    if idx == len(sorted_tether_ts_ns): return sorted_tether_ts_ns[-1]
    ts_before = sorted_tether_ts_ns[idx-1]; ts_after = sorted_tether_ts_ns[idx]
    if abs(target_ts_ns - ts_before) < abs(target_ts_ns - ts_after): return ts_before
    else: return ts_after

# --- Global State for Reconstruction ---
inspected_triangle_indices = set()

# --- Animation Setup ---
print("\nSetting up animation...")
# Create figure and main 3D axes + new Top View axes
fig = plt.figure(figsize=(16, 9)) # Adjusted figure size
# Adjust width ratio to make top view slightly larger relative to 3D view
gs = fig.add_gridspec(1, 2, width_ratios=[2, 1]) # Changed from [3, 1]
ax = fig.add_subplot(gs[0], projection='3d') # Main 3D plot
ax_top = fig.add_subplot(gs[1]) # New Top View (X-Y) plot

# --- Setup 3D Plot (ax) ---
ax.set_title('ROV Simulation (3D View)')
# Plot static elements first in the 3D plot
# 1. Environment STL Model - Initialize with uninspected color
env_mesh_collection = Poly3DCollection_3d(env_stl_vertices, facecolor=COLOR_UNINSPECTED, edgecolor=(0.2, 0.2, 0.2, 0.1)) # Use aliased name
ax.add_collection3d(env_mesh_collection)
print("Environment STL model added to 3D plot.")

# 2. Full ROV Trajectory (background) in 3D plot
# Ensure all_rov_data is not empty before trying to plot
if all_rov_data:
    full_rov_traj_points = np.array([p[1:4] for p in all_rov_data])
    ax.plot(full_rov_traj_points[:, 0], full_rov_traj_points[:, 1], full_rov_traj_points[:, 2], 'grey', linestyle=':', alpha=0.5, label='Full Trajectory', zorder=1) # Lower zorder
else:
    full_rov_traj_points = np.empty((0, 3)) # Empty array if no data

# Initialize dynamic plot elements for the 3D plot (ax)
rov_mesh_collection = Poly3DCollection_3d([], facecolor='blue', edgecolor='black', alpha=0.8, zorder=10) # Use aliased name
ax.add_collection3d(rov_mesh_collection)
tether_line, = ax.plot([], [], [], 'y-', linewidth=3, label='Tether (3D)', zorder=9) # Label updated
# Orientation axes (Optional: can be removed if ROV model is clear enough) in 3D plot
quivers = [
    ax.quiver([], [], [], [], [], [], color='r', length=AXIS_LENGTH, normalize=True, zorder=11, pivot='tail'), # Use pivot='tail', normalize=True
    ax.quiver([], [], [], [], [], [], color='g', length=AXIS_LENGTH, normalize=True, zorder=11, pivot='tail'),
    ax.quiver([], [], [], [], [], [], color='b', length=AXIS_LENGTH, normalize=True, zorder=11, pivot='tail')
]
# FOV Volume (Pyramid Faces) in 3D plot
fov_volume_collection = Poly3DCollection_3d([], alpha=ALPHA_FOV_VOLUME, facecolor=COLOR_FOV_VOLUME, edgecolor=None, zorder=8) # Use aliased name
ax.add_collection3d(fov_volume_collection)

# --- Create Inset Axes for Camera POV (within ax) ---
# Position: [left, bottom, width, height] relative to ax bounding box
# Move to top-right and make slightly smaller
inset_width = 0.20
inset_height = 0.20
inset_margin = 0.03
inset_pos = [1.0 - inset_width - inset_margin, 1.0 - inset_height - inset_margin, inset_width, inset_height] # Top-right corner
ax_cam_inset = ax.inset_axes(inset_pos)
ax_cam_inset.set_title('ROV POV', fontsize=7) # Smaller font
ax_cam_inset.set_xticks([]) # Hide ticks
ax_cam_inset.set_yticks([])
ax_cam_inset.set_aspect('equal', adjustable='box')
# Initialize PolyCollection for projected mesh faces in the inset axes
cam_mesh_collection = PolyCollection([], facecolor=COLOR_CURRENTLY_VISIBLE[:-1] + (0.6,), edgecolor=(0.1, 0.1, 0.1, 0.3), zorder=5)
ax_cam_inset.add_collection(cam_mesh_collection)
# Set inset camera view limits based on FOV (tangents at Z=1)
cam_x_lim = math.tan(math.radians(CAM_FOV_H_DEG / 2.0))
cam_y_lim = math.tan(math.radians(CAM_FOV_V_DEG / 2.0))
ax_cam_inset.set_xlim(-cam_x_lim, cam_x_lim)
ax_cam_inset.set_ylim(-cam_y_lim, cam_y_lim)
# ax_cam_inset.invert_yaxis() # Removed to flip POV 180 degrees vertically

# --- Setup Top View Plot (ax_top) ---
ax_top.set_title('Top View (X-Y)')
ax_top.set_xlabel('X World (m)')
ax_top.set_ylabel('Y World (m)')
ax_top.set_aspect('equal', adjustable='box')
ax_top.grid(True)
# Plot static environment projection (just vertices for simplicity)
ax_top.scatter(env_stl_vertices[:,:,0].flatten(), env_stl_vertices[:,:,1].flatten(), s=1, c='grey', alpha=0.1, zorder=1)
# Plot full trajectory projection
if full_rov_traj_points.size > 0:
    ax_top.plot(full_rov_traj_points[:, 0], full_rov_traj_points[:, 1], 'grey', linestyle=':', alpha=0.5, label='Full Trajectory', zorder=2)
# Initialize dynamic elements for top view
rov_pos_top, = ax_top.plot([], [], 'bo', markersize=6, alpha=0.8, label='ROV (Top)', zorder=10)
tether_line_top, = ax_top.plot([], [], 'y-', linewidth=2, label='Tether (Top)', zorder=9)
ax_top.legend(loc='upper right', fontsize=8)


# --- Set Plot Limits (ax and ax_top) ---
# Set plot limits based on Environment STL and full trajectory
print("Setting plot limits...")
plot_elements_x = [env_stl_vertices[:,:,0].flatten()]
plot_elements_y = [env_stl_vertices[:,:,1].flatten()]
plot_elements_z = [env_stl_vertices[:,:,2].flatten()]
if full_rov_traj_points.size > 0:
    plot_elements_x.append(full_rov_traj_points[:, 0])
    plot_elements_y.append(full_rov_traj_points[:, 1])
    plot_elements_z.append(full_rov_traj_points[:, 2])

all_x = np.concatenate(plot_elements_x)
all_y = np.concatenate(plot_elements_y)
all_z = np.concatenate(plot_elements_z)

if all_x.size > 0: # Ensure there's data to calculate limits
    x_min, x_max = all_x.min(), all_x.max(); y_min, y_max = all_y.min(), all_y.max(); z_min, z_max = all_z.min(), all_z.max()
    x_center = (x_min + x_max) / 2; y_center = (y_min + y_max) / 2; z_center = (z_min + z_max) / 2
    # Handle case where range is zero
    x_range = x_max - x_min if x_max > x_min else 1.0
    y_range = y_max - y_min if y_max > y_min else 1.0
    z_range = z_max - z_min if z_max > z_min else 1.0
    max_range = max(x_range, y_range, z_range) * 1.1 # Add buffer
    plot_radius = max_range / 2.0
    # Set limits for 3D plot
    ax.set_xlim(x_center - plot_radius, x_center + plot_radius)
    ax.set_ylim(y_center - plot_radius, y_center + plot_radius)
    ax.set_zlim(z_center - plot_radius, z_center + plot_radius)
    # Set limits for Top View plot (using X and Y ranges)
    top_view_radius = max(x_range, y_range) * 0.55 # Slightly more than half max range
    ax_top.set_xlim(x_center - top_view_radius, x_center + top_view_radius)
    ax_top.set_ylim(y_center - top_view_radius, y_center + top_view_radius)
    print("Plot limits set for both views.")
else:
    print("Warning: No data points found to determine plot limits. Using default limits.", file=sys.stderr)

# Set labels for 3D plot and apply standard font sizes
ax.set_xlabel('X World (m)', fontsize=12) # Reduced font size
ax.set_ylabel('Y World (m)', fontsize=12) # Reduced font size
ax.set_zlabel('Z World (m)', fontsize=12) # Reduced font size
ax.legend(loc='lower right', fontsize=10) # Reduced font size
ax.grid(True)
ax.tick_params(axis='both', which='major', labelsize=10) # Reduced font size


# Note: coverage_log and inspected_triangle_indices are reset before animation starts
coverage_log = []
# Text element configuration (only used in animation branch)
TEXT_FONTSIZE = 10 # Slightly smaller font
TIME_BOX_STYLE = dict(boxstyle='round,pad=0.3', fc='lightblue', alpha=0.7)
COVERAGE_BOX_STYLE = dict(boxstyle='round,pad=0.3', fc='lightgreen', alpha=0.7)
TETHER_BOX_STYLE_NORMAL = dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.7)
TETHER_BOX_STYLE_WARN = dict(boxstyle='round,pad=0.3', fc='lightcoral', alpha=0.7)

# Pre-calculate FOV half-angles in radians
fov_h_rad_half = math.radians(CAM_FOV_H_DEG / 2.0)
fov_v_rad_half = math.radians(CAM_FOV_V_DEG / 2.0)
fov_vis_range_sq = FOV_VIS_RANGE**2 # Pre-calculate squared range for efficiency

# Animation update function
def update(frame_index):
    # Add new top view elements and ensure cam_mesh_collection is global
    global quivers, env_mesh_collection, fov_volume_collection, inspected_triangle_indices, rov_mesh_collection, cam_mesh_collection, rov_pos_top, tether_line_top

    # Get ROV pose data (now includes quaternion)
    rov_ts_sec, rov_x, rov_y, rov_z, qx, qy, qz, qw = rov_data_anim[frame_index] # Unpack quaternion
    current_rov_pos = np.array([rov_x, rov_y, rov_z])
    rov_ts_ns = int(rov_ts_sec * 1e9)

    # --- Calculate Camera Pose ---
    valid_rotation = False
    fov_pyramid_faces = []
    cam_pos_world = np.zeros(3)
    cam_rotation_world = R.identity()
    cam_forward = np.array([1,0,0])
    cam_up = np.array([0,0,1])
    cam_right = np.array([0,-1,0])
    try:
        # Use quaternion directly (SciPy expects [x, y, z, w])
        rov_rotation = R.from_quat([qx, qy, qz, qw])
        # Apply camera offset rotation first, then ROV rotation
        # cam_rotation_world = rov_rotation * R.from_quat(CAM_OFFSET_QUAT) # Incorrect order?
        cam_rotation_world = R.from_quat(CAM_OFFSET_QUAT) * rov_rotation # Apply ROV rot, then camera offset rot relative to ROV
        # Calculate camera position by applying offset in ROV frame, then transforming to world
        cam_pos_world = current_rov_pos + rov_rotation.apply(CAM_OFFSET_POS)
        # Get camera axes in world frame
        cam_forward = cam_rotation_world.apply([1, 0, 0]) # Camera X-axis
        cam_up = cam_rotation_world.apply([0, 0, 1])      # Camera Z-axis
        cam_right = cam_rotation_world.apply([0, -1, 0]) # Camera Y-axis (negated for right-hand rule if needed, check projection)
        # cam_right = np.cross(cam_forward, cam_up) # Alternative calculation
        valid_rotation = True

        # Calculate FOV corner points for 3D visualization
        half_width = FOV_VIS_RANGE * math.tan(fov_h_rad_half)
        half_height = FOV_VIS_RANGE * math.tan(fov_v_rad_half)
        center_point = cam_pos_world + cam_forward * FOV_VIS_RANGE
        corner_ul = center_point + cam_up * half_height - cam_right * half_width
        corner_ur = center_point + cam_up * half_height + cam_right * half_width
        corner_ll = center_point - cam_up * half_height - cam_right * half_width
        corner_lr = center_point - cam_up * half_height + cam_right * half_width
        fov_pyramid_faces = [
            [cam_pos_world, corner_ul, corner_ur], [cam_pos_world, corner_ur, corner_lr],
            [cam_pos_world, corner_lr, corner_ll], [cam_pos_world, corner_ll, corner_ul]
        ]
    except ValueError as e:
        print(f"Warning: Error creating ROV rotation for frame {frame_index}, quat=[{qx},{qy},{qz},{qw}]. Error: {e}", file=sys.stderr) # Updated error message

    # --- Update Tether ---
    nearest_tether_ts_ns = find_nearest_tether_ts_ns(rov_ts_ns)
    if nearest_tether_ts_ns is not None and nearest_tether_ts_ns in tether_data:
        tether_points = tether_data[nearest_tether_ts_ns].copy()
        if tether_points.shape[0] > 0:
            # Ensure the last point of the tether matches the current ROV position
            tether_points[-1] = current_rov_pos # Ensure last point matches ROV
            # Update 3D tether
            tether_line.set_data_3d(tether_points[:, 0], tether_points[:, 1], tether_points[:, 2])
            # Update Top View tether
            tether_line_top.set_data(tether_points[:, 0], tether_points[:, 1])
        else:
            tether_line.set_data_3d([], [], []) # Empty path if no points
            tether_line_top.set_data([], [])
    else:
        tether_line.set_data_3d([], [], []) # Empty path if no matching timestamp or no tether data
        tether_line_top.set_data([], [])

    # --- Update ROV Mesh (3D) ---
    if valid_rotation:
        # Reshape for rotation: (N_triangles * 3_vertices, 3_coords)
        flat_vertices = rov_base_vertices.reshape(-1, 3)
        # Apply rotation and then translation
        transformed_flat = rov_rotation.apply(flat_vertices) + current_rov_pos
        # Reshape back: (N_triangles, 3_vertices, 3_coords)
        transformed_vertices = transformed_flat.reshape(rov_base_vertices.shape)
        rov_mesh_collection.set_verts(transformed_vertices)
    else:
        rov_mesh_collection.set_verts([]) # Hide if rotation is invalid

    # --- Update ROV Position Marker (Top View) ---
    rov_pos_top.set_data([rov_x], [rov_y])

    # --- Update Orientation Axes (3D Plot) --- (Optional, can be commented out)
    for q in quivers: q.remove()
    quivers = []
    if valid_rotation:
        try:
            axes_vectors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) # Base axes
            rotated_axes = rov_rotation.apply(axes_vectors) # Apply ROV rotation
            colors = ['r', 'g', 'b']
            for i in range(3):
                # Draw axes originating from current_rov_pos
                quivers.append(ax.quiver(current_rov_pos[0], current_rov_pos[1], current_rov_pos[2],
                                         rotated_axes[i, 0], rotated_axes[i, 1], rotated_axes[i, 2],
                                         color=colors[i], length=AXIS_LENGTH, normalize=True, zorder=11, pivot='tail'))
        except Exception as e:
             print(f"Warning: Error drawing orientation axes for frame {frame_index}. Error: {e}", file=sys.stderr)

    # --- Update Environment Mesh Colors for Reconstruction (in 3D Plot) ---
    currently_visible_indices = set()
    visible_vertices_world = [] # Store world coordinates of vertices of visible triangles
    if valid_rotation:
        indices_list_current = [] # Keep track of indices for color update
        for i, tri_vertices in enumerate(env_stl_vertices): # Use env_stl_vertices
            centroid = np.mean(tri_vertices, axis=0)
            view_vector = centroid - cam_pos_world
            dist_sq = np.dot(view_vector, view_vector)

            # 0. Distance Check (New)
            if dist_sq > fov_vis_range_sq:
                continue # Skip if triangle centroid is beyond the visual range

            # 1. Back-face Culling
            tri_normal = env_stl_normals[i] # Use env_stl_normals
            if np.linalg.norm(tri_normal) < 1e-9: continue
            dot_product = np.dot(tri_normal, normalize_vector(-view_vector))
            if dot_product < 0.1: continue # Facing away or edge-on

            # 2. FOV Check (Angle-based) - Check centroid
            view_vector_normalized = normalize_vector(view_vector)
            angle_to_forward = angle_between_vectors(cam_forward, view_vector_normalized)
            # Use a slightly wider angle check than the strict FOV to be generous
            max_fov_half_rad_check = max(fov_h_rad_half, fov_v_rad_half) * 1.1
            if angle_to_forward <= max_fov_half_rad_check:
                 currently_visible_indices.add(i) # Mark as currently visible
                 indices_list_current.append(i) # Add to list for color update
                 visible_vertices_world.append(tri_vertices) # Store vertices for camera view

    # Add currently visible to the set of all inspected triangles
    inspected_triangle_indices.update(currently_visible_indices)

    # Set face colors based on inspection status for the 3D plot
    new_face_colors = np.tile(np.array(COLOR_UNINSPECTED), (len(env_stl_vertices), 1)) # Use env_stl_vertices length
    if inspected_triangle_indices:
        indices_list = list(inspected_triangle_indices)
        new_face_colors[indices_list] = COLOR_INSPECTED
    # Use the list created during the loop for currently visible
    if indices_list_current:
        new_face_colors[indices_list_current] = COLOR_CURRENTLY_VISIBLE

    env_mesh_collection.set_facecolor(new_face_colors) # Update env_mesh_collection in 3D plot

    # --- Update FOV Volume in 3D Plot ---
    if valid_rotation and fov_pyramid_faces:
        fov_volume_collection.set_verts(fov_pyramid_faces)
    else:
        fov_volume_collection.set_verts([]) # Hide if rotation is invalid

    # --- Calculate Tether Length ---
    current_tether_length = 0.0
    if nearest_tether_ts_ns is not None and nearest_tether_ts_ns in tether_data:
        tether_points_for_len = tether_data[nearest_tether_ts_ns]
        if tether_points_for_len.shape[0] > 1:
            # Calculate segment lengths and sum them
            segment_vectors = np.diff(tether_points_for_len, axis=0)
            segment_lengths = np.linalg.norm(segment_vectors, axis=1)
            current_tether_length = np.sum(segment_lengths)

    # --- Calculate Coverage ---
    total_triangles = len(env_stl_vertices) # Use env_stl_vertices length
    coverage_percentage = (len(inspected_triangle_indices) / total_triangles) * 100 if total_triangles > 0 else 0

    # --- Update Camera View (Inset 2D Plot) ---
    projected_triangles_2d = [] # List to hold valid 2D triangles (3x2 arrays)
    if valid_rotation and visible_vertices_world:
        # Use pre-calculated inverse rotation and limits
        cam_rotation_inv = cam_rotation_world.inv()
        x_limit = math.tan(fov_h_rad_half)
        y_limit = math.tan(fov_v_rad_half)

        for tri_world in visible_vertices_world: # Iterate through list of 3x3 world coord triangles
            # Transform triangle vertices to camera frame
            tri_cam = cam_rotation_inv.apply(tri_world - cam_pos_world)

            # Simple Clipping: Check if all vertices are in front of the camera
            if np.all(tri_cam[:, 0] > 1e-6): # Use X-axis as forward
                # Perspective Projection for all 3 vertices
                projected_x = -tri_cam[:, 1] / tri_cam[:, 0] # Image X = -Camera Y
                projected_y = tri_cam[:, 2] / tri_cam[:, 0]  # Image Y = Camera Z
                tri_proj_2d = np.column_stack((projected_x, projected_y)) # Shape (3, 2)

                # Simple Clipping: Check if all projected vertices are within FOV limits
                if np.all((np.abs(tri_proj_2d[:, 0]) <= x_limit) & (np.abs(tri_proj_2d[:, 1]) <= y_limit)):
                    projected_triangles_2d.append(tri_proj_2d) # Add the 3x2 array

    # Update the PolyCollection in the camera view
    if projected_triangles_2d:
        cam_mesh_collection.set_verts(projected_triangles_2d)
    else:
        cam_mesh_collection.set_verts([]) # Clear if no triangles are visible/valid

    # --- Update Info Text Boxes in 3D Plot ---
    # Time (Elapsed)
    elapsed_time_sec = rov_ts_sec - start_time_sec
    time_text.set_text(f"Elapsed Time: {elapsed_time_sec:.2f} s")

    # Coverage (Thermometer Style Bar + Percentage)
    bar_width = 20 # Width of the text progress bar
    progress = coverage_percentage / 100.0
    filled_width = progress * bar_width
    full_blocks = int(filled_width)
    # Calculate the fractional part (0-7 for 8 levels: empty, ▏...▉)
    fractional_part = int((filled_width - full_blocks) * 8)
    # Unicode blocks: U+2588 to U+258F (Full block to 1/8 block)
    # We use U+2588 (Full) and U+258F down to U+2589 (1/8 to 7/8)
    # Let's use 8 levels: ' ', '▏', '▎', '▍', '▌', '▋', '▊', '▉' (index 0 to 7)
    fractional_chars = [' ', '▏', '▎', '▍', '▌', '▋', '▊', '▉']
    fractional_block_char = fractional_chars[fractional_part] if full_blocks < bar_width else ''

    empty_blocks = bar_width - full_blocks - (1 if fractional_block_char != ' ' else 0)

    bar_str = '█' * full_blocks + fractional_block_char + ' ' * empty_blocks
    coverage_text.set_text(f"Coverage: [{bar_str}] {coverage_percentage:.1f}%")
    coverage_text.set_color('darkgreen') # Set text color to green


    # Tether Length (Value + Color/Box Change)
    tether_length_str = f"Tether Length: {current_tether_length:.2f} m"
    tether_text.set_text(tether_length_str)
    L_max = 10.0 # Define L_max
    if current_tether_length > L_max:
        tether_text.set_color('darkred') # Darker red for better contrast
        tether_text.get_bbox_patch().set_boxstyle(TETHER_BOX_STYLE_WARN['boxstyle'])
        tether_text.get_bbox_patch().set_facecolor(TETHER_BOX_STYLE_WARN['fc'])
        tether_text.get_bbox_patch().set_alpha(TETHER_BOX_STYLE_WARN['alpha'])
    else:
        tether_text.set_color('black')
        tether_text.get_bbox_patch().set_boxstyle(TETHER_BOX_STYLE_NORMAL['boxstyle'])
        tether_text.get_bbox_patch().set_facecolor(TETHER_BOX_STYLE_NORMAL['fc'])
        tether_text.get_bbox_patch().set_alpha(TETHER_BOX_STYLE_NORMAL['alpha'])


    # Update 3D plot title (optional, as info is now in text box)
    ax.set_title(f'ROV Simulation (3D View - Frame {frame_index+1}/{num_anim_frames})') # Frame count 1-based

    # Log coverage data
    coverage_log.append({'time_elapsed': elapsed_time_sec, 'coverage_percent': coverage_percentage})

    # Return updated artists from all plots
    artists_to_return = [
        rov_mesh_collection, tether_line, env_mesh_collection, fov_volume_collection, # 3D elements
        time_text, coverage_text, tether_text, # Text elements
        cam_mesh_collection, # Inset camera elements
        rov_pos_top, tether_line_top # Top view elements
    ] + quivers # Add quivers list
    return artists_to_return


# --- Function to Render a Single Static Frame ---
def render_static_frame(fig_static, ax_static, data_point):
    global quivers, env_mesh_collection, rov_mesh_collection # Need to modify these globals

    # Get ROV pose data
    rov_ts_sec, rov_x, rov_y, rov_z, qx, qy, qz, qw = data_point
    current_rov_pos = np.array([rov_x, rov_y, rov_z])
    rov_ts_ns = int(rov_ts_sec * 1e9)

    # --- Calculate ROV Rotation ---
    valid_rotation = False
    try:
        rov_rotation = R.from_quat([qx, qy, qz, qw])
        valid_rotation = True
    except ValueError as e:
        print(f"Warning: Error creating ROV rotation for static frame, quat=[{qx},{qy},{qz},{qw}]. Error: {e}", file=sys.stderr)

    # --- Update Tether ---
    nearest_tether_ts_ns = find_nearest_tether_ts_ns(rov_ts_ns)
    tether_points = []
    if nearest_tether_ts_ns is not None and nearest_tether_ts_ns in tether_data:
        tether_points = tether_data[nearest_tether_ts_ns].copy()
        if tether_points.shape[0] > 0:
            tether_points[-1] = current_rov_pos # Ensure last point matches ROV

    # --- Update ROV Mesh ---
    transformed_vertices = []
    if valid_rotation:
        flat_vertices = rov_base_vertices.reshape(-1, 3)
        transformed_flat = rov_rotation.apply(flat_vertices) + current_rov_pos
        transformed_vertices = transformed_flat.reshape(rov_base_vertices.shape)

    # --- Draw Static Elements ---
    # Environment (already plotted, but set final color state)
    # Calculate final inspected state (run through all data points quickly)
    print("Calculating final inspected state for static plot...")
    final_inspected_indices = set()
    for frame_data in all_rov_data: # Use full data
        ts, x, y, z, qx_f, qy_f, qz_f, qw_f = frame_data
        pos_f = np.array([x, y, z])
        try:
            rot_f = R.from_quat([qx_f, qy_f, qz_f, qw_f])
            cam_rot_f = R.from_quat(CAM_OFFSET_QUAT) * rot_f
            cam_pos_f = pos_f + rot_f.apply(CAM_OFFSET_POS)
            cam_fwd_f = cam_rot_f.apply([1, 0, 0])

            for i, tri_vertices in enumerate(env_stl_vertices):
                centroid = np.mean(tri_vertices, axis=0)
                view_vector = centroid - cam_pos_f
                dist_sq = np.dot(view_vector, view_vector)
                if dist_sq > fov_vis_range_sq: continue
                tri_normal = env_stl_normals[i]
                if np.linalg.norm(tri_normal) < 1e-9: continue
                dot_product = np.dot(tri_normal, normalize_vector(-view_vector))
                if dot_product < 0.1: continue
                view_vector_normalized = normalize_vector(view_vector)
                angle_to_forward = angle_between_vectors(cam_fwd_f, view_vector_normalized)
                max_fov_half_rad_check = max(fov_h_rad_half, fov_v_rad_half) * 1.1
                if angle_to_forward <= max_fov_half_rad_check:
                    final_inspected_indices.add(i)
        except ValueError:
            continue # Skip frame if rotation invalid

    final_face_colors = np.tile(np.array(COLOR_UNINSPECTED), (len(env_stl_vertices), 1))
    if final_inspected_indices:
        indices_list = list(final_inspected_indices)
        final_face_colors[indices_list] = COLOR_INSPECTED
    env_mesh_collection.set_facecolor(final_face_colors)
    print(f"Final coverage for static plot: {(len(final_inspected_indices) / len(env_stl_vertices)) * 100:.2f}%")


    # ROV Mesh
    if transformed_vertices.size > 0:
        rov_mesh_collection_static = Poly3DCollection_3d(transformed_vertices, facecolor='blue', edgecolor='black', alpha=0.8, zorder=10)
        ax_static.add_collection3d(rov_mesh_collection_static)

    # Tether Line
    if len(tether_points) > 1:
        ax_static.plot(tether_points[:, 0], tether_points[:, 1], tether_points[:, 2], 'y-', linewidth=5.0, zorder=9) # Thick line

    # Orientation Axes (Optional)
    if valid_rotation:
        try:
            axes_vectors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            rotated_axes = rov_rotation.apply(axes_vectors)
            colors = ['r', 'g', 'b']
            for i in range(3):
                ax_static.quiver(current_rov_pos[0], current_rov_pos[1], current_rov_pos[2],
                                 rotated_axes[i, 0], rotated_axes[i, 1], rotated_axes[i, 2],
                                 color=colors[i], length=AXIS_LENGTH, normalize=True, zorder=11, pivot='tail')
        except Exception as e:
             print(f"Warning: Error drawing orientation axes for static frame. Error: {e}", file=sys.stderr)


# --- Main Execution Logic ---

if args.final_frame_only:
    # --- Generate Static Final Frame ---
    print("\nGenerating static final frame...")

    # Create figure and 3D axes ONLY
    fig_static = plt.figure(figsize=(12, 9)) # Adjust size as needed
    ax_static = fig_static.add_subplot(111, projection='3d')

    # Setup static elements (Environment, Full Trajectory)
    env_mesh_collection = Poly3DCollection_3d(env_stl_vertices, facecolor=COLOR_UNINSPECTED, edgecolor=(0.2, 0.2, 0.2, 0.1))
    ax_static.add_collection3d(env_mesh_collection)
    if all_rov_data:
        full_rov_traj_points = np.array([p[1:4] for p in all_rov_data])
        ax_static.plot(full_rov_traj_points[:, 0], full_rov_traj_points[:, 1], full_rov_traj_points[:, 2], 'grey', linestyle=':', alpha=0.5, zorder=1, linewidth=1.3) # Increased trajectory linewidth

    # Set plot limits (zoomed in) and labels for static plot
    if all_x.size > 0:
        # Use a smaller multiplier for plot_radius to zoom in
        static_plot_radius = max(x_range, y_range, z_range) * 0.9 / 2.0 # Reduced buffer multiplier from 1.1 to 0.9
        ax_static.set_xlim(x_center - static_plot_radius, x_center + static_plot_radius)
        ax_static.set_ylim(y_center - static_plot_radius, y_center + static_plot_radius)
        ax_static.set_zlim(z_center - static_plot_radius, z_center + static_plot_radius)
    ax_static.set_xlabel('X World (m)', fontsize=12) # Reduced font size
    ax_static.set_ylabel('Y World (m)', fontsize=12) # Reduced font size
    ax_static.set_zlabel('Z World (m)', fontsize=12) # Reduced font size
    ax_static.tick_params(axis='both', which='major', labelsize=10) # Reduced font size
    # Limit the number of ticks
    ax_static.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax_static.yaxis.set_major_locator(MaxNLocator(nbins=5))
    ax_static.zaxis.set_major_locator(MaxNLocator(nbins=5))
    ax_static.grid(True)

    # Render the final frame state
    if all_rov_data:
        final_data_point = all_rov_data[-1]
        render_static_frame(fig_static, ax_static, final_data_point)
    else:
        print("Error: No ROV data loaded, cannot generate final frame.", file=sys.stderr)

    # Save the figure
    print(f"Saving final frame to {args.save_final_frame}...")
    try:
        fig_static.savefig(args.save_final_frame, format='pdf', bbox_inches='tight', pad_inches=0)
        print("Final frame saved successfully.")
    except Exception as e:
        print(f"Error saving static frame: {e}", file=sys.stderr)
    # Exit after saving the static frame if the flag is set
    sys.exit(0) # Moved inside the if block

else:
    # --- Standard Animation Logic ---
    # Create text elements for live data display - ONLY FOR ANIMATION
    time_text = ax.text2D(0.02, 0.98, '', transform=ax.transAxes, verticalalignment='top', fontsize=TEXT_FONTSIZE, bbox=TIME_BOX_STYLE)
    coverage_text = ax.text2D(0.02, 0.91, '', transform=ax.transAxes, verticalalignment='top', fontsize=TEXT_FONTSIZE, bbox=COVERAGE_BOX_STYLE)
    tether_text = ax.text2D(0.02, 0.84, '', transform=ax.transAxes, verticalalignment='top', fontsize=TEXT_FONTSIZE, bbox=TETHER_BOX_STYLE_NORMAL)

    # --- Explicitly Reset State Before Animation ---
    print("\nResetting coverage state before animation...")
    inspected_triangle_indices.clear() # Ensure it's empty before update calls start
    coverage_log = [] # Ensure log is empty before update calls start

    # Create animation
    # Note: Blitting might be problematic with multiple subplots and complex updates. Set blit=False.
    ani = animation.FuncAnimation(fig, update, frames=num_anim_frames,
                                  interval=ANIMATION_INTERVAL_MS, blit=False, repeat=False)

    # --- Force Animation Generation & Save Coverage Data --- (COMMENTED OUT TO PRIORITIZE INTERACTIVE DISPLAY)
    # # Trigger frame generation by saving animation temporarily (forces update loop)
    # # This populates the coverage_log
    # print("\nGenerating animation frames to capture coverage data...")
    # try:
    #     # Save to a dummy file to force frame generation
    #     ani.save('temp_anim_trigger.mp4', writer='ffmpeg', fps=30, dpi=75) # Use low dpi for speed
    #     os.remove('temp_anim_trigger.mp4') # Remove the dummy file
    #     print("Frame generation complete.")
    #
    #     # Now save the logged coverage data
    #     if coverage_log:
    #         print("Saving coverage data...")
    #         coverage_df = pd.DataFrame(coverage_log)
    #         # Remove duplicates based on time_elapsed, keep last entry
    #         coverage_df = coverage_df.drop_duplicates(subset=['time_elapsed'], keep='last')
    #         coverage_output_path = os.path.join(data_dir_abs, 'coverage_data.csv')
    #         try:
    #             coverage_df.to_csv(coverage_output_path, index=False, float_format='%.3f')
    #             print(f"Coverage data saved to: {coverage_output_path}")
    #         except Exception as e:
    #             print(f"Error saving coverage data to {coverage_output_path}: {e}", file=sys.stderr)
    #     else:
    #         print("No coverage data logged to save.")
    #
    # except Exception as e:
    #     print(f"\nWarning: Could not force frame generation by saving animation (maybe ffmpeg is missing?). Coverage data might not be saved.", file=sys.stderr)
    #     print(f"Error details: {e}", file=sys.stderr)
    #     # print("Attempting to show plot anyway...") # Changed message slightly
    #     print("Attempting to show plot anyway...") # Corrected indentation


    # --- Save Animation ---
    output_video_path = os.path.join(data_dir_abs, OUTPUT_VIDEO_FILENAME)
    print(f"\nAnimation created ({num_anim_frames} frames). Saving video to {output_video_path}...")
    print("This may take a while...")
    try:
        # Ensure figure layout is adjusted before saving
        plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.05, wspace=0.15)
        ani.save(output_video_path, writer='ffmpeg', fps=VIDEO_FPS, dpi=VIDEO_DPI)
        print(f"Video saved successfully to {output_video_path}")
    except Exception as e:
        print("\nError saving animation:", file=sys.stderr)
        print("Please ensure 'ffmpeg' is installed and accessible in your system's PATH.", file=sys.stderr)
        print(f"Detailed error: {e}", file=sys.stderr)

    # Close the plot figure to prevent it from trying to display after saving
    plt.close(fig)

print("\nScript finished.")

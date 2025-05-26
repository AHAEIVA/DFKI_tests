#!/usr/bin/env python3

import yaml
import numpy as np
import sys
import os

# --- Configuration ---
RAW_MARKER_FILE_REL = 'results/stl_marker_raw.yaml'
OUTPUT_NUMPY_FILE_REL = 'results/stl_vertices.npy'
MARKER_TYPE_TRIANGLE_LIST = 7 # visualization_msgs/Marker type constant

# Get absolute paths
script_dir = os.path.dirname(os.path.abspath(__file__))
raw_marker_file_abs = os.path.join(script_dir, RAW_MARKER_FILE_REL)
output_numpy_file_abs = os.path.join(script_dir, OUTPUT_NUMPY_FILE_REL)

print(f"Processing raw marker file: {raw_marker_file_abs}")
print(f"Output numpy file: {output_numpy_file_abs}")

# --- Helper Function ---
def parse_yaml_stream(filepath):
    """Parses a stream of YAML documents from a file."""
    data = []
    try:
        with open(filepath, 'r') as f:
            try:
                for doc in yaml.safe_load_all(f):
                    if doc:
                        data.append(doc)
            except yaml.YAMLError as e:
                print(f"Error parsing YAML file {filepath}: {e}", file=sys.stderr)
                # Attempt recovery (optional, might be complex for markers)
                return None # Indicate failure for simplicity here
    except FileNotFoundError:
        print(f"Error: File not found - {filepath}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"An unexpected error occurred reading {filepath}: {e}", file=sys.stderr)
        return None
    return data

# --- Main Logic ---
print("Parsing raw marker data...")
marker_data_raw = parse_yaml_stream(raw_marker_file_abs)

if marker_data_raw is None or not marker_data_raw:
    print("Error: Could not parse or find data in raw marker file.", file=sys.stderr)
    sys.exit(1)

print(f"Parsed {len(marker_data_raw)} marker messages.")

stl_vertices = None

# Find the first TRIANGLE_LIST marker in the MarkerArray
for msg in marker_data_raw:
    if 'markers' in msg and isinstance(msg['markers'], list):
        for marker in msg['markers']:
            if 'type' in marker and marker['type'] == MARKER_TYPE_TRIANGLE_LIST and 'points' in marker:
                print("Found TRIANGLE_LIST marker.")
                points_data = marker['points']
                vertices = []
                valid_points = True
                for p_data in points_data:
                    if isinstance(p_data, dict) and all(k in p_data for k in ('x', 'y', 'z')):
                        vertices.append([p_data['x'], p_data['y'], p_data['z']])
                    else:
                        print(f"Warning: Invalid point structure found in marker: {p_data}", file=sys.stderr)
                        valid_points = False
                        break # Stop processing this marker if points are bad
                if valid_points and vertices:
                    stl_vertices = np.array(vertices)
                    print(f"Extracted {len(stl_vertices)} vertices.")
                    break # Stop after finding the first valid TRIANGLE_LIST
        if stl_vertices is not None:
            break # Exit outer loop too
    # Handle case where it's a single Marker message, not MarkerArray
    elif 'type' in msg and msg['type'] == MARKER_TYPE_TRIANGLE_LIST and 'points' in msg:
         print("Found single TRIANGLE_LIST marker.")
         points_data = msg['points']
         vertices = []
         valid_points = True
         for p_data in points_data:
             if isinstance(p_data, dict) and all(k in p_data for k in ('x', 'y', 'z')):
                 vertices.append([p_data['x'], p_data['y'], p_data['z']])
             else:
                 print(f"Warning: Invalid point structure found in marker: {p_data}", file=sys.stderr)
                 valid_points = False
                 break
         if valid_points and vertices:
             stl_vertices = np.array(vertices)
             print(f"Extracted {len(stl_vertices)} vertices.")
             break # Stop after finding the first valid TRIANGLE_LIST


if stl_vertices is None:
    print("Error: No valid TRIANGLE_LIST marker found in the data.", file=sys.stderr)
    sys.exit(1)

# Save the vertices to a numpy file
try:
    np.save(output_numpy_file_abs, stl_vertices)
    print(f"Successfully saved STL vertices to {output_numpy_file_abs}")
except Exception as e:
    print(f"Error saving numpy file: {e}", file=sys.stderr)
    sys.exit(1)

print("\nScript finished.")

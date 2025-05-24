#!/usr/bin/env python3

import yaml
import yaml
import csv
import sys
import os
import math # Re-added for degrees conversion
import argparse

# --- Helper Functions ---

def parse_yaml_stream(filepath):
    """Parses a stream of YAML documents separated by '---'."""
    documents = []
    try:
        with open(filepath, 'r') as infile:
            content = infile.read()
            # Split by '---' and filter out empty strings
            yaml_docs = [doc for doc in content.split('---') if doc.strip()]
            for doc_str in yaml_docs:
                try:
                    data = yaml.safe_load(doc_str)
                    if data: # Ensure it's not None or empty
                        documents.append(data)
                except yaml.YAMLError as e:
                    print(f"Warning: Skipping invalid YAML document in {filepath}: {e}\nContent: {doc_str[:100]}...", file=sys.stderr)
    except FileNotFoundError:
        print(f"Error: Input file not found - {filepath}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"Error reading file {filepath}: {e}", file=sys.stderr)
        return None
    return documents

def get_timestamp_sec(header):
    """Extracts timestamp in seconds from ROS message header."""
    try:
        return header['stamp']['secs'] + header['stamp']['nsecs'] * 1e-9
    except (KeyError, TypeError):
        return None

# --- Argument Parsing ---
parser = argparse.ArgumentParser(description='Process ROS topic data from YAML stream text files into CSV files. Extracts position and quaternion orientation from Odometry, Euler angles (degrees) from Vector3Stamped, and points from Marker.')
parser.add_argument('--odom-txt', required=True, help='Path to input odometry text file (YAML stream, nav_msgs/Odometry)')
parser.add_argument('--orient-txt', required=True, help='Path to input orientation text file (YAML stream, geometry_msgs/Vector3Stamped, assumed roll/pitch/yaw in DEGREES)') # Updated assumption
parser.add_argument('--tether-txt', required=True, help='Path to input tether path text file (YAML stream, visualization_msgs/Marker LINE_LIST)')
parser.add_argument('--odom-csv', required=True, help='Path for output odometry CSV file (will include orientation quaternion)')
parser.add_argument('--orient-csv', required=True, help='Path for output orientation CSV file (roll, pitch, yaw in degrees)')
parser.add_argument('--tether-csv', required=True, help='Path for output tether path CSV file')

args = parser.parse_args()

# --- Main Processing Logic ---

# Use paths from arguments
odom_txt_path = args.odom_txt
orient_txt_path = args.orient_txt # Re-added
tether_txt_path = args.tether_txt

output_odom_csv_path = args.odom_csv
output_orient_csv_path = args.orient_csv # Re-added
output_tether_csv_path = args.tether_csv

# Ensure output directories exist
for output_path in [output_odom_csv_path, output_orient_csv_path, output_tether_csv_path]: # Re-added orient path
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
            print(f"Created output directory: {output_dir}")
        except OSError as e:
            print(f"Error creating directory {output_dir}: {e}", file=sys.stderr)
            sys.exit(1)


# 1. Process Odometry Data (nav_msgs/Odometry)
print(f"Processing Odometry data from {odom_txt_path}...")
odom_docs = parse_yaml_stream(odom_txt_path)
if odom_docs is not None:
    odom_csv_data = []
    for doc in odom_docs:
        try:
            # Adjust path based on actual Odometry message structure
            header = doc.get('header', {})
            ts = get_timestamp_sec(header)
            pose_data = doc.get('pose', {}).get('pose', {}) # Standard Odometry has pose.pose
            pos = pose_data.get('position', {})
            orient = pose_data.get('orientation', {})

            x = pos.get('x')
            y = pos.get('y')
            z = pos.get('z')
            qx = orient.get('x')
            qy = orient.get('y')
            qz = orient.get('z')
            qw = orient.get('w')

            # Check if basic data (ts, position) is present
            if ts is not None and all(v is not None for v in [x, y, z]):
                # Check if quaternion is valid (not None and not all zeros)
                is_valid_quat = all(v is not None for v in [qx, qy, qz, qw]) and \
                                not (qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0)

                if is_valid_quat:
                    odom_csv_data.append([
                        f"{ts:.9f}",
                        f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                        f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}", f"{qw:.6f}"
                    ])
                else:
                    # Quaternion is invalid or missing, save position data only
                    print(f"Info: Saving odom message with invalid/missing quaternion (ts={ts}, pos=[{x},{y},{z}], quat=[{qx},{qy},{qz},{qw}]). Quaternion fields will be empty.", file=sys.stderr)
                    odom_csv_data.append([
                        f"{ts:.9f}",
                        f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                        "", "", "", "" # Empty strings for invalid quaternion
                    ])
            else:
                 # Timestamp or position is missing, skip entirely
                 print(f"Warning: Skipping odom message with missing timestamp or position (ts={ts}, pos=[{x},{y},{z}]): {str(doc)[:200]}...", file=sys.stderr)

        except Exception as e:
            print(f"Warning: Error processing odom message: {e}\nMessage: {str(doc)[:200]}...", file=sys.stderr)

    print(f"Processed {len(odom_docs) if odom_docs else 0} odometry messages, extracted {len(odom_csv_data)} data points.")
    try:
        with open(output_odom_csv_path, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']) # Added quaternion columns
            writer.writerows(odom_csv_data)
        print(f"Successfully wrote {len(odom_csv_data)} rows (including orientation) to {output_odom_csv_path}")
    except IOError as e:
        print(f"Error writing CSV file {output_odom_csv_path}: {e}", file=sys.stderr)
else:
    print(f"Could not process odometry file: {odom_txt_path}")


# 2. Process Orientation Data (geometry_msgs/Vector3Stamped)
print(f"\nProcessing Orientation data (Vector3Stamped) from {orient_txt_path}...")
orient_docs = parse_yaml_stream(orient_txt_path)
if orient_docs is not None:
    orient_csv_data = []
    for doc in orient_docs:
        try:
            header = doc.get('header', {})
            ts = get_timestamp_sec(header)
            vector = doc.get('vector', {})
            roll_deg = vector.get('x') # Assuming x=roll, y=pitch, z=yaw are already in degrees
            pitch_deg = vector.get('y')
            yaw_deg = vector.get('z')

            if ts is not None and all(v is not None for v in [roll_deg, pitch_deg, yaw_deg]):
                # Values are already degrees, no conversion needed
                orient_csv_data.append([
                    f"{ts:.9f}",
                    f"{roll_deg:.4f}", f"{pitch_deg:.4f}", f"{yaw_deg:.4f}" # Write degrees directly
                ])
            else:
                print(f"Warning: Skipping orientation message with missing data (ts={ts}, vec=[{roll_deg},{pitch_deg},{yaw_deg}]): {str(doc)[:200]}...", file=sys.stderr)
        except Exception as e:
            print(f"Warning: Error processing orientation message: {e}\nMessage: {str(doc)[:200]}...", file=sys.stderr)

    print(f"Extracted {len(orient_csv_data)} valid orientation points.")
    try:
        with open(output_orient_csv_path, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(['timestamp', 'roll_deg', 'pitch_deg', 'yaw_deg'])
            writer.writerows(orient_csv_data)
        print(f"Successfully wrote {len(orient_csv_data)} rows to {output_orient_csv_path}")
    except IOError as e:
        print(f"Error writing CSV file {output_orient_csv_path}: {e}", file=sys.stderr)
else:
    print(f"Could not process orientation file: {orient_txt_path}")


# 3. Process Tether Path Data (visualization_msgs/Marker, type LINE_LIST)
print(f"\nProcessing Tether Path data (Marker msgs) from {tether_txt_path}...")
tether_docs = parse_yaml_stream(tether_txt_path)
if tether_docs is not None:
    tether_csv_data = []
    total_points = 0
    processed_messages = 0
    for doc in tether_docs:
        try:
            # Check if it's a Marker message (basic check)
            if 'header' in doc and 'points' in doc and doc.get('type') == 8: # Type 8 is LINE_LIST
                processed_messages += 1
                header = doc.get('header', {})
                ts = get_timestamp_sec(header)
                points = doc.get('points', []) # List of geometry_msgs/Point

                if ts is not None:
                    for i, point in enumerate(points):
                        x = point.get('x')
                        y = point.get('y')
                        z = point.get('z')
                        if x is not None and y is not None and z is not None:
                            tether_csv_data.append([f"{ts:.9f}", i, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])
                            total_points += 1
                        else:
                            print(f"Warning: Skipping tether point with missing position data in message at ts {ts}", file=sys.stderr)
                else:
                    print(f"Warning: Skipping tether marker message with missing timestamp: {str(doc)[:200]}...", file=sys.stderr)
            # else: # Optionally log messages that are not LINE_LIST markers
                # print(f"Info: Skipping non-LINE_LIST marker message: {doc.get('id', 'N/A')}", file=sys.stderr)

        except Exception as e:
             print(f"Warning: Error processing tether marker message: {e}\nMessage: {str(doc)[:200]}...", file=sys.stderr)

    print(f"Processed {processed_messages} tether marker messages containing {total_points} total points.")
    try:
        with open(output_tether_csv_path, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(['timestamp', 'point_index', 'x', 'y', 'z'])
            writer.writerows(tether_csv_data)
        print(f"Successfully wrote {len(tether_csv_data)} rows to {output_tether_csv_path}")
    except IOError as e:
        print(f"Error writing CSV file {output_tether_csv_path}: {e}", file=sys.stderr)
else:
    print(f"Could not process tether file: {tether_txt_path}")

print("\nProcessing script finished.")

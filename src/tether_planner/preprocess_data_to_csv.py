#!/usr/bin/env python3

import yaml
import csv
import sys
import os
import rosbag # Import rosbag library
import numpy as np

# --- Configuration ---
MAX_MESSAGES_TO_PROCESS = 50000 # Limit for reading from bag (can be adjusted)
BAG_FILE_REL = 'results/rosbags/2025-04-04-12-49-02.bag'
ODOM_TOPIC = '/mobula/rov/odometry'
ORIENT_TOPIC = '/mobula/rov/orientation' # Vector3Stamped (roll, pitch, yaw in degrees?)

# Output file
COMBINED_CSV_FILE_REL = 'results/odom_euler_combined.csv'

# Get absolute paths
script_dir = os.path.dirname(os.path.abspath(__file__))
bag_file_abs = os.path.join(script_dir, BAG_FILE_REL)
combined_csv_abs = os.path.join(script_dir, COMBINED_CSV_FILE_REL)

print(f"Preprocessing data directly from bag file to CSV:")
print(f"Bag file: {bag_file_abs}")
print(f"Output CSV: {combined_csv_abs}")
print(f"Processing max {MAX_MESSAGES_TO_PROCESS} messages per topic.")

# --- Data Storage ---
odom_data = {} # {timestamp_ns: [x, y, z]}
orient_data = {} # {timestamp_ns: [roll, pitch, yaw]} (Assuming degrees from Vector3Stamped)

# --- Read Data Directly from Bag ---
odom_read_count = 0
orient_read_count = 0
try:
    print(f"\nReading topics from {bag_file_abs}...")
    with rosbag.Bag(bag_file_abs, 'r') as bag:
        # Read Odometry Data
        print(f"Reading {ODOM_TOPIC}...")
        for topic, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
            odom_read_count += 1
            if odom_read_count > MAX_MESSAGES_TO_PROCESS:
                print(f"Reached max messages ({MAX_MESSAGES_TO_PROCESS}) for {ODOM_TOPIC}.")
                break
            try:
                # Extract position (ignore invalid orientation quaternion here)
                pos = msg.pose.pose.position
                timestamp_ns = t.to_nsec() # Use nanoseconds for precise matching
                odom_data[timestamp_ns] = [pos.x, pos.y, pos.z]
            except AttributeError as e:
                print(f"Warning: Skipping malformed odom message #{odom_read_count}: {e}", file=sys.stderr)
        print(f"Read {len(odom_data)} valid odometry messages.")

        # Read Orientation Data (Vector3Stamped)
        print(f"Reading {ORIENT_TOPIC}...")
        for topic, msg, t in bag.read_messages(topics=[ORIENT_TOPIC]):
            orient_read_count += 1
            if orient_read_count > MAX_MESSAGES_TO_PROCESS:
                 print(f"Reached max messages ({MAX_MESSAGES_TO_PROCESS}) for {ORIENT_TOPIC}.")
                 break
            try:
                # Extract Euler angles (assuming x=roll, y=pitch, z=yaw in degrees)
                vec = msg.vector
                timestamp_ns = t.to_nsec()
                orient_data[timestamp_ns] = [vec.x, vec.y, vec.z]
            except AttributeError as e:
                 print(f"Warning: Skipping malformed orientation message #{orient_read_count}: {e}", file=sys.stderr)
        print(f"Read {len(orient_data)} valid orientation messages.")

except FileNotFoundError:
    print(f"Error: Bag file not found - {bag_file_abs}", file=sys.stderr)
    sys.exit(1)
except rosbag.ROSBagException as e:
     print(f"Error reading bag file {bag_file_abs}: {e}", file=sys.stderr)
     # This might catch the deserialization errors if they persist even with python lib
     sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred reading bag {bag_file_abs}: {e}", file=sys.stderr)
    sys.exit(1)

if not odom_data or not orient_data:
    print("Error: Failed to read sufficient data from one or both topics.", file=sys.stderr)
    sys.exit(1)

# --- Combine Data by Timestamp ---
print("\nCombining odometry and orientation data by nearest timestamp...")
combined_data = []
# Sort orientation timestamps for efficient searching
sorted_orient_ts = np.array(sorted(orient_data.keys()))

processed_count = 0
written_count = 0
for odom_ts, pos in odom_data.items():
    processed_count += 1
    # Find the index of the closest orientation timestamp
    idx = np.searchsorted(sorted_orient_ts, odom_ts, side="left")

    closest_orient_ts = -1
    if idx == 0:
        closest_orient_ts = sorted_orient_ts[0]
    elif idx == len(sorted_orient_ts):
        closest_orient_ts = sorted_orient_ts[-1]
    else:
        # Check which neighbour is closer
        ts_before = sorted_orient_ts[idx-1]
        ts_after = sorted_orient_ts[idx]
        if abs(odom_ts - ts_before) < abs(odom_ts - ts_after):
            closest_orient_ts = ts_before
        else:
            closest_orient_ts = ts_after

    if closest_orient_ts != -1:
        orient = orient_data[closest_orient_ts]
        # Combine: timestamp (seconds), x, y, z, roll, pitch, yaw
        timestamp_sec = odom_ts / 1e9
        combined_data.append([timestamp_sec] + pos + orient)
        written_count += 1
    # else: # Should not happen with the logic above unless orient_data is empty
    #    print(f"Warning: Could not find matching orientation for odom_ts {odom_ts}", file=sys.stderr)

print(f"Processed {processed_count} odom messages, found matches for {written_count}.")

# --- Write Combined CSV ---
print(f"\nWriting combined data to {combined_csv_abs}...")
try:
    with open(combined_csv_abs, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        # Header: timestamp (seconds), position, orientation (euler degrees?)
        writer.writerow(['timestamp', 'x', 'y', 'z', 'roll_deg', 'pitch_deg', 'yaw_deg'])
        for row in combined_data:
             # Format numbers for CSV
             formatted_row = [f"{row[0]:.9f}"] + [f"{val:.6f}" for val in row[1:]]
             writer.writerow(formatted_row)
    print(f"Successfully wrote {len(combined_data)} rows to {combined_csv_abs}")
except IOError as e:
    print(f"Error writing CSV file {combined_csv_abs}: {e}", file=sys.stderr)
except Exception as e:
    print(f"An unexpected error occurred writing CSV: {e}", file=sys.stderr)


print("\nPreprocessing script finished.")

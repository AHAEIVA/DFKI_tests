#!/usr/bin/env python3

import rosbag
import sys
import os

# --- Configuration ---
BAG_FILE_REL = 'results/rosbags/2025-04-04-12-49-02.bag'
ODOM_TOPIC = '/mobula/rov/odometry'
ORIENT_TOPIC = '/mobula/rov/orientation' # The Vector3Stamped topic

# Get absolute path
script_dir = os.path.dirname(os.path.abspath(__file__))
bag_file_abs = os.path.join(script_dir, BAG_FILE_REL)

print(f"Inspecting first messages from topics in: {bag_file_abs}")

odom_msg_found = False
orient_msg_found = False

try:
    with rosbag.Bag(bag_file_abs, 'r') as bag:
        print("\n--- Checking Topics ---")
        topics_info = bag.get_type_and_topic_info().topics
        if ODOM_TOPIC in topics_info:
            print(f"Found Odometry topic: {ODOM_TOPIC} (Type: {topics_info[ODOM_TOPIC].msg_type})")
        else:
            print(f"Odometry topic {ODOM_TOPIC} NOT FOUND in bag.")

        if ORIENT_TOPIC in topics_info:
            print(f"Found Orientation topic: {ORIENT_TOPIC} (Type: {topics_info[ORIENT_TOPIC].msg_type})")
            # Verify type matches expected Vector3Stamped
            if topics_info[ORIENT_TOPIC].msg_type != 'geometry_msgs/Vector3Stamped':
                 print(f"WARNING: Orientation topic type is '{topics_info[ORIENT_TOPIC].msg_type}', expected 'geometry_msgs/Vector3Stamped'.")
        else:
            print(f"Orientation topic {ORIENT_TOPIC} NOT FOUND in bag.")


        print(f"\n--- Reading First Odometry Message ({ODOM_TOPIC}) ---")
        # Iterate through messages, only processing the first one found for the target topic
        for topic, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
            print(f"Timestamp: {t}")
            print(msg)
            odom_msg_found = True
            break # Stop after finding the first message
        if not odom_msg_found:
            print("No messages found for the odometry topic.")

        print(f"\n--- Reading First Orientation Message ({ORIENT_TOPIC}) ---")
        # Iterate through messages again for the orientation topic
        for topic, msg, t in bag.read_messages(topics=[ORIENT_TOPIC]):
            print(f"Timestamp: {t}")
            print(msg)
            orient_msg_found = True
            break # Stop after finding the first message
        if not orient_msg_found:
            print("No messages found for the orientation topic.")

except FileNotFoundError:
    print(f"Error: Bag file not found at {bag_file_abs}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"An error occurred while reading the bag file: {e}", file=sys.stderr)
    sys.exit(1)

print("\nInspection finished.")

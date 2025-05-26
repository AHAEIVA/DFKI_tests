#!/usr/bin/env python3

import rosbag
import sys
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

# Configuration
bag_file_rel = 'results/rosbags/2025-04-04-12-49-02.bag'
odom_topic = '/mobula/rov/odometry'
tether_topic = '/tether_path'
odom_output_rel = 'results/rov_pose_trajectory_script.txt'
tether_output_rel = 'results/path_tether_script.txt'

# Get absolute paths based on script location or CWD
script_dir = os.path.dirname(os.path.abspath(__file__))
bag_file_abs = os.path.join(script_dir, bag_file_rel)
odom_output_abs = os.path.join(script_dir, odom_output_rel)
tether_output_abs = os.path.join(script_dir, tether_output_rel)

print(f"Input bag file: {bag_file_abs}")
print(f"Odometry output file: {odom_output_abs}")
print(f"Tether path output file: {tether_output_abs}")

odom_count = 0
tether_count = 0
error_count = 0

try:
    with rosbag.Bag(bag_file_abs, 'r') as bag, \
         open(odom_output_abs, 'w') as odom_file, \
         open(tether_output_abs, 'w') as tether_file:

        # Write headers
        odom_file.write("timestamp,pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w\n")
        tether_file.write("timestamp,point_index,pos_x,pos_y,pos_z\n") # timestamp per point

        print("Processing bag file...")
        for topic, msg, t in bag.read_messages(topics=[odom_topic, tether_topic]):
            try:
                timestamp_sec = t.to_sec()

                if topic == odom_topic:
                    odom_count += 1
                    pos = msg.pose.pose.position
                    orient = msg.pose.pose.orientation
                    odom_file.write(f"{timestamp_sec:.9f},{pos.x:.6f},{pos.y:.6f},{pos.z:.6f},{orient.x:.6f},{orient.y:.6f},{orient.z:.6f},{orient.w:.6f}\n")

                elif topic == tether_topic:
                    tether_count += 1
                    for i, pose_stamped in enumerate(msg.poses):
                        pos = pose_stamped.pose.position
                        # Write each point in the path as a separate row
                        tether_file.write(f"{timestamp_sec:.9f},{i},{pos.x:.6f},{pos.y:.6f},{pos.z:.6f}\n")

            # More specific error handling within the loop
            except (rosbag.ROSBagException, genpy.DeserializationError, struct.error) as specific_e:
                error_count += 1
                print(f"Deserialization/Bag error processing message at time {t.to_sec()}: {specific_e}", file=sys.stderr)
                # Continue processing other messages
            except Exception as e:
                error_count += 1
                print(f"Generic error processing message at time {t.to_sec()}: {e}", file=sys.stderr)
                # Continue processing other messages if possible

except FileNotFoundError:
    print(f"Error: Bag file not found at {bag_file_abs}", file=sys.stderr)
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred: {e}", file=sys.stderr)
    sys.exit(1)

print(f"Processing complete.")
print(f"Odometry messages processed: {odom_count}")
print(f"Tether path messages processed: {tether_count}")
print(f"Errors encountered: {error_count}")
print(f"Data written to:")
print(f" - {odom_output_abs}")
print(f" - {tether_output_abs}")

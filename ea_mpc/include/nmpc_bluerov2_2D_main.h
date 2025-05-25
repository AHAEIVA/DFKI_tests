/**
 * @file   nmpc_bluerov2_2-D_main.h 
 * entagelement aware MPC, current implementation works in conjuction with NEPTUNE planner.
 * @author Hakim Amer
 * @date   Nov 2024
 *
 */

#ifndef NMPC_PC_MAIN_H
#define NMPC_PC_MAIN_H

// ROS and Message Includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <algorithm> // for std::clamp

// Standard Libraries
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>

// Eigen Libraries
#include <Eigen/Core>
#include <Eigen/Dense>

// NMPC Includes
#include <nmpc_bluerov2_2D_main.h>
#include <nmpc_bluerov2_2D.h>


using namespace std;

// Constants
const double TOLERANCE = 1e-6;
double sampleTime = 0.02;

// Global Variables for Position, Reference, and Obstacles
double distx, disty, distz;
double current_ref_x, current_ref_y, current_ref_z;
double test_x, test_y;
double neptune_x, neptune_y;
double tether_end_x = 0.0, tether_end_y = 0.0;
double base_x = 0.0, base_y = 0.0;

bool CONTACT = false;

// Reference Trajectories
std::vector<double> ref_traj_x, ref_traj_y, ref_traj_z;
std::vector<double> neptune_planner_x, neptune_planner_y, neptune_planner_z;
geometry_msgs::Point neptune_goal;

// Tether Positions
std::vector<geometry_msgs::Point> tether_positions;

// Obstacle and Path Data
std::vector<std::vector<double>> obstacle_centers;
std::vector<std::vector<double>> path;

// NMPC Cost Parameters
//std::vector<double> ent_point = {0.0, 0.0, 0.0};
std::vector<double> obs_centre = {0.0, 0.0, 0.0};


// Tether config
std::vector<double> base_pos = {0.0, 0.0};


//Neptune params:
int size_refs = 10;
int neptune_planner_size = 10;

int count_traj = 0;
int neptune_planner_count = 0;

// Control Parameters
double m_in, g_in;
Eigen::VectorXd Uref_in(NMPC_NU);
Eigen::VectorXd W_in(NMPC_NY);

// Reference Position and Orientation
Eigen::Vector3d ref_position, ref_velocity;
double ref_yaw_rad;
int ref_traj_type;
std::vector<double> ref_trajectory;
std::vector<double> angles, angles_d;

// Subscriber and Publisher Definitions
ros::Subscriber state_sub, ref_traj_sub, ref_position_sub, ref_velocity_sub;
ros::Subscriber ref_yaw_sub, ref_point_sub, pos_sub, vel_sub, dist_Fx_predInit_sub;
ros::Subscriber dist_Fy_predInit_sub, dist_Fz_predInit_sub, dist_data_sub;
ros::Subscriber dist_Fx_data_sub, dist_Fy_data_sub, dist_Fz_data_sub;
ros::Subscriber orientation_sub;
ros::Subscriber neptune_state_sub, neptune_trajectory_sub, neptune_goal_sub, neptune_tether_sub;

ros::Publisher att_throttle_pub, attitude_pub, nmpc_cmd_wrench_pub;
ros::Publisher nmpc_cmd_Fz_pub, nmpc_cmd_exeTime_pub, nmpc_cmd_kkt_pub;
ros::Publisher nmpc_cmd_obj_pub, nmpc_ctrl_pub, s_sdot_pub;
ros::Publisher nmpc_pred_traj_pub, odom_point_pub, path_pub;
ros::Publisher pred_traj_viz_pub;
ros::Publisher eampc_tether_pub;;
//ros::Publisher state_to_neptune_pub;
ros::Publisher entanglement_point_pub;
ros::Publisher closest_obstacle_pub;


// Strings for Topic Names
std::string mocap_topic_part, dist_Fx_predInit_topic, dist_Fy_predInit_topic;
std::string dist_Fz_predInit_topic, dist_Fx_data_topic, dist_Fy_data_topic, dist_Fz_data_topic;

// Control Flags
bool online_ref_yaw = false;
bool control_stop = false;
bool use_dist_estimates = false;

// Debugging and Status Flags
int print_flag_offboard = 1, print_flag_arm = 1, print_flag_altctl = 1;
int print_flag_traj_finished = 0;

// Struct for Disturbance Data
struct _dist_struct
{
    bool predInit = false;
    int print_predInit = 1;
    std::vector<double> data;
    std::vector<double> data_zeros;
} dist_Fx, dist_Fy, dist_Fz;

// Timing Variables
double t, t_cc_loop;

// Current State Variables
tf::Quaternion current_att_quat;
tf::Matrix3x3 current_att_mat;
std::vector<double> pos_ref, current_pos_att, current_vel_rate;
std::vector<double> current_vel_body, current_states, current_s_sdot;

// Custom Structures
nmpc_struct_ nmpc_struct;
online_data_struct_ online_data;

// Initialize variables to store entanglement point and closest obstacle data
std::vector<double> ent_point_;  // Declared but not initialized
std::vector<double> closest_obs;  // Declared but not initialized

std::vector<double> err_vector; // Vector to store errors
double err_sum_ = 0;              // Sum of errors over the horizon
double rotate_dir_ = 0.0;
std::vector<double> err_vector_; // Define and initialize with 101 elements set to 0.0

//EA-MPC params
double dist_obs_ = 2.0;       //safety dsitance (not used now)
const int integ_error_h_ = 500;      // integration horizon for the entagelement aware error cost(number of steps)
const double error_sum_factor_ = 0.03; //factor to multiply the integration of error 
const double error_cap_ = 100.0; //factor to multiply the integration of error 

#endif // NMPC_PC_MAIN_H

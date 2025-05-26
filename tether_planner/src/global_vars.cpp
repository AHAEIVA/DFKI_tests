#include "global_vars.hpp"

// Cylinder obstacles
std::vector<cylinder_obs> cylinders;
std::vector<cylinder_obs> cylinders_safe;

// Point cloud and related variables
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr eiva_pcloud(new pcl::PointCloud<pcl::PointXYZ>);
bool voxel_grid_initialized = false;
pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

// Parameters
double time_step;
double L_max;
double delta;                // Step size
double equivalenceTolerance; // Equivalence tolerance
double safe_offset;

// Colors
std_msgs::ColorRGBA tetherColor;
std_msgs::ColorRGBA directPath;
std_msgs::ColorRGBA safePath;
std_msgs::ColorRGBA ropepathColor;
std_msgs::ColorRGBA rovpathColor;
std_msgs::ColorRGBA safepathColor;

// Time
std::chrono::duration<double> time_tether_model_computation;

// Flags
bool reset_tether;
bool record_trajectory;
bool inspection_done = false;
bool TA_Planner_ON;
bool goal_reached;
bool Tether_Length_exceeded = false;
bool USE_NVBLOX_COLLISION_CHECKER;

// Other variables
std::string trajectory_filename;
std::string inspection_model;
std::vector<double> way_point;
int count;
double model_scale;
std::vector<std::vector<double>> way_point_traj;
std::vector<double> base;
std::vector<ompl::base::State *> contactPoints;
ompl::base::RealVectorStateSpace::StateType *state_base;
double scale_factor;
Eigen::Vector3f translation;
Eigen::Matrix3f rotation;
double distanceThreshold;

// Declare ROS publishers as global variables
ros::Publisher rov_path_pub;
ros::Publisher rope_path_pub;
ros::Publisher obstacle_pub;
ros::Publisher tether_path_pub;
ros::Publisher planner_path_pub;
ros::Publisher safe_planner_path_pub;
ros::Publisher direct_path_pub;
ros::Publisher safe_path_pub;
ros::Publisher point_cloud_pub;
ros::Publisher voxel_grid_pub;
ros::Publisher blue_rov_pub;
ros::Publisher trajectory_pub;
ros::Publisher ref_pub;
ros::Publisher point_pub;
ros::Publisher cylinder_pub;
ros::Publisher exit_points_pub;
ros::Publisher bounding_box_pub;
ros::Publisher stl_model_pub;
ros::Publisher vector_pub;

//ros::Subscriber pos_sub;
//ros::Subscriber orientation_sub;
//ros::Subscriber reset_tether_sub;
//ros::Subscriber record_trajectory_sub;
//ros::Subscriber goal_point_sub;


std::vector<double> current_att_quat(4, 0.0); // Quaternion has 4 components
std::vector<double> current_vel_rate(3, 0.0); // Velocity rate has 3 components
std::vector<double> current_pos_att(3, 0.0); // Position and attitude have 3 components
bool new_data_received = false;
std::vector<double> current_vel_body(3, 0.0); // Velocity body has 3 components
std::vector<double> angles(3, 0.0); // Angles have 3 components
std::vector<double> angles_d(3, 0.0); // Angles in degrees have 3 components
std::vector<double> goal(3, 0.0);
std::vector<double> goal_t1(3, 0.0);
std::vector<double> viz_point(3,0.0);
ompl::geometric::PathGeometric P_t(nullptr); // Initialize with nullptr
ompl::geometric::PathGeometric Path_sample(nullptr); // Initialize with nullptr



Eigen::Vector3f max_point;
Eigen::Vector3f min_point;

std::string txtfilePath;
std::string nvblx_file_path;
std::string input_xyz_file;
std::string stl_file;

//nblox
std::unique_ptr<nvblox::Mapper> mapper;
const nvblox::TsdfLayer* tsdf_layer;
const nvblox::MeshLayer* mesh_layer;

std::vector<Triangle> inspection_model_stl;

std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> stl_mesh;

double collision_threshold;
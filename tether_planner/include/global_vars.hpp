#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/Bool.h>

#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <std_msgs/ColorRGBA.h>  // Include the correct header

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "nvblox/core/types.h"
#include "nvblox/mapper/mapper.h"
#include <nvblox/integrators/esdf_slicer.h>
#include <nvblox/mesh/mesh.h>
#include <nvblox/map/unified_3d_grid.h>

#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/distance.h>


// Define a structure for a cylinder
struct cylinder_obs
{
    Eigen::Vector3f baseCenter;
    Eigen::Vector3f axis;
    float radius;
    float height;
};


struct Triangle {
    Eigen::Vector3f normal;
    Eigen::Vector3f vertex1;
    Eigen::Vector3f vertex2;
    Eigen::Vector3f vertex3;
  };

// List of cylinders
extern std::vector<cylinder_obs> cylinders;
extern std::vector<cylinder_obs> cylinders_safe;

// Point cloud and related variables
extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
extern bool voxel_grid_initialized;
extern pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
extern pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

// Parameters
extern double time_step;
extern double L_max;
extern double delta;                // Step size
extern double equivalenceTolerance; // Equivalence tolerance
extern double safe_offset;

// Colors
extern std_msgs::ColorRGBA tetherColor;
extern std_msgs::ColorRGBA directPath;
extern std_msgs::ColorRGBA safePath;
extern std_msgs::ColorRGBA ropepathColor;
extern std_msgs::ColorRGBA rovpathColor;
extern std_msgs::ColorRGBA safepathColor;

// Time
extern std::chrono::duration<double> time_tether_model_computation;

// Flags
extern bool reset_tether;
extern bool record_trajectory;
extern bool inspection_done;
extern bool TA_Planner_ON;
extern bool goal_reached;
extern bool Tether_Length_exceeded;
extern bool USE_NVBLOX_COLLISION_CHECKER;

// Other variables
extern std::string trajectory_filename;
extern std::string inspection_model;
extern std::vector<double> way_point;
extern int count;
extern double model_scale;
extern double scale_factor;
extern std::vector<std::vector<double>> way_point_traj;

extern std::vector<double> current_att_quat;
extern std::vector<double> current_vel_rate;
extern std::vector<double> current_pos_att;
extern bool new_data_received;
extern std::vector<double> current_vel_body;
extern std::vector<double> angles;
extern std::vector<double> angles_d;
extern std::vector<double> goal;
extern std::vector<double> goal_t1;
extern Eigen::Vector3f translation;
extern Eigen::Matrix3f rotation;
extern std::vector<Triangle> inspection_model_stl;


// Declare ROS publishers as global variables
extern ros::Publisher rov_path_pub;
extern ros::Publisher rope_path_pub;
extern ros::Publisher obstacle_pub;
extern ros::Publisher tether_path_pub;
extern ros::Publisher planner_path_pub;
extern ros::Publisher safe_planner_path_pub;
extern ros::Publisher direct_path_pub;
extern ros::Publisher safe_path_pub;
extern ros::Publisher point_cloud_pub;
extern ros::Publisher voxel_grid_pub;
extern ros::Publisher blue_rov_pub;
extern ros::Publisher trajectory_pub;
extern ros::Publisher ref_pub;
extern ros::Publisher point_pub;
extern ros::Publisher cylinder_pub;
extern ros::Publisher exit_points_pub;
extern ros::Publisher bounding_box_pub;
extern ros::Publisher stl_model_pub;
extern ros::Publisher vector_pub;

extern ros::Subscriber pos_sub;
extern ros::Subscriber orientation_sub;
extern ros::Subscriber reset_tether_sub;
extern ros::Subscriber record_trajectory_sub;
extern ros::Subscriber goal_point_sub;

extern std::vector<double> base;
extern std::vector<ompl::base::State *> contactPoints;
extern ompl::base::RealVectorStateSpace::StateType *state_base;
extern Eigen::Vector3f max_point;
extern Eigen::Vector3f min_point;
extern std::vector<double> viz_point;
extern ompl::geometric::PathGeometric P_t;
extern ompl::geometric::PathGeometric Path_sample;


extern double distanceThreshold ;


extern std::string txtfilePath;
extern std::string nvblx_file_path;
extern std::string input_xyz_file;
extern std::string stl_file;


extern std::unique_ptr<nvblox::Mapper> mapper;
extern const nvblox::TsdfLayer *tsdf_layer;
extern const nvblox::MeshLayer *mesh_layer;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr eiva_pcloud;

//nvblox
extern std::unique_ptr<nvblox::Mapper> mapper;
extern const nvblox::TsdfLayer* tsdf_layer;
extern const nvblox::MeshLayer* mesh_layer;


extern std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> stl_mesh;

extern double collision_threshold;
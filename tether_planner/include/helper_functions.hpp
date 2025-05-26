#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP
#include "publishers.hpp"
#include "global_vars.hpp"
#include "my_motion_validator.hpp"
#include <Eigen/Dense>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "collision_checker.hpp"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>





#include <pcl/common/transforms.h> // Include the necessary header for PCL transformations
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/message.h>
#include <ros/message_event.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>


#include "nvblox_functions.hpp"
// External variables


// Callback functions

void pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
void goal_cb(const geometry_msgs::PointStamped::ConstPtr &msg);
void reset_tether_cb(const std_msgs::Bool::ConstPtr &msg);
void record_trajectory_on_cb(const std_msgs::Bool::ConstPtr &msg);

// Utility functions
void transformAndScaleTriangles(std::vector<Triangle> &triangles, float scale_factor = 0.1, 
    const Eigen::Vector3f &translation = Eigen::Vector3f::Zero(), 
    const Eigen::Matrix3f &rotation = Eigen::Matrix3f::Identity());
void printTriangles(const std::vector<Triangle> &triangles) ;
void loadParameters(ros::NodeHandle &nh);
bool goal_updated(const std::vector<double> &vec1,
                  const std::vector<double> &vec2, double threshold_distance);
std::vector<std::vector<double>> sparsifyTrajectory(const std::string &filePath,
                                                    double distanceThreshold);
double distance(const std::vector<double> &wp1, const std::vector<double> &wp2);
void printTrajectory(const std::vector<std::vector<double>> &trajectory);
void transformWaypoints(std::vector<std::vector<double>> &waypoints,
                        float scale_factor, const Eigen::Vector3f &translation,
                        const Eigen::Matrix3f &rotation);
bool isPointInsideCylinder(const Eigen::Vector3f &point,
                           const cylinder_obs &cylinder);
bool isStateValid(const ompl::base::State *state);
bool isStateValid_safe(const ompl::base::State *state);
void densifyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void initializeVoxelGridAndKdTree(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         float scale_factor, const Eigen::Vector3f &translation,
                         const Eigen::Matrix3f &rotation);
void initializeTrajectoryFilename();
void recordRosbag(const std::string &bag_filename,
                  const std::vector<std::string> &topics,
                  ros::Duration duration);
std::vector<double> crossProduct(const std::vector<double> &v1,
                                 const std::vector<double> &v2);
std::vector<double> normalize(const std::vector<double> &v);
std::vector<double> findPlaneNormal(const std::vector<double> &v1,
                                    const std::vector<double> &v2);
std::vector<double>
findPerpendicularLineDirection(const std::vector<double> &v1,
                               const std::vector<double> &a);
std::vector<std::vector<double>>
sampleAtDistance(const std::vector<double> &start,
                 const std::vector<double> &direction, double delta);
void saveTetherPathData(const ros::Time &ros_time,
                        const ompl::geometric::PathGeometric &tether);
void saveTrajectory(const ros::Time &ros_time,
                    const std::vector<double> &rov_pos);
void loadAndTransformWaypoints(
    const std::string &filePath, double distanceThreshold, float scale_factor,
    const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation,
    std::vector<std::vector<double>> &way_point_traj);
void initializeOMPLSpace(std::shared_ptr<ompl::base::SpaceInformation> &si,
        std::shared_ptr<ompl::base::SpaceInformation> &si_t);
void initializePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void initializeObstacles();
void setTxtFilePath(const std::string &current_directory, const std::string &model_name, std::string &file_path);
void setNVBloxFilePath(const std::string &current_directory, const std::string &model_name, std::string &file_path);
void setxyzFilePath(const std::string &current_directory, const std::string &model_name, std::string &file_path);
void printPath(const ompl::geometric::PathGeometric &path);
void initializeGlobalVariables();
// Struct to hold triangle data
void convertPCDToXYZ(const std::string &input_pcd_file, const std::string &output_xyz_file);
Eigen::Vector3f getESDFGradient(const nvblox::EsdfLayer& esdf_layer, const Eigen::Vector3f& point);
// Function to read an STL file
std::vector<Triangle> readSTLFile(const std::string &filename);
void initializeROS(int argc, char **argv, ros::NodeHandle &nh);

void initializeSubscribers(ros::NodeHandle &nh);
void initializePublishers(ros::NodeHandle &nh);

void loadNvbloxModel(const std::string& file_path);

void loadSTLModel(const std::string& file_path);

void loadXYZFileToPCL(const std::string &filename);
Eigen::Vector3f findClosestNormal(const Eigen::Vector3f &point, const std::vector<Triangle> &triangles);
bool CollisionCheckSTL(const std::vector<Triangle> &triangles, const fcl::CollisionObjectf &object);
bool isStateValidSTL(const ompl::base::State *state);
void initializeSTLMesh();
bool isPointInsideMesh(const Eigen::Vector3f &point);
void transformEIVAPC(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation);
std::vector<ompl::base::State*> generateHelicalPath(std::shared_ptr<ompl::base::SpaceInformation> si, double radius, double height, int num_points);

void sparsifyAndSavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, float leaf_size, const std::string &output_file);
void removeWaypoint(std::vector<std::vector<double>> &way_point_traj, int index);
void transformSTLModel(std::vector<Triangle> &triangles, float scale_factor, 
    const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation);

// Function to calculate the distance from a point to a triangle
float pointToTriangleDistance(const Eigen::Vector3f &point, 
    const Triangle &triangle);


// Function to find the closest normal vector of the STL model to the point
Eigen::Vector3f findClosestNormal(const Eigen::Vector3f &point, 
    const std::vector<Triangle> &triangles);


    Eigen::Vector3f findClosestTriangleCenter(const Eigen::Vector3f &point,
         const std::vector<Triangle> &triangles);

float pointToTriangleCenterDistance(const Eigen::Vector3f &point,
     const Triangle &triangle);


double calculateDistance(const std::vector<double> &point1,
     const std::vector<double> &point2);

#endif // HELPER_FUNCTIONS_HPP
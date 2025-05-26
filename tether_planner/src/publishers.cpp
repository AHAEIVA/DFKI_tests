#include "publishers.hpp"
#include <ros/ros.h>


void publishBlueRovMarker(ros::Publisher& rov_path_pub, 
    const std::vector<double>& current_pos_att, 
    const std::vector<double>& current_att_quat,
    const std::string& frame_id) {
   

    visualization_msgs::Marker marker;

    double distance = 0.2; // Distance to add in the x direction
    double yaw = angles[2]; // Assuming yaw is stored in the third element of the quaternion

    // Transform the distance in the x direction based on the yaw angle
    double transformed_x = current_pos_att[0] + distance * cos(yaw);
    double transformed_y = current_pos_att[1] + distance * sin(yaw);


    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "blue_rov";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    //marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set position from current_pos_att
//    marker.pose.position.x = current_pos_att[0];
 //   marker.pose.position.y = current_pos_att[1];
    marker.pose.position.x = transformed_x;
    marker.pose.position.y = transformed_y;
    marker.pose.position.z = current_pos_att[2];

    // Convert roll, pitch, yaw to quaternion
    tf2::Quaternion q;
   q.setRPY(angles[0], angles[1],angles[2]+M_PI/2);
   marker.pose.orientation.x = q.x();
   marker.pose.orientation.y = q.y();
   marker.pose.orientation.z = q.z();
   marker.pose.orientation.w = q.w();


  // ROS_INFO_STREAM("\033[1;31mRoll: " << 
  //  angles[0] << ", Pitch: " << angles[1] <<
  //   ", Yaw: " << angles[2] << "\033[0m");



    // Set scale
    marker.scale.x = 0.002;
    marker.scale.y = 0.002;
    marker.scale.z = 0.002;

    // Set color (optional)
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0; // Fully visible

    // Mesh file path
    marker.mesh_resource = "package://tether_planner/models/saab_simple.stl";
   // marker.mesh_resource = "https://raw.githubusercontent.com/gundam-global-challenge/gundam_robot/master/gundam_rx78_description/meshes/rx78_object_005-lib.dae";
  //  marker.mesh_resource = "https://github.com/patrickelectric/bluerov_ros_playground/blob/master/model/BlueRov2/meshes/BlueRov2.stl";

    // Publish the marker
    rov_path_pub.publish(marker);
}


void publishCylinders(ros::Publisher &publisher, const std::vector<cylinder_obs> &cylinders, const std::string &frame_id)
{
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < cylinders.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cylinders";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cylinders[i].baseCenter.x();
        marker.pose.position.y = cylinders[i].baseCenter.y();
        marker.pose.position.z = -cylinders[i].baseCenter.z();

        // Calculate the quaternion for the cylinder's orientation
        Eigen::Vector3f z_axis(0.0, 0.0, 1.0);
        Eigen::Quaternionf quaternion = Eigen::Quaternionf::FromTwoVectors(z_axis, cylinders[i].axis);

        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        marker.scale.x = cylinders[i].radius * 2.0;
        marker.scale.y = cylinders[i].radius * 2.0;
        marker.scale.z = cylinders[i].height;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.4;
        marker_array.markers.push_back(marker);
    }
    publisher.publish(marker_array);
}




void publishTrajectory(const ros::Publisher &publisher, const std::vector<std::vector<double>> &trajectory) {
    // Create the message to hold the trajectory markers
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "world";  // Set the reference frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;  // No rotation

    marker.scale.x = 0.2;  // Size of the spheres
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 0.5;  // Semi-transparent
    marker.color.r = 0.8;  // Red color
    marker.color.g = 0.4;  // Green color
    marker.color.b = 0.0;  // Blue color

    // Add the points from the trajectory as markers
    for (const auto &waypoint : trajectory) {
        if (waypoint.size() >= 3) {  // Ensure there are at least 3 elements (X, Y, Z)
            geometry_msgs::Point point;
            point.x = waypoint[0];  // X
            point.y = waypoint[1];  // Y
            point.z = waypoint[2];  // Z

            marker.points.push_back(point);
        }
    }

    // Publish the markers
    publisher.publish(marker);
    //ROS_INFO("Published trajectory as markers");
}






void publishTetherPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id, const std_msgs::ColorRGBA &color)
{
    nav_msgs::Path tether_path_msg;
    tether_path_msg.header.frame_id = frame_id;
    tether_path_msg.header.stamp = ros::Time::now();

    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = ros::Time::now();  // Use current time
        pose.pose.position.x = state->values[0];
        pose.pose.position.y = state->values[1];
        pose.pose.position.z = state->values[2];
        pose.pose.orientation.w = 1.0;  // Default orientation (can be updated for full 6DOF path if needed)
        tether_path_msg.poses.push_back(pose);
    }

    // Create a marker for visualization
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "tether_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;  // Line width
    marker.color = color;

    for (const auto &pose : tether_path_msg.poses)
    {
        geometry_msgs::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y;
        p.z = pose.pose.position.z;
        marker.points.push_back(p);
    }

    pub.publish(tether_path_msg);
    pub.publish(marker);


}






void publishPointCloud(const ros::Publisher &publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";
    publisher.publish(output);
  
    // Print the published points
    //for (const auto &point : cloud->points) {
    //  std::cout << "Published point: " << point.x << ", " << point.y << ", " << point.z << std::endl;
   // }
}





void publishVoxelGrid(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}








// Function to create and publish a path
void publishPath(ros::Publisher &pub, const ompl::geometric::PathGeometric &path, const std::string &frame_id, const std::string &ns, const std_msgs::ColorRGBA &color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color = color;  // Use the provided color

    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        geometry_msgs::Point p;
        p.x = state->values[0];
        p.y = state->values[1];
        p.z = state->values[2];
        marker.points.push_back(p);
    }

    pub.publish(marker);
}

// Function to create and publish obstacles


void publishRef(const ros::Publisher &publisher, const std::vector<double> &point) {
   

    //if (point.size() < 4) {
    //    point.push_back(0.0);  // Add yaw of zero
    //}


    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";  // Set the reference frame
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = point[0];  // X
    pose_msg.pose.position.y = point[1];  // Y
    pose_msg.pose.position.z = point[2];  // Z

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, point[3]);  // Roll and pitch are 0, yaw is point[3]
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    publisher.publish(pose_msg);
    //ROS_INFO_STREAM("\033[1;31mYaw angle: " << point[3] << "\033[0m");

   // ROS_INFO("Published pose: [%f, %f, %f, %f]", point[0], point[1], point[2], point[3]);
}




void publishExitPoints(const std::vector<std::vector<double>>& exit_points, ros::Publisher& publisher)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& point : exit_points)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world"; // Set the frame ID
        marker.header.stamp = ros::Time::now();
        marker.ns = "exit_points";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point[0];
        marker.pose.position.y = point[1];
        marker.pose.position.z = point[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15; // Set the scale of the marker
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.a = 1.0; // Set the alpha value
        marker.color.r = 1.0; // Set the color (red)
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
    }

    publisher.publish(marker_array);
}




void publishBoundingBox(const ros::Publisher &publisher, const Eigen::Vector3f &min_point, const Eigen::Vector3f &max_point, const std::string &frame_id, std_msgs::ColorRGBA &color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Calculate the center of the bounding box
    Eigen::Vector3f center = (min_point + max_point) / 2.0f;

    // Set the position of the marker
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();

    // Set the orientation of the marker (no rotation)
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker (size of the bounding box)
    marker.scale.x = max_point.x() - min_point.x();
    marker.scale.y = max_point.y() - min_point.y();
    marker.scale.z = max_point.z() - min_point.z();

    // Set the color of the marker
    color.a = 0.3; // Semi-transparent

    marker.color = color;

    // Publish the marker
    publisher.publish(marker);
}



void publishAllData() {
    publishPointCloud(point_cloud_pub, cloud);
    publishPath(rope_path_pub, P_t, "world", "rope_path", ropepathColor);
    publishTetherPath(tether_path_pub, P_t, "world", tetherColor);
    publishTrajectory(trajectory_pub, way_point_traj);
    publishRef(ref_pub, way_point);
    //publishRef(point_pub, viz_point);
    publishCylinders(cylinder_pub, cylinders, "world");
    publishBlueRovMarker(blue_rov_pub, current_pos_att, current_att_quat, "world");
    publishTetherPath(planner_path_pub, Path_sample, "world", rovpathColor);
    publishBoundingBox(bounding_box_pub, min_point, max_point, "world", rovpathColor);
    publishSTLModel(inspection_model_stl);
    
}


void publishSTLModel(const std::vector<Triangle> &triangles) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "stl_model";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  
    for (const auto &triangle : triangles) {
        geometry_msgs::Point p1, p2, p3;
        p1.x = triangle.vertex1.x();
        p1.y = triangle.vertex1.y();
        p1.z = triangle.vertex1.z();
        p2.x = triangle.vertex2.x();
        p2.y = triangle.vertex2.y();
        p2.z = triangle.vertex2.z();
        p3.x = triangle.vertex3.x();
        p3.y = triangle.vertex3.y();
        p3.z = triangle.vertex3.z();
  
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);

        // Calculate and print the center of the triangle
        geometry_msgs::Point center;
        center.x = (p1.x + p2.x + p3.x) / 3.0;
        center.y = (p1.y + p2.y + p3.y) / 3.0;
        center.z = (p1.z + p2.z + p3.z) / 3.0;
       // ROS_INFO("Triangle center: [%f, %f, %f]", center.x, center.y, center.z);
    }
  
    marker_array.markers.push_back(marker);
    stl_model_pub.publish(marker_array);
    //ROS_INFO("STL model published");

    // Print the number of triangles
    //ROS_INFO("Number of triangles: %lu", triangles.size());
}



void publishVector(const Eigen::Vector3f& start, const Eigen::Vector3f& vector, const ros::Publisher& pub, const std::string& frame_id, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "vector";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the start and end points of the arrow
    geometry_msgs::Point p_start, p_end;
    p_start.x = start.x();
    p_start.y = start.y();
    p_start.z = start.z();
    p_end.x = start.x() + vector.x();
    p_end.y = start.y() + vector.y();
    p_end.z = start.z() + vector.z();

    marker.points.push_back(p_start);
    marker.points.push_back(p_end);

    // Set the scale of the arrow
    marker.scale.x = 0.1; // Shaft diameter
    marker.scale.y = 0.2; // Head diameter
    marker.scale.z = 0.2; // Head length

    // Set the color of the arrow
    marker.color.a = 1.0; // Alpha
    marker.color.r = 1.0; // Red
    marker.color.g = 0.0; // Green
    marker.color.b = 0.0; // Blue

    // Publish the marker
    pub.publish(marker);
}



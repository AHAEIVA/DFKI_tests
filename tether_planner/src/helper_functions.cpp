#include "helper_functions.hpp"

void printTriangles(const std::vector<Triangle> &triangles) {
  if (triangles.empty()) {
      std::cout << "Triangles are empty" << std::endl;
      return;
  }

  for (const auto &triangle : triangles) {
      std::cout << "Normal: [" << triangle.normal.x() << ", " << triangle.normal.y() << ", " << triangle.normal.z() << "]" << std::endl;
      std::cout << "Vertex 1: [" << triangle.vertex1.x() << ", " << triangle.vertex1.y() << ", " << triangle.vertex1.z() << "]" << std::endl;
      std::cout << "Vertex 2: [" << triangle.vertex2.x() << ", " << triangle.vertex2.y() << ", " << triangle.vertex2.z() << "]" << std::endl;
      std::cout << "Vertex 3: [" << triangle.vertex3.x() << ", " << triangle.vertex3.y() << ", " << triangle.vertex3.z() << "]" << std::endl;
  }
}


void transformAndScaleTriangles(std::vector<Triangle> &triangles, float scale_factor, 
  const Eigen::Vector3f &translation, 
  const Eigen::Matrix3f &rotation) {
for (auto &triangle : triangles) {
// Scale vertices
triangle.vertex1 *= scale_factor;
triangle.vertex2 *= scale_factor;
triangle.vertex3 *= scale_factor;

// Apply rotation
triangle.vertex1 = rotation * triangle.vertex1;
triangle.vertex2 = rotation * triangle.vertex2;
triangle.vertex3 = rotation * triangle.vertex3;

// Apply translation
triangle.vertex1 += translation;
triangle.vertex2 += translation;
triangle.vertex3 += translation;

// Transform the normal
triangle.normal = rotation * triangle.normal;
}
}


bool isPointInsideMesh(const Eigen::Vector3f &point) {
  // Create an FCL point as a tiny sphere
  auto point_geometry = std::make_shared<fcl::Spheref>(0.001); // Very small sphere
  fcl::CollisionObjectf point_object(point_geometry);

  // Set the transform of the point object
  fcl::Transform3f point_transform;
  point_transform.translation() = point;
  point_object.setTransform(point_transform);
  point_object.computeAABB();

  // Create collision object for STL mesh
  fcl::CollisionObjectf stl_object(stl_mesh);

  // Perform a distance query
  fcl::DistanceRequestf request;
  fcl::DistanceResultf result;
  fcl::distance(&stl_object, &point_object, request, result);


  std::cout << "Distance from point to mesh: " << result.min_distance << std::endl;

  // If the distance is close to zero, the point is on or inside the mesh
  return result.min_distance < 0.0001;
}

bool isStateValidSTL(const ompl::base::State *state) {
  const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
  if (!realState) {
      return false;
  }

  // Extract robot position
  Eigen::Vector3f robot_position(realState->values[0], realState->values[1], realState->values[2]);

  // Collision object for STL mesh
  fcl::CollisionObjectf stl_object(stl_mesh);

  // Create a small sphere to represent the robot
  auto robot_geometry = std::make_shared<fcl::Spheref>(0.1);
  fcl::CollisionObjectf robot_object(robot_geometry);
  fcl::Transform3f robot_transform;
  robot_transform.translation() = robot_position;
  robot_object.setTransform(robot_transform);
  robot_object.computeAABB();

  // Perform collision check
  fcl::CollisionRequestf request;
  fcl::CollisionResultf result;
  fcl::collide(&stl_object, &robot_object, request, result);

  // If collision is detected, return false
  if (result.isCollision()) {
      return false;
  }

  // If no collision, check if the point is inside the mesh
  return !isPointInsideMesh(robot_position);
}
/*
bool isStateValidSTL(const ompl::base::State *state) {
  // Cast the state to RealVectorStateSpace::StateType
  const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
  if (!realState) {
      // ROS_ERROR("State is not of type RealVectorStateSpace::StateType.");
      return false;
  }

  // Access the position values
  double x = realState->values[0];
  double y = realState->values[1];
  double z = realState->values[2];

  // Convert position to Eigen vector
  Eigen::Vector3f robot_position(x, y, z);

  // Create FCL collision object for the STL model
  fcl::CollisionObjectf stl_object(stl_mesh);

  // Create FCL collision object for the robot position
  fcl::CollisionObjectf robot_object(std::make_shared<fcl::Spheref>(0.1)); // Assuming a sphere with radius 0.1
  robot_object.setTranslation(robot_position);

  // Perform collision checking
  fcl::CollisionRequestf request;
  fcl::CollisionResultf result;
  fcl::collide(&stl_object, &robot_object, request, result);

  return !result.isCollision();
}
*/

void initializeSTLMesh() {
  // Create a vector of FCL triangles
  std::vector<fcl::Triangle> fcl_triangles;
  std::vector<fcl::Vector3f> vertices;

  for (const auto &triangle : inspection_model_stl) {
      fcl::Vector3f v1(triangle.vertex1.x(), triangle.vertex1.y(), triangle.vertex1.z());
      fcl::Vector3f v2(triangle.vertex2.x(), triangle.vertex2.y(), triangle.vertex2.z());
      fcl::Vector3f v3(triangle.vertex3.x(), triangle.vertex3.y(), triangle.vertex3.z());

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);

      fcl_triangles.emplace_back(vertices.size() - 3, vertices.size() - 2, vertices.size() - 1);
  }

  // Create FCL collision geometry
  stl_mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSf>>();
  stl_mesh->beginModel();
  stl_mesh->addSubModel(vertices, fcl_triangles);
  stl_mesh->endModel();
}




bool isStateValid(const ompl::base::State *state) {
  // Cast the state to RealVectorStateSpace::StateType
  const auto *realState =
      state->as<ompl::base::RealVectorStateSpace::StateType>();
  if (!realState) {
    // ROS_ERROR("State is not of type RealVectorStateSpace::StateType.");
    return false;
  }

  // Access the position values
  double x = realState->values[0];
  double y = realState->values[1];
  double z = realState->values[2];

  // Convert position to Eigen vector
  Eigen::Vector3f robot_position(x, y, z);

  // Print the robot's position
  // ROS_INFO("Checking state at position: [%f, %f, %f]", x, y, z);

  // Check if the robot's position is inside any cylinder
  for (const auto &cylinder : cylinders) {
    if (isPointInsideCylinder(robot_position, cylinder)) {
      // ROS_WARN("Collision detected with cylinder.");
      return false; // Collision detected with cylinder
    }
  }

  // ROS_INFO("State is valid.");
  return true; // No collision
}

bool isStateValid_safe(const ompl::base::State *state) {
  // Cast the state to RealVectorStateSpace::StateType
  const auto *realState =
      state->as<ompl::base::RealVectorStateSpace::StateType>();
  if (!realState) {
    // ROS_ERROR("State is not of type RealVectorStateSpace::StateType.");
    return false;
  }

  // Access the position values
  double x = realState->values[0];
  double y = realState->values[1];
  double z = realState->values[2];

  // Convert position to Eigen vector
  Eigen::Vector3f robot_position(x, y, z);

  // Print the robot's position
  // ROS_INFO("Checking state at position: [%f, %f, %f]", x, y, z);

  // Check if the robot's position is inside any cylinder in cylinders_safe
  for (const auto &cylinder : cylinders_safe) {
    if (isPointInsideCylinder(robot_position, cylinder)) {
      // ROS_WARN("Collision detected with cylinder.");
      return false; // Collision detected with cylinder
    }
  }

  // ROS_INFO("State is valid.");
  return true; // No collision
}





std::vector<double> findPlaneNormal(const std::vector<double> &v1,
                                    const std::vector<double> &v2) {
  return crossProduct(v1, v2);
}

std::vector<double>
findPerpendicularLineDirection(const std::vector<double> &v1,
                               const std::vector<double> &a) {
  return crossProduct(v1, a); // Direction of the line
}

bool isPointInsideCylinder(const Eigen::Vector3f &point,
                           const cylinder_obs &cylinder) {
  // Print the base center of the cylinder
  // std::cout << "Cylinder Base Center: " << cylinder.baseCenter.transpose() <<
  // std::endl;

  // Compute the vector from the base center to the point
  Eigen::Vector3f baseToPoint;
  baseToPoint.x() = point.x() - cylinder.baseCenter.x();
  baseToPoint.y() = point.y() - cylinder.baseCenter.y();
  baseToPoint.z() = point.z() + cylinder.baseCenter.z();

  // Print the baseToPoint
  // std::cout << "Base to Point: " << baseToPoint.transpose() << std::endl;

  // Normalize the axis of the cylinder
  Eigen::Vector3f axis = cylinder.axis.normalized();

  // Project the vector onto the cylinder's axis to get the height component
  float projectionLength;
  if (axis == Eigen::Vector3f(0, 0, 1)) {
    // If the cylinder extends along the z-axis, use the z-component for
    // projection length
    projectionLength = baseToPoint.z();
  } else {
    // Otherwise, project the vector onto the cylinder's axis
    projectionLength = -baseToPoint.x();
  }

  // Print the projection length
  // std::cout << "Projection Length: " << projectionLength << std::endl;

  // Check if the projection is within the cylinder's height range
  if (projectionLength < 0 || projectionLength > cylinder.height) {
    return false;
  }

  // Compute the closest point on the cylinder axis
  Eigen::Vector3f closestPointOnAxis =
      cylinder.baseCenter + projectionLength * axis;

  // Print the closest point on the axis
  // std::cout << "Closest Point on Axis: " << closestPointOnAxis.transpose() <<
  // std::endl;

  // Compute the radial distance based on the cylinder's axis
  float radialDistance;
  if (axis == Eigen::Vector3f(0, 0, 1)) {
    // If the cylinder extends along the z-axis, compute the radial distance in
    // the XY plane
    radialDistance = std::sqrt(baseToPoint.x() * baseToPoint.x() +
                               baseToPoint.y() * baseToPoint.y());
  } else {
    // Compute the radial distance by removing the axial component
    radialDistance = std::sqrt(baseToPoint.z() * baseToPoint.z() +
                               baseToPoint.y() * baseToPoint.y());
  }

  // Print the radial distance
  // std::cout << "Radial Distance: " << radialDistance << std::endl;

  // Check if the point is within the cylinder's radius
  bool isInside = radialDistance <= cylinder.radius;

  // Print the result
  // std::cout << "Is Inside: " << isInside << std::endl;

  return isInside;
}

// Custom motion validator class

bool goal_updated(const std::vector<double> &vec1,
                  const std::vector<double> &vec2, double threshold_distance) {
  if (vec1.size() != vec2.size()) {
    return true;
  }
  double distance_squared = 0.0;
  for (size_t i = 0; i < vec1.size(); ++i) {
    distance_squared += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
  }
  return distance_squared > (threshold_distance * threshold_distance);
}

void reset_tether_cb(const std_msgs::Bool::ConstPtr &msg) {
  reset_tether = msg->data; // Assign the data from the message to the boolean
}

void record_trajectory_on_cb(const std_msgs::Bool::ConstPtr &msg) {
  record_trajectory =
      msg->data; // Assign the data from the message to the boolean
}

void goal_cb(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // Process the received goal point
  // ROS_INFO("Received goal point: [x: %f, y: %f, z: %f]", msg->point.x,
  // msg->point.y, msg->point.z);
  // Update the goal with the received point
  goal[0] = msg->point.x;
  goal[1] = msg->point.y;
  goal[2] = msg->point.z;
}

void pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  current_att_quat = {
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  current_vel_rate = {msg->twist.twist.linear.x,  msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z,  msg->twist.twist.angular.x,
                      msg->twist.twist.angular.y, msg->twist.twist.angular.z};
  current_pos_att = {msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z,
                     0.0,
                     0.0,
                     0.0}; // roll, pitch, yaw can be computed
  new_data_received = true;
  //ROS_INFO_STREAM("\033[1;31mcurrent_pos_att: ["
   // << current_att_quat[0] << ", "
   // << current_att_quat[1] << ", "
  //  << current_att_quat[2] << "]\033[0m");

  //ROS_INFO("current_pos_att: [%f, %f, %f]", current_pos_att[0], current_pos_att[1], current_pos_att[2]);
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  current_vel_body = {msg->twist.linear.x,  msg->twist.linear.y,
                      msg->twist.linear.z,  msg->twist.angular.x,
                      msg->twist.angular.y, msg->twist.angular.z};
}

void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
  angles = {msg->vector.x * (M_PI / 180), msg->vector.y * (M_PI / 180),
            msg->vector.z * (M_PI / 180)};
  angles_d = {msg->vector.x, msg->vector.y, msg->vector.z};
}

////////////////////////
////////////////////////

// Function to compute the Euclidean distance between two waypoints
double distance(const std::vector<double> &wp1,
                const std::vector<double> &wp2) {
  return std::sqrt(std::pow(wp2[0] - wp1[0], 2) + // X
                   std::pow(wp2[1] - wp1[1], 2) + // Y
                   std::pow(wp2[2] - wp1[2], 2)); // Z
}

std::vector<std::vector<double>> sparsifyTrajectory(const std::string &filePath,
                                                    double distanceThreshold) {
  std::vector<std::vector<double>> way_point_traj;
  std::ifstream inputFile(filePath);

  if (!inputFile.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    return way_point_traj;
  }

  std::string line;
  std::vector<double> previousWaypoint = {0, 0, 0,
                                          0}; // Initialize previous waypoint
  bool isFirstPoint = true;

  while (std::getline(inputFile, line)) {
    std::stringstream ss(line);
    std::string timestampStr, xStr, yStr, zStr, pitchStr, yawStr;

    std::getline(ss, timestampStr, ',');
    std::getline(ss, xStr, ',');
    std::getline(ss, yStr, ',');
    std::getline(ss, zStr, ',');
    std::getline(ss, pitchStr, ',');
    std::getline(ss, yawStr, ',');

    double x = std::stod(xStr.substr(3));     // Remove 'X: ' part
    double y = std::stod(yStr.substr(3));     // Remove 'Y: ' part
    double z = std::stod(zStr.substr(3));     // Remove 'Z: ' part
    double yaw = std::stod(yawStr.substr(5)); // Remove 'YAW: ' part

    // Sparsify based on the Euclidean distance threshold
    std::vector<double> currentWaypoint = {x, y, z, yaw};
    if (isFirstPoint ||
        distance(previousWaypoint, currentWaypoint) > distanceThreshold) {
      way_point_traj.push_back(currentWaypoint);
      previousWaypoint = currentWaypoint;
      isFirstPoint = false;
    }
  }

  inputFile.close();

  return way_point_traj;
}

void transformWaypoints(std::vector<std::vector<double>> &waypoints,
                        float scale_factor, const Eigen::Vector3f &translation,
                        const Eigen::Matrix3f &rotation) {
  ROS_INFO("Transforming waypoints.");

  for (auto &waypoint : waypoints) {
    // Convert waypoint to Eigen::Vector3f for transformation (assuming waypoint
    // = {x, y, z, yaw})
    Eigen::Vector3f p(waypoint[0] * scale_factor, waypoint[1] * scale_factor,
                      waypoint[2] * scale_factor);

    // Apply rotation
    p = rotation * p;

    // Apply translation
    waypoint[0] = p.x() + translation.x(); // Transformed X
    waypoint[1] = p.y() + translation.y(); // Transformed Y
    waypoint[2] = p.z() + translation.z(); // Transformed Z
  }
}

void printTrajectory(const std::vector<std::vector<double>> &trajectory) {
  for (const auto &wp : trajectory) {
    std::cout << "X: " << wp[0] << ", Y: " << wp[1] << ", Z: " << wp[2]
              << ", YAW: " << wp[3] << std::endl;
  }
}



void densifyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  // Check if the input cloud is empty
  if (cloud->empty()) {
    PCL_ERROR("Input cloud is empty!\n");
    return;
  }

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  // Output point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Initialize object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setComputeNormals(false);

  // Set parameters
  mls.setInputCloud(cloud);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03); // Adjust the search radius as needed

  // Reconstruct
  mls.process(*mls_points);

  // Check if the output cloud is empty
  if (mls_points->empty()) {
    PCL_ERROR("Densified cloud is empty!\n");
    return;
  }

  // Update the input cloud with the densified points
  cloud = mls_points;
}

void initializeVoxelGridAndKdTree(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  // ROS_INFO("Initializing voxel grid and k-d tree.");

  // Downsample the point cloud using a voxel grid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.01f, 0.01f,
                         0.01f); // Adjust the leaf size for higher resolution
  voxel_grid.filter(*filtered_cloud);

  // ROS_INFO("Voxel grid filter applied. Original points: %zu, Filtered points:
  // %zu", cloud->points.size(), filtered_cloud->points.size());

  // Initialize the k-d tree with the downsampled point cloud
  kdtree.setInputCloud(filtered_cloud);
  voxel_grid_initialized = true;

  // ROS_INFO("K-d tree initialized with filtered point cloud.");
}

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         float scale_factor, const Eigen::Vector3f &translation,
                         const Eigen::Matrix3f &rotation) {
  ROS_INFO("Transforming point cloud.");
  for (auto &point : cloud->points) {
    // Apply scaling
    Eigen::Vector3f p(point.x * scale_factor, point.y * scale_factor,
                      point.z * scale_factor);

    // Apply rotation
    p = rotation * p;

    // Apply translation
    point.x = p.x() + translation.x();
    point.y = p.y() + translation.y();
    point.z = p.z() + translation.z();
  }
}

// Data Collection Functions
void initializeTrajectoryFilename() {
  // Directory to save the files
  std::string directory =
      "/home/hakim/tether_planning_ws/src/rope_rrt/results/";

  // Ensure the directory exists
  std::filesystem::create_directories(directory);

  // Find the highest numbered file in the directory
  int max_number = 0;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    std::string filename = entry.path().filename().string();
    if (filename.find("trajectory_results_") == 0 &&
        filename.find(".txt") != std::string::npos) {
      int number = std::stoi(filename.substr(18, filename.size() - 22));
      if (number > max_number) {
        max_number = number;
      }
    }
  }

  // Generate a new filename with an incremented number
  std::ostringstream filename;
  filename << directory << "trajectory_results_" << (max_number + 1) << ".txt";
  trajectory_filename = filename.str();
}

void saveTrajectoryData(const ros::Time &ros_time,
                        const std::vector<double> &rov_pos,
                        const std::vector<double> &angles,
                        const ompl::geometric::PathGeometric &tether,
                        bool record_trajectory) {
  if (!record_trajectory) {
    return;
  }

  // Open the file for writing
  std::ofstream file(trajectory_filename);
  if (!file.is_open()) {
    ROS_ERROR("Failed to open file: %s", trajectory_filename.c_str());
    return;
  }

  // Write the header
  file << "ros_time, rov_pos_x, rov_pos_y, rov_pos_z, yaw, pitch, roll, "
          "tether_x, tether_y, tether_z\n";

  // Write the data row by row
  for (std::size_t i = 0; i < tether.getStateCount(); ++i) {
    const auto *state =
        tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    file << std::fixed << std::setprecision(6) << ros_time.toSec() << ", "
         << rov_pos[0] << ", " << rov_pos[1] << ", " << rov_pos[2] << ", "
         << angles[0] << ", " << angles[1] << ", " << angles[2] << ", "
         << state->values[0] << ", " << state->values[1] << ", "
         << state->values[2] << "\n";
  }

  // Close the file
  file.close();
  ROS_INFO("Trajectory data saved to file: %s", trajectory_filename.c_str());
}

std::vector<double> crossProduct(const std::vector<double> &v1,
                                 const std::vector<double> &v2) {
  std::vector<double> cross(3);
  cross[0] = v1[1] * v2[2] - v1[2] * v2[1];
  cross[1] = v1[2] * v2[0] - v1[0] * v2[2];
  cross[2] = v1[0] * v2[1] - v1[1] * v2[0];
  return cross;
}

// Function to normalize a vector
std::vector<double> normalize(const std::vector<double> &v) {
  double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  std::vector<double> unit_vector(3);
  unit_vector[0] = v[0] / norm;
  unit_vector[1] = v[1] / norm;
  unit_vector[2] = v[2] / norm;
  return unit_vector;
}

std::vector<std::vector<double>>
sampleAtDistance(const std::vector<double> &start,
                 const std::vector<double> &direction, double delta) {
  std::vector<double> posSample = {start[0] + delta * direction[0],
                                   start[1] + delta * direction[1],
                                   start[2] + delta * direction[2]};

  std::vector<double> negSample = {start[0] - delta * direction[0],
                                   start[1] - delta * direction[1],
                                   start[2] - delta * direction[2]};

  return {posSample,
          negSample}; // Return both the positive and negative samples
}

// Function to compute the desired vector
// Function to compute the perpendicular unit vector using v1 and v2

void saveTrajectory(const ros::Time &ros_time,
                    const std::vector<double> &rov_pos) {
  // Ensure the results directory exists
  std::filesystem::create_directories("results");

  // Open the file in append mode
  std::ofstream file("results/rov_position.txt", std::ios::app);

  if (file.is_open()) {
    // Write the time and position data to the file
    file << std::fixed << std::setprecision(6) << ros_time.toSec() << ", "
         << rov_pos[0] << ", " << rov_pos[1] << ", " << rov_pos[2] << std::endl;

    file.close();
  } else {
    ROS_ERROR("Unable to open file for writing ROV position data.");
  }
}

void saveTetherPathData(const ros::Time &ros_time,
                        const ompl::geometric::PathGeometric &tether) {
  // Ensure the results directory exists
  std::filesystem::create_directories("results");

  // Open the file in append mode
  std::ofstream file("results/tether_path.txt", std::ios::app);

  if (file.is_open()) {
    // Write the time to the file
    file << std::fixed << std::setprecision(6) << ros_time.toSec() << std::endl;

    // Write the tether path data to the file
    for (std::size_t i = 0; i < tether.getStateCount(); ++i) {
      const auto *state =
          tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      file << std::fixed << std::setprecision(6) << state->values[0] << ", "
           << state->values[1] << ", " << state->values[2] << std::endl;
    }

    file.close();
  } else {
    ROS_ERROR("Unable to open file for writing tether path data.");
  }
}


// Function to read ASCII STL file
std::vector<Triangle> readSTLFile(const std::string &filename) {
  std::vector<Triangle> triangles;
  std::ifstream file(filename);
  
  if (!file.is_open()) {
      std::cerr << "Unable to open file: " << filename << std::endl;
      return triangles;
  }
  
  std::string line;
  Triangle currentTriangle;
  int vertexCount = 0;
  bool insideFacet = false;
  
  while (std::getline(file, line)) {
      // Trim leading and trailing whitespace
      line.erase(0, line.find_first_not_of(" \t"));
      line.erase(line.find_last_not_of(" \t") + 1);
      
      if (line.empty()) continue;
      
      if (line.find("facet normal") != std::string::npos) {
          insideFacet = true;
          vertexCount = 0;
          
          // Parse normal
          float nx, ny, nz;
          sscanf(line.c_str(), "facet normal %f %f %f", &nx, &ny, &nz);
          currentTriangle.normal = Eigen::Vector3f(nx, ny, nz);
      } 
      else if (line.find("vertex") != std::string::npos) {
          float vx, vy, vz;
          sscanf(line.c_str(), "vertex %f %f %f", &vx, &vy, &vz);
          
          if (vertexCount == 0) {
              currentTriangle.vertex1 = Eigen::Vector3f(vx, vy, vz);
          } else if (vertexCount == 1) {
              currentTriangle.vertex2 = Eigen::Vector3f(vx, vy, vz);
          } else if (vertexCount == 2) {
              currentTriangle.vertex3 = Eigen::Vector3f(vx, vy, vz);
          }
          
          vertexCount++;
      } 
      else if (line.find("endfacet") != std::string::npos) {
          if (insideFacet && vertexCount == 3) {
              triangles.push_back(currentTriangle);
          }
          insideFacet = false;
      }
  }
  
  file.close();
  return triangles;
}

void transformEIVAPC(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation) {
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << translation;
  transform.rotate(rotation);

  pcl::transformPointCloud(*cloud, *cloud, transform);
}


void loadXYZFileToPCL(const std::string &filename) {
  eiva_pcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    float x, y, z;
    if (!(iss >> x >> y >> z)) {
      std::cerr << "Error reading line: " << line << std::endl;
      continue;
      
    }
    eiva_pcloud->points.emplace_back(x, y, z);
    // Print the loaded point
    //std::cout << "Loaded point: " << x << ", " << y << ", " << z << std::endl;
  }

  file.close();
  eiva_pcloud->width = eiva_pcloud->points.size();
  eiva_pcloud->height = 1;
  eiva_pcloud->is_dense = true;
}

void loadParameters(ros::NodeHandle &nh) {
  nh.param("tether/L_max", L_max, 10.0);
  nh.param("tether_color/r", tetherColor.r, 1.0f);
  nh.param("tether_color/g", tetherColor.g, 0.0f);
  nh.param("tether_color/b", tetherColor.b, 0.0f);
  nh.param("tether_color/a", tetherColor.a, 1.0f);
  nh.param("direct_path_color/r", directPath.r, 1.0f);
  nh.param("direct_path_color/g", directPath.g, 0.0f);
  nh.param("direct_path_color/b", directPath.b, 0.0f);
  nh.param("direct_path_color/a", directPath.a, 1.0f);
  nh.param("safe_path_color/r", safePath.r, 0.0f);
  nh.param("safe_path_color/g", safePath.g, 0.0f);
  nh.param("safe_path_color/b", safePath.b, 1.0f);
  nh.param("safe_path_color/a", safePath.a, 1.0f);
  nh.param("ropepath_color/r", ropepathColor.r, 0.6f);
  nh.param("ropepath_color/g", ropepathColor.g, 0.6f);
  nh.param("ropepath_color/b", ropepathColor.b, 0.0f);
  nh.param("ropepath_color/a", ropepathColor.a, 1.0f);
  nh.param("rovpath_color/r", rovpathColor.r, 0.8f);
  nh.param("rovpath_color/g", rovpathColor.g, 0.8f);
  nh.param("rovpath_color/b", rovpathColor.b, 0.0f);
  nh.param("rovpath_color/a", rovpathColor.a, 0.5f);
  nh.param("safepath_color/r", safepathColor.r, 0.8f);
  nh.param("safepath_color/g", safepathColor.g, 0.8f);
  nh.param("safepath_color/b", safepathColor.b, 0.0f);
  nh.param("safepath_color/a", safepathColor.a, 1.0f);

  nh.param("simulation/use_nvblox", USE_NVBLOX_COLLISION_CHECKER, true);
  nh.param("simulation/time_step", time_step, 1.0);
  nh.param("simulation/ta_planner_on", TA_Planner_ON, true);
  nh.param<std::string>("simulation/model", inspection_model,
                        std::string("/eiva_logo"));
  nh.param("simulation/model_scale", model_scale, 1.0);

  nh.param("tether/delta", delta, 0.2);
  nh.param("tether/eq_tolerance", equivalenceTolerance, 0.000001);
  nh.param("tether/safe_offset", safe_offset, 0.3);
  nh.param("tether/collision_threshold", collision_threshold, -0.1);

  ROS_INFO("L_max: %f", L_max);
  ROS_INFO("tether_color: [%f, %f, %f, %f]", tetherColor.r, tetherColor.g,
           tetherColor.b, tetherColor.a);
  ROS_INFO("direct_path_color: [%f, %f, %f, %f]", directPath.r, directPath.g,
           directPath.b, directPath.a);
  ROS_INFO("safe_path_color: [%f, %f, %f, %f]", safePath.r, safePath.g,
           safePath.b, safePath.a);
  ROS_INFO("ropepath_color: [%f, %f, %f, %f]", ropepathColor.r, ropepathColor.g,
           ropepathColor.b, ropepathColor.a);
  ROS_INFO("rovpath_color: [%f, %f, %f, %f]", rovpathColor.r, rovpathColor.g,
           rovpathColor.b, rovpathColor.a);
  ROS_INFO("safepath_color: [%f, %f, %f, %f]", safepathColor.r, safepathColor.g,
           safepathColor.b, safepathColor.a);
  ROS_INFO("time_step: %f", time_step);
  ROS_INFO("TA_Planner_ON: %d", TA_Planner_ON);
  ROS_INFO("delta: %f", delta);
  ROS_INFO("equivalenceTolerance: %f", equivalenceTolerance);
  ROS_INFO("safe_offset: %f", safe_offset);
}

void initializeObstacles() {
  way_point = {0.0, 0.0, 4.0};

  cylinder_obs cylinder2;
  cylinder2.baseCenter = Eigen::Vector3f(-2.0, 1.0, -1.5);
  cylinder2.axis = Eigen::Vector3f(1.0, -0.0, 0.01);
  cylinder2.radius = 0.4;
  cylinder2.height = 5.0;
  cylinders.push_back(cylinder2);

  cylinder_obs cylinder1;
  cylinder1.baseCenter = Eigen::Vector3f(1.2, 1.0, -1.0);
  cylinder1.axis = Eigen::Vector3f(0.0, 0.0, 1.0);
  cylinder1.radius = 0.4;
  cylinder1.height = 5.0;
  cylinders.push_back(cylinder1);

  cylinder_obs cylinder2_safe = cylinder2;
  cylinder2_safe.radius += 0.5;
  cylinders_safe.push_back(cylinder2_safe);

  cylinder_obs cylinder1_safe = cylinder1;
  cylinder1_safe.radius += 0.5;
  cylinders_safe.push_back(cylinder1_safe);
}

void removeWaypoint(std::vector<std::vector<double>> &way_point_traj, int index) {
  if (index >= 0 && index < way_point_traj.size()) {
      way_point_traj.erase(way_point_traj.begin() + index);
      std::cout << "Removed waypoint at index " << index << std::endl;
  } else {
      std::cerr << "Index out of range. Cannot remove waypoint." << std::endl;
  }
}


void initializePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    float scale_factor = 1.0;
  



    std::string filename = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
      PCL_ERROR("Couldn't read file %s \n", filename.c_str());
      exit(-1);
    }
  
    //Eigen::Vector3f translation_local(2.0, 1.0, 7.5);


    //Eigen::Matrix3f rotation_local = Eigen::DiagonalMatrix<float, 3>(-1, -1, -1);

    //transformPointCloud(cloud, scale_factor, translation_local, rotation_local);
  
  
  
    initializeVoxelGridAndKdTree(cloud);
  }

void initializeOMPLSpace(std::shared_ptr<ompl::base::SpaceInformation> &si,
                         std::shared_ptr<ompl::base::SpaceInformation> &si_t) {
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(-30);
  bounds.setHigh(30);
  space->setBounds(bounds);

  si = std::make_shared<ompl::base::SpaceInformation>(space);
  si->setStateValidityChecker(isStateValid);
  si->setMotionValidator(std::make_shared<CustomMotionValidator>(si));
  si->setup();

  si_t = std::make_shared<ompl::base::SpaceInformation>(space);
  si_t->setStateValidityChecker(isStateValid);
  si_t->setMotionValidator(std::make_shared<CustomMotionValidator>(si_t));
  si_t->setup();


  

  P_t = ompl::geometric::PathGeometric(si_t);
  //P_t.append(state_base);

  //P_sample = ompl::geometric::PathGeometric(si_t);
  //Path_sample.append(state_base);

}

std::unique_ptr<nvblox::Mapper> initializeNVBlox() {
  std::string nvblx_file_path =
      "/home/hakim/tether_planning_ws/src/tether_planner/models/dfki_pipe.nvblx";
  std::unique_ptr<nvblox::Mapper> mapper = mapFromCake(nvblx_file_path);

  if (!mapper) {
    std::cerr << "Failed to load mapper from " << nvblx_file_path << std::endl;
    return nullptr;
  }

  const nvblox::TsdfLayer &tsdf_layer = mapper->tsdf_layer();
  const nvblox::MeshLayer &mesh_layer = mapper->mesh_layer();

  std::cout << "TSDF Layer Size: " << tsdf_layer.size() << std::endl;
  std::cout << "Mesh Layer Size: " << mesh_layer.size() << std::endl;

  return mapper;
}

void loadAndTransformWaypoints(
    const std::string &filePath, double distanceThreshold, float scale_factor,
    const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation,
    std::vector<std::vector<double>> &way_point_traj) {
  // Load waypoints from file
  std::ifstream file(filePath);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filePath << std::endl;
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::vector<double> waypoint(3);
    if (!(iss >> waypoint[0] >> waypoint[1] >> waypoint[2])) {
      break;
    }
    way_point_traj.push_back(waypoint);
  }
  file.close();

  // Sparsify waypoints
  way_point_traj = sparsifyTrajectory(filePath, distanceThreshold);
  transformWaypoints(way_point_traj, scale_factor, translation, rotation);
   
  //Eigen::Vector3f translation_local(2.0, 1.0, 7.5);
  //double scale_local =0.1;
  //Eigen::Matrix3f rotation_local = Eigen::DiagonalMatrix<float, 3>(1, 1, 1);
  //Eigen::Matrix3f rotation_local = Eigen::DiagonalMatrix<float, 3>(-1, -1, -1);

  //transformWaypoints(way_point_traj, scale_local, translation_local, rotation_local);
  //translation_local << 4.0, 2.0, -2.0;
       
       
 // translation_local <<2.0, 8.5, 3.0; // Move closer to the origin

 // rotation_local = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX());

 // transformWaypoints(way_point_traj, 1.0, translation_local, rotation_local);

 
 
 // translation_local <<0.0, -1.5, -2.5; // Move closer to the origin
 // rotation_local = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());

  //transformWaypoints(way_point_traj, 1.2, translation_local, rotation_local);


}




// Function to set the file path for the text file
void setTxtFilePath(const std::string &current_directory, const std::string &model_name, std::string &file_path)
{
    file_path = current_directory + "/models/" + model_name + ".txt";
}

// Function to set the file path for the NVBlox model
void setNVBloxFilePath(const std::string &current_directory, const std::string &model_name, std::string &file_path)
{
    file_path = current_directory + "/models/" + model_name + ".nvblx";
}

// Function to set the file path for the XYZ file
void setXYZFilePath(const std::string &current_directory, const std::string &model_name, std::string &file_path)
{
    file_path = current_directory + "/models/" + model_name + ".xyz";
}



void initializeROS(int argc, char **argv, ros::NodeHandle &nh)
{
    ros::init(argc, argv, "tether_planner");
    ros::Time::init();
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    ROS_INFO("Initialized Node");
}


void initializeSubscribers(ros::NodeHandle &nh)
{
    ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);
    ros::Subscriber reset_tether_sub = nh.subscribe("reset_tether_topic", 10, reset_tether_cb);
    ros::Subscriber record_trajectory_sub = nh.subscribe("record_trajectory_topic", 10, record_trajectory_on_cb);
    ros::Subscriber goal_point_sub = nh.subscribe<geometry_msgs::PointStamped>("goal_point_pub", 10, goal_cb);
}


// Function to initialize publishers
void initializePublishers(ros::NodeHandle &nh)
{
    rov_path_pub = nh.advertise<visualization_msgs::Marker>("rov_path", 10);
    rope_path_pub = nh.advertise<visualization_msgs::Marker>("rope_path", 10);
    obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 10);
    tether_path_pub = nh.advertise<nav_msgs::Path>("tether_path", 10);
    planner_path_pub = nh.advertise<nav_msgs::Path>("planner_path", 10);
    safe_planner_path_pub = nh.advertise<nav_msgs::Path>("safe_planner_path", 10);
    direct_path_pub = nh.advertise<visualization_msgs::Marker>("direct_optimal_path", 10);
    safe_path_pub = nh.advertise<visualization_msgs::Marker>("safe_path", 10);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
    voxel_grid_pub = nh.advertise<sensor_msgs::PointCloud2>("voxel_grid", 1);
    blue_rov_pub = nh.advertise<visualization_msgs::Marker>("blue_rov", 10);
    trajectory_pub = nh.advertise<visualization_msgs::Marker>("inspection_reference_trajectory", 10);
    ref_pub = nh.advertise<geometry_msgs::PoseStamped>("/ropeplanner_goal", 10);
    point_pub = nh.advertise<geometry_msgs::PoseStamped>("/viz_point", 10);
    cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinders", 10);
    exit_points_pub = nh.advertise<visualization_msgs::MarkerArray>("exit_points", 1);
    bounding_box_pub = nh.advertise<visualization_msgs::Marker>("bounding_box", 1);
    stl_model_pub = nh.advertise<visualization_msgs::MarkerArray>("stl_model_marker", 1);
    vector_pub = nh.advertise<visualization_msgs::Marker>("vector_marker", 10);
  }


void initializeGlobalVariables() {
    current_att_quat = std::vector<double>(4, 0.0);
    current_vel_rate = std::vector<double>(3, 0.0);
    current_pos_att = std::vector<double>(3, 0.0);
    new_data_received = false;
    current_vel_body = std::vector<double>(3, 0.0);
    angles = std::vector<double>(3, 0.0);
    angles_d = std::vector<double>(3, 0.0);
    goal = std::vector<double>(3, 0.0);
    goal_t1 = std::vector<double>(3, 0.0);
    base = std::vector<double>(3, 0.0);
    contactPoints.reserve(2000);
    contactPoints.clear();
    scale_factor = 0.1;
    translation = Eigen::Vector3f(2.0, 1.0, -4.0);
    rotation = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
    count = 0;
    double distanceThreshold = 10;
    //txtfilePath = "/home/hakim/tether_planning_ws/src/input_data/dfki_pipe_fc.txt";
    txtfilePath = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe_fc.txt";

    //nvblx_file_path = "/home/hakim/tether_planning_ws/src/tether_planner/models/pipe.nvblx";
    nvblx_file_path = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.nvblx";

    input_xyz_file = "/home/hakim/tether_planning_ws/src/tether_planner/models/EivaLogo.xyz";
    stl_file = "/home/hakim/tether_planning_ws/src/tether_planner/models/pipe_simple.stl";


    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
    state_base = space->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
    state_base->values[0] = 0.0;
    state_base->values[1] = 0.0;
    state_base->values[2] = 2.0;
    
    inspection_model_stl = readSTLFile(stl_file);
    way_point = {0.0, 0.0, 0.0};  // Proper way to assign values
    //printTriangles(inspection_model_stl);
   //    transformAndScaleTriangles(inspection_model_stl);
   //transformAndScaleTriangles(inspection_model_stl, 0.1, Eigen::Vector3f(0.0, 0.0, 2.0));
    transformAndScaleTriangles(inspection_model_stl, 0.05, Eigen::Vector3f(0.0, 0.0, 2.0));

   initializeSTLMesh();
    //max_point = Eigen::Vector3f(0.728, 0.784, 0.39);
    //min_point = Eigen::Vector3f(-3.472, -0.168, -0.112);

}



void loadNvbloxModel(const std::string& file_path) {
  // Load the nvblox map from the file
  mapper = mapFromCake(file_path);
  std::cout << "NVBlox file path: " << file_path << std::endl;
  // Check if the mapper was loaded successfully
  if (!mapper) {
      std::cerr << "Failed to load mapper from " << file_path << std::endl;
      return;
  }

  // Access the TSDF layer and mesh layer for further processing or visualization
  tsdf_layer = &mapper->tsdf_layer();
  mesh_layer = &mapper->mesh_layer();
}











bool CollisionCheckSTL(const std::vector<Triangle> &triangles, const fcl::CollisionObjectf &object) {
  // Create a vector of FCL triangles
  std::vector<fcl::Triangle> fcl_triangles;
  std::vector<fcl::Vector3f> vertices;

  for (const auto &triangle : triangles) {
      fcl::Vector3f v1(triangle.vertex1.x(), triangle.vertex1.y(), triangle.vertex1.z());
      fcl::Vector3f v2(triangle.vertex2.x(), triangle.vertex2.y(), triangle.vertex2.z());
      fcl::Vector3f v3(triangle.vertex3.x(), triangle.vertex3.y(), triangle.vertex3.z());

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);

      fcl_triangles.emplace_back(vertices.size() - 3, vertices.size() - 2, vertices.size() - 1);
  }

  // Create FCL collision geometry
  auto stl_mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSf>>();
  stl_mesh->beginModel();
  stl_mesh->addSubModel(vertices, fcl_triangles);
  stl_mesh->endModel();

  // Create FCL collision object for the STL model
  fcl::CollisionObjectf stl_object(stl_mesh);

  // Perform collision checking
  fcl::CollisionRequestf request;
  fcl::CollisionResultf result;
  fcl::collide(&stl_object, &object, request, result);

  return result.isCollision();
}


std::vector<ompl::base::State*> generateHelicalPath(std::shared_ptr<ompl::base::SpaceInformation> si, double radius, double height, int num_points) {
  std::vector<ompl::base::State*> path;
  double angle_step = 2 * M_PI / num_points;
  double height_step = height / num_points;

  for (int i = 0; i < num_points; ++i) {
      double angle = i * angle_step;
      double x = i * height_step;
      double y = radius * cos(angle);
      double z = radius * sin(angle);

      ompl::base::State *state = si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = y;
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = z;

      path.push_back(state);
  }

  return path;
}


void sparsifyAndSavePointCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> const& cloud, float voxel_size, const std::string& output_path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Apply voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);  // Set voxel size
  voxel_filter.filter(*filtered_cloud);  // Apply filter

  // Save the filtered point cloud to a file
  pcl::io::savePCDFileBinary(output_path, *filtered_cloud);  // Save as PCD
}


void convertPCDToXYZ(const std::string &input_pcd_file, const std::string &output_xyz_file) {
  // Load the PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
      PCL_ERROR("Couldn't read file %s \n", input_pcd_file.c_str());
      return;
  }

  // Open the output XYZ file
  std::ofstream ofs(output_xyz_file);
  if (!ofs.is_open()) {
      std::cerr << "Unable to open file: " << output_xyz_file << std::endl;
      return;
  }

  // Write the points to the XYZ file
  for (const auto &point : cloud->points) {
      ofs << point.x << " " << point.y << " " << point.z << std::endl;
  }

  ofs.close();
  std::cout << "Saved " << cloud->points.size() << " data points to " << output_xyz_file << std::endl;
}


void printPath(const ompl::geometric::PathGeometric &path) {
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      std::cout << "State " << i << ": [" << state->values[0] << ", " << state->values[1] << ", " << state->values[2] << "]" << std::endl;
  }
}



// Function to calculate the distance from a point to a triangle
float pointToTriangleDistance(const Eigen::Vector3f &point, const Triangle &triangle) {
  // Calculate the vectors from the triangle vertices to the point
  Eigen::Vector3f v0 = triangle.vertex1;
  Eigen::Vector3f v1 = triangle.vertex2;
  Eigen::Vector3f v2 = triangle.vertex3;
  Eigen::Vector3f p = point;

  // Calculate the normal of the triangle
  Eigen::Vector3f normal = (v1 - v0).cross(v2 - v0).normalized();

  // Calculate the distance from the point to the plane of the triangle
  float distance = std::abs((p - v0).dot(normal));

  return distance;
}

// Function to find the closest normal vector of the STL model to the point
Eigen::Vector3f findClosestNormal(const Eigen::Vector3f &point, const std::vector<Triangle> &triangles) {
  float min_distance = std::numeric_limits<float>::max();
  Eigen::Vector3f closest_normal;

  for (const auto &triangle : triangles) {
      float distance = pointToTriangleDistance(point, triangle);
      if (distance < min_distance) {
          min_distance = distance;
          closest_normal = triangle.normal;
      }
  }

  return closest_normal;
}








void transformSTLModel(std::vector<Triangle> &triangles, float scale_factor, const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation) {
  for (auto &triangle : triangles) {
      // Scale vertices
      triangle.vertex1 *= scale_factor;
      triangle.vertex2 *= scale_factor;
      triangle.vertex3 *= scale_factor;

      // Apply rotation
      triangle.vertex1 = rotation * triangle.vertex1;
      triangle.vertex2 = rotation * triangle.vertex2;
      triangle.vertex3 = rotation * triangle.vertex3;

      // Apply translation
      triangle.vertex1 += translation;
      triangle.vertex2 += translation;
      triangle.vertex3 += translation;

      // Transform the normal
      triangle.normal = rotation * triangle.normal;
  }
}



Eigen::Vector3f findClosestTriangleCenter(const Eigen::Vector3f &point, const std::vector<Triangle> &triangles) {
  float min_distance = std::numeric_limits<float>::max();
  Eigen::Vector3f closest_center;

  for (const auto &triangle : triangles) {
      float distance = pointToTriangleCenterDistance(point, triangle);
      if (distance < min_distance) {
          min_distance = distance;
          closest_center = (triangle.vertex1 + triangle.vertex2 + triangle.vertex3) / 3.0f;
      }
  }

  return closest_center;
}


float pointToTriangleCenterDistance(const Eigen::Vector3f &point, const Triangle &triangle) {
  // Calculate the center of the triangle
  Eigen::Vector3f center = (triangle.vertex1 + triangle.vertex2 + triangle.vertex3) / 3.0f;

  // Calculate the distance from the point to the center of the triangle
  float distance = (point - center).norm();

  return distance;
}



double calculateDistance(const std::vector<double> &point1, const std::vector<double> &point2) {
  return std::sqrt(std::pow(point1[0] - point2[0], 2) +
                   std::pow(point1[1] - point2[1], 2) +
                   std::pow(point1[2] - point2[2], 2));
}
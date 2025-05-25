/**
 * @file   nmpc_bluerov2_2-D_main.cpp 
 * entagelement aware MPC, current implementation works in conjuction with NEPTUNE planner.
 * @author Hakim Amer
 * @date   Nov 2024
 *
 */

#include <nmpc_bluerov2_2D_main.h>


// Helper Functions
// ===========================


//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////



// function to integration the entaglement 
void entanglement_integration_function() {
    static size_t current_index = 0; // Track the current position in the fixed-size vector

    double err_x = (ref_position[0] - current_pos_att[0]) * (ref_position[0] - current_pos_att[0]) * online_data.ent_point[0];
    double err_y = (ref_position[1] - current_pos_att[1]) * (ref_position[1] - current_pos_att[1]) * online_data.ent_point[1];
    double err_t = abs(err_x) + abs(err_y);

    // Update the error sum: subtract the old value at the current index, then add the new value
    err_sum_ -= err_vector_[current_index];
    err_vector_[current_index] = err_t;
    err_sum_ += err_t;

    // Move to the next index, wrapping around to 0 when reaching the end
    current_index = (current_index + 1) % integ_error_h_;

    // Ceiling on error summation
     
     
     if (std::isnan(err_sum_)) {
    std::cerr << "\033[31mError: err_sum_ is NaN!\033[0m" << std::endl; // Print in red
    err_sum_ = 0.0; // Assign zero to err_sum_
} else   if (err_sum_ < 0.0) {
        err_sum_ = 0.0;
    } else {
        err_sum_ = std::min(err_sum_, error_cap_);  // Apply upper limit of 100
    }


    

    for (size_t i = 0; i < err_vector_.size(); ++i) {
    std::cout << "err_vector_[" << i << "] = " << err_vector_[i] << std::endl;
}
    std::cout << "\033[31mAccumulated error is: " << err_sum_ << "\033[0m" << std::endl;
        std::cout << "\033[index: " << current_index << "\033[0m" << std::endl;

}


void entagelement_rotation_direction(){

    int rd_x = (ent_point_[0] -current_pos_att[0] ) ;
    int rd_y = (ent_point_[1] -current_pos_att[1] ) ;

    int ox = ( online_data.obs_centre.at(0) - current_pos_att[0] ) ;
    int oy = (online_data.obs_centre.at(1) - current_pos_att[1] ) ;
    // Calculate the 2D cross product
    int cross_product = rd_x * oy - rd_y * ox;

    // Determine rotation direction
    
    if (cross_product > 0) {
        rotate_dir_ = 1.0; // Counterclockwise
    } else if (cross_product < 0) {
        rotate_dir_ = -1.0; // Clockwise
    } else {
        rotate_dir_ = 0.0; // Collinear (no rotation)
    }

    rotate_dir_ = rotate_dir_;
    std::cout << "\033[32mRotation direction: " << rotate_dir_ << "\033[0m" << std::endl;

}


void initialize_reference_if_Empty()
{
    // Check if current_ref_x is NaN and assign the first element of ref_path_x if it is
    if (std::isnan(current_ref_x) && !ref_traj_x.empty())
    {
        current_ref_x = ref_traj_x[0];
    }

    // Check if current_ref_y is NaN and assign the first element of ref_path_y if it is
    if (std::isnan(current_ref_y) && !ref_traj_y.empty())
    {
        current_ref_y = ref_traj_y[0];
    }

    // Check if current_ref_z is NaN and assign the first element of ref_path_z if it is
    if (std::isnan(current_ref_z) && !ref_traj_z.empty())
    {
        current_ref_z = ref_traj_z[0];
    }

    std::cout << "current_ref_x: " << current_ref_x << "\n";
    std::cout << "current_ref_y: " << current_ref_y << "\n";
    std::cout << "current_ref_z: " << current_ref_z << "\n";
}


// Function to find the closest obstacle to the ROV position
std::vector<double> findClosestObstacle(const geometry_msgs::Point *conct_pos, std::vector<double> rov_position)
{
    double min_distance = std::numeric_limits<double>::max();
    std::vector<double> closest_obstacle_center;
    for (const auto &center : obstacle_centers)
    {
        double distance;
        if (conct_pos != nullptr)
        {
            distance = std::sqrt(
                std::pow(center[0] - conct_pos->x, 2) +
                std::pow(center[1] - conct_pos->y, 2) +
                std::pow(center[2] - conct_pos->z, 2));
        }
        else
        {
            // conct_pos.x is empty
            // Handle the empty case here if needed
            distance = std::sqrt(
                std::pow(center[0] - rov_position[0], 2) +
                std::pow(center[1] - rov_position[1], 2) +
                std::pow(center[2] - rov_position[2], 2));
        }

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_obstacle_center = center;
        }
    }

    ROS_INFO("Closest obstacle at (%f, %f, %f) with distance %f",
             closest_obstacle_center[0], closest_obstacle_center[1],
             closest_obstacle_center[2], min_distance);

    return closest_obstacle_center;
}


bool arePointsEqual(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return (std::fabs(p1.x - p2.x) < TOLERANCE && std::fabs(p1.y - p2.y) < TOLERANCE);
}

geometry_msgs::Point *getFirstContactPoint(const std::vector<geometry_msgs::Point> &tether)
{
    std::unordered_map<std::string, int> pointCount;

    for (const auto &point : tether)
    {
        std::string key = std::to_string(point.x) + "_" + std::to_string(point.y);
        // std::cout << "Processing point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
        // std::cout << "Generated key: " << key << std::endl;

        // Check if this point has appeared before based on its (x, y) values
        pointCount[key]++;

        if (pointCount[key] == 2)
        {
            // Print the contact point
           // std::cout << "First contact point found: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            return new geometry_msgs::Point(point); // Dynamically allocate to return a pointer
        }
    }

    return nullptr; // Return nullptr if no contact point is found
}



std::vector<double> compute_entanglement_point(const std::vector<double> &R,
                                                         const std::vector<double> &C,
                                                         const std::vector<double> &B
                                                         )
    {
       // Calculate the vector BC and RC
    std::vector<double> BC = {C[0] - B[0], C[1] - B[1]};
    std::vector<double> RC = {C[0] - R[0], C[1] - R[1]};

    // Calculate the magnitudes (lengths) of BC and RC
    double l1 = std::sqrt(BC[0] * BC[0] + BC[1] * BC[1]);
    double l2 = std::sqrt(RC[0] * RC[0] + RC[1] * RC[1]);

    // Calculate the cosine of the angle
    double dot_product = (BC[0] * RC[0]) + (BC[1] * RC[1]);

    double cos_theta = dot_product / (l1 * l2);


    double D = abs(l2 * cos_theta);

    double L = D + l1;
    double scale = L/l1;

    std::vector<double> BD = {scale * BC[0], scale * BC[1]};
 
    std::vector<double> e =   {BD[0] + B[0], BD[1] + B[1]};

    return e;
    }





void updateEntanglementData(
    const std::vector<geometry_msgs::Point> &tether_positions,
    const std::vector<std::vector<double>> &obstacle_centers,
    const std::vector<double> &rov_position,
    const geometry_msgs::Point &neptune_goal,
    online_data_struct_ &online_data)
{
    // Get the first contact point
    geometry_msgs::Point *contactPoint = getFirstContactPoint(tether_positions);
    ROS_INFO("Contact Point: x=%f, y=%f, z=%f", contactPoint ? contactPoint->x : 0.0,
                                                      contactPoint ? contactPoint->y : 0.0,
                                                      contactPoint ? contactPoint->z : 0.0);

    // Initialize variables
    std::vector<double> closest_obs(3, 0.0);
    std::vector<double> ent_point(3, 0.0); // Initialize entanglement point

    // Check if there are any obstacles
    if (!obstacle_centers.empty())
    {
        closest_obs = findClosestObstacle(contactPoint, rov_position);
        //ROS_INFO("Closest obstacle: x=%f, y=%f", closest_obs[0], closest_obs[1]);
    }
    else
    {
        ROS_WARN("No obstacles found.");
    }

    // Determine if contact was made
    int CONTACT = (contactPoint != nullptr) ? 1 : 0;
    ROS_INFO("Contact status: %d", CONTACT);

    // If contact was made
    if (CONTACT == 1)
    {
        if (contactPoint != nullptr)
        {
            // Compute the entanglement point
            std::vector<double> contact_vector = {contactPoint->x, contactPoint->y, contactPoint->z};
            ent_point = compute_entanglement_point(rov_position, contact_vector, base_pos);
            //ROS_INFO("Entanglement point (from contact): x=%f, y=%f, z=%f", ent_point[0], ent_point[1], ent_point[2]);
        }
    }
    else
    {
        // No contact, set entanglement point to goal
        //ent_point[0] = neptune_goal.x;
        //ent_point[1] = neptune_goal.y;
        ent_point[0] = current_pos_att[0];
        ent_point[1] = current_pos_att[1];

        ent_point[2] = 0.0; // Z-coordinate is set to 0.0
        
        //ROS_INFO("Entanglement point (from goal): x=%f, y=%f, z=%f", ent_point[0], ent_point[1], ent_point[2]);
    }

    // Assign the entanglement point to the online data structure
    online_data.ent_point.at(0) = ent_point[0];
    online_data.ent_point.at(1) = ent_point[1];
    online_data.ent_point.at(2) = rotate_dir_ * err_sum_; // Ensure Z-coordinate is 0.0

    //ROS_INFO("Assigned entanglement point to online data: x=%f, y=%f, z=%f",
            // online_data.ent_point[0], online_data.ent_point[1], online_data.ent_point[2]);

    // Select the obstacle center based on the contact status
    if (contactPoint != nullptr)
    {
        // If contact is made, use the contact point as the obstacle center
        online_data.obs_centre.at(0) = contactPoint->x;
        online_data.obs_centre.at(1) = contactPoint->y;
        //online_data.obs_centre.at(2) = dist_obs_; // Z-coordinate is 0.0
        //ROS_INFO("Obstacle center (from contact): x=%f, y=%f, z=%f",
                 //online_data.obs_centre[0], online_data.obs_centre[1], online_data.obs_centre[2]);
    }
    else
    {

        // Otherwise, select the closest obstacle
        online_data.obs_centre.at(0) = closest_obs[0];
        online_data.obs_centre.at(1) = closest_obs[1];
        //online_data.obs_centre.at(2) = dist_obs_; // Z-coordinate is 0.0
        //ROS_INFO("Obstacle center (from closest obstacle): x=%f, y=%f, z=%f",
                 //online_data.obs_centre[0], online_data.obs_centre[1], online_data.obs_centre[2]);
    }
    ent_point_[0]= ent_point[0];
    ent_point_[1]= ent_point[1];
    //online_data.ent_point.at(0) = ent_point_[0] ;
    //online_data.ent_point.at(1) = ent_point_[1] ;
    online_data.ent_point.at(0) = ent_point_[0] - current_pos_att[0] ;
    online_data.ent_point.at(1) = ent_point_[1] - current_pos_att[1] ;
    //online_data.ent_point.at(2) = - rotate_dir_ * err_sum_ *1000.0;
    online_data.ent_point.at(2) =  -error_sum_factor_ * err_sum_ ;  

    
    dist_obs_ = sqrt((online_data.obs_centre.at(0) - current_pos_att[0]) * (online_data.obs_centre.at(0) - current_pos_att[0])  + 
                (online_data.obs_centre.at(1) - current_pos_att[1]) * (online_data.obs_centre.at(1) - current_pos_att[1])) ; 
    online_data.obs_centre.at(2) = dist_obs_; // Z-coordinate is 0.0


}

// Callbacks for Neptune Planner and Tether Data
// ===========================







void neptune_planner_traj_cb(const nav_msgs::Path::ConstPtr &msg)
{
    neptune_planner_count = 0;

    neptune_planner_size = msg->poses.size();

    for (const auto &pose_stamped : msg->poses)
    {
        // Extract position data and push it to the respective trajectory vectors
        neptune_planner_x.push_back(pose_stamped.pose.position.x);
        neptune_planner_y.push_back(pose_stamped.pose.position.y);
        neptune_planner_z.push_back(pose_stamped.pose.position.z);
    }
}




void ref_traj_cb(const nav_msgs::Path::ConstPtr& msg)
{

    size_refs =msg->poses.size();


   for (const auto& pose_stamped : msg->poses) {
        // Extract position data and push it to the respective trajectory vectors
    ref_traj_x.push_back(pose_stamped.pose.position.x);
       ref_traj_y.push_back(pose_stamped.pose.position.y);
        ref_traj_z.push_back(pose_stamped.pose.position.z);
    }


// Using back() to get the most recent point
if (!msg->poses.empty()) {
    test_x = msg->poses.back().pose.position.x;
    test_y = msg->poses.back().pose.position.y;


}
}


 
void neptune_trajectory_cb(const visualization_msgs::Marker::ConstPtr& msg)
{
    // Check if there are any points in the message
    if (!msg->points.empty()) {
        // Get the latest point (last element in the points array)
        geometry_msgs::Point latest_point = msg->points.back();
        
        // Extract the position values
        neptune_x = latest_point.x;
        neptune_y = latest_point.y;
    }
  
}



void neptune_goal_cb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Save the point data to neptune_goal
    neptune_goal = msg->point;
                
if (!std::isnan(neptune_goal.x) && !std::isnan(neptune_goal.y)) {
    ref_position[0] = neptune_goal.x;
    ref_position[1] = neptune_goal.y;
}
}


void neptune_tether_cb(const visualization_msgs::Marker::ConstPtr &msg)
{
    // Check if the points array has elements
    // tether_positions.clear();
    tether_positions.resize(msg->points.size());

    if (!msg->points.empty())
    {
        // Iterate through all points in the message and add them to tether_positions
        // for (const auto& point : msg->points) {
        //    tether_positions.push_back(point);  // Add each point to the vector
        // }
        for (int i = 0; i < msg->points.size(); i++)
        {

            tether_positions[i] = msg->points[i];
        }

        tether_end_x = tether_positions.back().x;
        tether_end_y = tether_positions.back().y;

        base_pos = {msg->points.front().x, msg->points.front().y};

    }
}


// Callbacks for External References and State feedback from Mobula
// ===========================


void ref_position_cb(const geometry_msgs::Vector3::ConstPtr &msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr &msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void ref_yaw_cb(const std_msgs::Float64::ConstPtr &msg)
{
    ref_yaw_rad = msg->data;
}

void pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z,
                        msg->twist.twist.angular.x,
                        msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z};

    current_pos_att = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}

void orientation_cb(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{

    angles = {msg->vector.x * (M_PI / 180),
              msg->vector.y * (M_PI / 180),
              msg->vector.z * (M_PI / 180)};
    angles_d = {msg->vector.x,
                msg->vector.y,
                msg->vector.z};
}



// Publishers
// ===========================


void publishTetherPath(const std::vector<std::vector<double>>& contact_points, 
                       ros::Publisher& path_pub) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // Set your frame of reference, e.g., "map" or "world"
    
    for (const auto& point : contact_points) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        pose.pose.position.z = 0.0; // Assuming a 2D plane; adjust if needed

        // Optional: Set orientation if the tether direction is significant
        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
}








void NMPC_PC::publish_wrench(struct command_struct &commandstruct)
{

    geometry_msgs::Wrench nmpc_wrench_msg;

    nmpc_wrench_msg.force.x = commandstruct.control_wrench_vec[0];
    nmpc_wrench_msg.force.y = commandstruct.control_wrench_vec[1];
    nmpc_wrench_msg.force.z = commandstruct.control_wrench_vec[2];

    nmpc_wrench_msg.torque.x = 0.0;
    nmpc_wrench_msg.torque.y = 0.0;
    nmpc_wrench_msg.torque.z = commandstruct.control_wrench_vec[3];

    nmpc_cmd_wrench_pub.publish(nmpc_wrench_msg);

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub.publish(obj_val_msg);
}

void NMPC_PC::publish_pred_trajectory(struct acado_struct &traj_struct)
{
    // Create an instance of the Path message type (new topic)
    nav_msgs::Path pred_traj_viz_msg;

    // Set the frame_id for the path (use your frame of reference, e.g., "map")
    pred_traj_viz_msg.header.frame_id = "world";  
    pred_traj_viz_msg.header.stamp = ros::Time::now();  // Timestamp the path

    // Populate the Path message with the trajectory data
    for (int i = 0; i < NMPC_N * NMPC_NX; i += NMPC_NX) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = ros::Time::now();  // Timestamp for each pose

        // Assuming `nmpc_struct.x` contains the state data as [x, y, z] every 3 elements
        pose.pose.position.x = nmpc_struct.x[i + 0];  // x position
        pose.pose.position.y = nmpc_struct.x[i + 1];  // y position
        pose.pose.position.z = nmpc_struct.x[i + 2];  // z position

        // Optionally, set orientation, assuming default (no rotation)
        pose.pose.orientation.w = 1.0;  // Identity quaternion (no rotation)

        // Add this pose to the path
        pred_traj_viz_msg.poses.push_back(pose);
    }

    // Publish the trajectory path for visualization
    pred_traj_viz_pub.publish(pred_traj_viz_msg);
}



void publish_path_points(const std::vector<double>& point)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Change to your desired frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the color of the points (for example, red)
    marker.color.r = 1.0f;  // Red
    marker.color.g = 0.0f;  // Green
    marker.color.b = 0.0f;  // Blue
    marker.color.a = 1.0f;  // Fully opaque

    marker.scale.x = 0.5;  // Set the size of the points
    marker.scale.y = 0.5;

    // Add the single point to the marker
    geometry_msgs::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = 0.0;  // If you want 3D points, you can modify the z-value
    marker.points.push_back(p);

    // Publish the marker
    path_pub.publish(marker);
}


void publishPointStamped(ros::Publisher& odom_point_pub) {
    // Get the current position directly within the function (example values)
    //std::vector<double> current_pos = {1.0, 2.0, 3.0}; // Example values for position

    // Create a PointStamped message
    geometry_msgs::PointStamped point_msg;

    // Set the timestamp and frame_id
    point_msg.header.stamp = ros::Time::now();  // Set the timestamp to the current time
    point_msg.header.frame_id = "world";         // Set the frame ID to your global frame (change if needed)

    // Set the position values from the current position vector
    point_msg.point.x = current_pos_att.at(0);
    point_msg.point.y = current_pos_att.at(1);
    point_msg.point.z = current_pos_att.at(2);

    // Publish the PointStamped message
    odom_point_pub.publish(point_msg);
}


void publishEntanglementPoint(ros::Publisher &entanglement_point_pub) {
    // Create a message to publish the entanglement point
    geometry_msgs::PointStamped ent_point_msg;
    ent_point_msg.header.stamp = ros::Time::now();
    ent_point_msg.header.frame_id = "world";  // Modify with the correct frame
    ent_point_msg.point.x = ent_point_[0];
    ent_point_msg.point.y = ent_point_[1];
    ent_point_msg.point.z = ent_point_[2];
    
    // Publish the entanglement point
    entanglement_point_pub.publish(ent_point_msg);
}

void publishClosestObstacle(ros::Publisher &pub) {
    // Create a message to publish the closest obstacle
    geometry_msgs::PointStamped closest_obs_msg;
    closest_obs_msg.header.stamp = ros::Time::now();
    closest_obs_msg.header.frame_id = "world";  // Modify with the correct frame
    closest_obs_msg.point.x = online_data.obs_centre[0];
    closest_obs_msg.point.y = online_data.obs_centre[1];
    closest_obs_msg.point.z = online_data.obs_centre[2];
    
    // Publish the closest obstacle
    pub.publish(closest_obs_msg);
}



// params initiliazations
// ===========================

// Function to initialize ROS parameters
void initializeParams(ros::NodeHandle& nh, nmpc_struct_& nmpc_struct, bool& online_ref_yaw, bool& use_dist_estimates) {

    ros::param::get("verbose", nmpc_struct.verbose);
    ros::param::get("yaw_control", nmpc_struct.yaw_control);
    ros::param::get("online_ref_yaw", online_ref_yaw);
    ros::param::get("use_dist_estimates", use_dist_estimates);
    ros::param::get("W_Wn_factor", nmpc_struct.W_Wn_factor);


    int u_idx = 0;
    ros::param::get("F_x_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_y_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_z_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("Mz_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("slack_ref", nmpc_struct.U_ref(u_idx++));

    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    ros::param::get("W_x", nmpc_struct.W(w_idx++));
    ros::param::get("W_y", nmpc_struct.W(w_idx++));
    ros::param::get("W_z", nmpc_struct.W(w_idx++));
    ros::param::get("W_u", nmpc_struct.W(w_idx++));
    ros::param::get("W_v", nmpc_struct.W(w_idx++));
    ros::param::get("W_w", nmpc_struct.W(w_idx++));
    ros::param::get("W_psi", nmpc_struct.W(w_idx++));
    ros::param::get("W_r", nmpc_struct.W(w_idx++));
    ros::param::get("W_e", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fx", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fy", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fz", nmpc_struct.W(w_idx++));
    ros::param::get("W_Mz", nmpc_struct.W(w_idx++));
    ros::param::get("W_slack", nmpc_struct.W(w_idx++));

    assert(w_idx == NMPC_NY);


}

// Function to initialize publishers
void initializePublishers(ros::NodeHandle& nh) {
    nmpc_cmd_wrench_pub = nh.advertise<geometry_msgs::Wrench>("/mobula/rov/wrench", 1, true);
    nmpc_cmd_exeTime_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, true);
    //nmpc_cmd_kkt_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, true);
    nmpc_cmd_obj_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/obj", 1, true);
    nmpc_pred_traj_pub = nh.advertise<std_msgs::Float64MultiArray>("nmpc_predicted_trajectory", 1, true);
    //s_sdot_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1, true);
    odom_point_pub = nh.advertise<geometry_msgs::PointStamped>("odom_point", 10);
    path_pub = nh.advertise<visualization_msgs::Marker>("/tether_path/", 10);
    pred_traj_viz_pub = nh.advertise<nav_msgs::Path>("nmpc_predicted_trajectory_viz", 1);
    entanglement_point_pub = nh.advertise<geometry_msgs::PointStamped>("entanglement_point", 10);
    closest_obstacle_pub = nh.advertise<geometry_msgs::PointStamped>("closest_obstacle", 10);
    eampc_tether_pub = nh.advertise<nav_msgs::Path>("eampc_tether", 10);
    //state_to_neptune_pub = nh.advertise<snapstack_msgs::State>("/firefly1/neptune/state1", 10);

}

// Function to initialize subscribers
void initializeSubscribers(ros::NodeHandle& nh) {
    ref_traj_sub = nh.subscribe<nav_msgs::Path>("/firefly1/neptune/sampled_traj3", 1, ref_traj_cb);
    ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
    orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);

    // Neptune-specific subscribers
   // neptune_obstacles_sub = nh.subscribe<decomp_ros_msgs::PolyhedronArray>("/firefly1/neptune/poly_safe", 1, obstacles_cb);
    neptune_tether_sub = nh.subscribe<visualization_msgs::Marker>("/firefly1/neptune/tether_rough", 1, neptune_tether_cb);
    neptune_trajectory_sub = nh.subscribe("/firefly1/neptune/actual_traj", 10, neptune_trajectory_cb);
    neptune_goal_sub = nh.subscribe("/firefly1/neptune/point_G", 10, neptune_goal_cb);
}




int main(int argc, char **argv)
{

    // ROS node initialization
    ros::init(argc, argv, "bluerov2_nmpc_node");
    ros::NodeHandle nh;


    // Initialize vectors for storing position, velocity, and force data
    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_vel_body.resize(6);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);
    ref_traj_x.resize(20);
    ref_traj_y.resize(20);
    ref_traj_z.resize(20);
    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    ref_traj_type = 0;
    ref_position << 0, 0, 0;
    ref_velocity << 0, 0, 0;
    ent_point_ = std::vector<double>(3, 0.0);  // ent_point_ is now a vector of 3 elements, all initialized to 0.0
    closest_obs = std::vector<double>(3, 0.0);  // closest_obs_ is now a vector of 3 elements, all initialized to 0.0
    err_vector_ = std::vector<double>(integ_error_h_, 0.0); // Define and initialize with 101 elements set to 0.0

    // Initialize parameters, publishers, and subscribers
    initializeParams(nh, nmpc_struct, online_ref_yaw, use_dist_estimates);
    initializePublishers(nh);
    initializeSubscribers(nh);
   
    // Set sample time and initialize NMPC
    nmpc_struct.sample_time = sampleTime;
    NMPC_PC *nmpc_pc = new NMPC_PC(nmpc_struct);
    ros::Rate rate(1 / sampleTime);




    // Initialize online data struct
    online_data.distFx.assign(NMPC_N + 1, 0.0);
    online_data.distFy.assign(NMPC_N + 1, 0.0);
    online_data.distFz.assign(NMPC_N + 1, 0.0);
    online_data.ent_point.assign(NMPC_N + 1, 0.0);
    online_data.obs_centre.assign(NMPC_N + 1, 0.0);

    angles = {0, 0, 0};
    control_stop = false;
    int loop_counter = 0;
 

    // MPC initilization 
    // ===========================
    online_data.distFx.resize(NMPC_N + 1);
    online_data.distFy.resize(NMPC_N + 1);
    online_data.distFz.resize(NMPC_N + 1);

    std::fill(online_data.distFx.begin(), online_data.distFx.end(), 0.0);
    std::fill(online_data.distFy.begin(), online_data.distFy.end(), 0.0);
    std::fill(online_data.distFz.begin(), online_data.distFz.end(), 0.0);
    

    pos_ref = current_pos_att;

    if (!nmpc_pc->return_control_init_value())
    {
        nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
        if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
        {
            std::cout << "***********************************\n";
            std::cout << "NMPC: initialized correctly\n";
            std::cout << "***********************************\n";
        }
    }

    // Main Loop 
    // ===========================
    
    while (ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec(); //get ros time
  
        current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              angles.at(2),
                              current_vel_rate.at(5),
                              online_data.obs_centre[0],     //ent_point[0],
                              online_data.obs_centre[1],      //ent_point[1],
                              0.0,
                              online_data.obs_centre[0],      //obs_centre[0],
                              online_data.obs_centre[1],      //obs_centre[1],
                              0.0,
                              0.0     //slack variable  
                              } ; 

            std::vector<double> rov_position = {tether_end_x, tether_end_y, 0.0};

            //std::vector<double> rov_position = {0.0, 0.0, 0.0};
             
 


            ref_trajectory = { ref_position[0], // x
                               ref_position[1], // y
                                0.0, // z
                                0.0, // u
                                0.0, // v
                                0.0, // w
                                0.0,  //psi
                                0.0,  //r
                                0.0 //c_ent
                                };//r
//                              0.0   //e cost
                            //  };

            std::cout << "current_states = ";
            for (int idx = 0; idx < current_states.size(); idx++)
            {
                std::cout << current_states[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_trajectory = ";
            for (int idx = 0; idx < ref_trajectory.size(); idx++)
            {
                std::cout << ref_trajectory[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "Closest Obstacle = ";
            for (int idx = 0; idx < 2; idx++)
            {
                std::cout << online_data.obs_centre[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "\033[31mEntanglement vector = ";
            for (int idx = 0; idx < 2; idx++)
            {
                std::cout << online_data.ent_point[idx] << ",";
            }
            std::cout << "\033[0m\n";  // Reset color to default



            // Call updateEntanglementData to update entanglement data
            updateEntanglementData(tether_positions, obstacle_centers, rov_position, neptune_goal, online_data);
            publishEntanglementPoint(entanglement_point_pub);
            publishClosestObstacle(closest_obstacle_pub);



           entanglement_integration_function();
            entagelement_rotation_direction();

            //publishTetherPath(contact_points, path_pub);
            
            nmpc_pc->nmpc_core(nmpc_struct,
                               nmpc_pc->nmpc_struct,
                               nmpc_pc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;
               //  cout<<"feedback failed"<<endl;

            if (std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
                std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                control_stop = true;
                exit(0);
            }


            nmpc_pc->publish_pred_trajectory(nmpc_pc->nmpc_struct);
            nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);
            nmpc_pc->publish_wrench(nmpc_pc->nmpc_cmd_struct);

            publishPointStamped(odom_point_pub);

            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}





















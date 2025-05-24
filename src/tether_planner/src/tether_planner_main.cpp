#include "collision_checker.hpp"
#include "nvblox_functions.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <rope_rrt.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "tether_planner");
  ros::NodeHandle nh;

  // Load initial position from YAML file
  std::string config_path =
      "/home/hakim/tether_planning_ws/src/tether_planner/config/tether_params.yaml";
  YAML::Node config = YAML::LoadFile(config_path);
  double initial_x = config["initial_position"]["x"].as<double>();
  double initial_y = config["initial_position"]["y"].as<double>();
  double initial_z = config["initial_position"]["z"].as<double>();

  ros::Time ros_time;
  ros::Time last_time =
  ros::Time::now(); // Track the last time position was updated
  ros_time = ros::Time::now();
  ros::Rate rate(1 / time_step);
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

  std::shared_ptr<ompl::base::SpaceInformation> si, si_t;
  initializeROS(argc, argv, nh);
  initializePublishers(nh);
  //initializeSubscribers(nh);
  initializeGlobalVariables();
  loadParameters(nh);
  initializeObstacles();
  initializePointCloud(cloud);

  ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/mobula/rov/odometry", 1, pos_cb);
  ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/mobula/rov/orientation", 1, orientation_cb);
  ros::Subscriber reset_tether_sub = nh.subscribe("reset_tether_topic", 10, reset_tether_cb);
  ros::Subscriber record_trajectory_sub = nh.subscribe("record_trajectory_topic", 10, record_trajectory_on_cb);
  ros::Subscriber goal_point_sub = nh.subscribe<geometry_msgs::PointStamped>("goal_point_pub", 10, goal_cb);



  initializeOMPLSpace(si, si_t);
  loadXYZFileToPCL(input_xyz_file);
  Eigen::Matrix3f rotation;
  rotation <<  1, 0, 0,
               0, 1, 0,
               0, 0, 1; // Rotation matrix to negate x, y, and z

  Eigen::Matrix3f rotation2;
  rotation2 <<  1, 0, 0,
               0, cos(M_PI/2), -sin(M_PI/2),
               0, sin(M_PI/2), cos(M_PI/2); // Rotation matrix for 90 degrees around X-axis


  Eigen::Vector3f translation(0.0, 0.0, 1.0); // Example translation
  Eigen::Vector3f translation2(0.0, 0.0, 0.0); // Example translation
  Eigen::Vector3f translation3(0.0, 0.0, -2.0); // Example translation

  // Transform the point cloud
  transformEIVAPC( eiva_pcloud, translation, rotation);
  distanceThreshold = 10.0;
  //Eigen::Vector3f translation_local(2.0, 1.0, 7.5);


  //Eigen::Matrix3f rotation_local = Eigen::DiagonalMatrix<float, 3>(-1, -1, -1);
  scale_factor = 0.05;
    //transformPointCloud(cloud, scale_factor, translation_local, rotation_local);
  loadAndTransformWaypoints(txtfilePath, distanceThreshold * 0.5, scale_factor * 1.0 ,
                             translation3, rotation2, way_point_traj);
                             
                             transformSTLModel(inspection_model_stl, 1.0, translation2, rotation);
                             transformPointCloud(cloud , scale_factor *20 , translation, rotation);

  //transformWaypoints(way_point_traj, 1.5, 0 * translation, rotation);

  //std::unique_ptr<nvblox::Mapper> pipe_map = mapFromPointCloud(points);




  for (int i = 0; i < 10; ++i) {
    ros::spinOnce();
    rate.sleep();
  }
   
  // Sparsify and save the point cloud
  float leaf_size = 0.02f; // Adjust the leaf size as needed
  std::string output_pcd_file= "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.pcd";
  //sparsifyAndSavePointCloud(cloud, leaf_size, output_pcd_file);
  // Convert the PCD file to XYZ and save it
  std::string output_xyz_file = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.xyz";
  convertPCDToXYZ(output_pcd_file, output_xyz_file);

  std::vector<Eigen::Vector3f> points = pclToEigen(cloud);

  //loadNvbloxModel(nvblx_file_path);

  //scale_factor = 1.0;
  rotation <<  1, 0, 0,
               0, -1, 0,
               0, 0, -1; // Rotation matrix to negate x, y, and z
  std::unique_ptr<nvblox::Mapper> pipe_map = mapFromPipe(1.0, rotation, translation  );
  const nvblox::EsdfLayer& esdf_layer = pipe_map->esdf_layer();
  loadNvbloxModel(nvblx_file_path);


  TetherPlanner planner(
      delta,
      equivalenceTolerance); // Create an instance of the TetherPlanner class

  //CollisionChecker collision_checker(*mapper);
  //CollisionChecker collision_checker(mapper, collision_threshold);
  CollisionChecker collision_checker(*mapper, collision_threshold);


   //testing isStateValidSTL

   auto isStateValid = [&collision_checker](const ompl::base::State *state) {
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::Vector3d point(realState->values[0], realState->values[1], realState->values[2]);
    return collision_checker.isCollisionFree(point, collision_threshold);
  };

  auto isStateValid_path = [&collision_checker](const ompl::base::State *state) {
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::Vector3d point(realState->values[0], realState->values[1], realState->values[2]);
    return collision_checker.isCollisionFree(point, collision_threshold + 0.03 );
  };

  si_t->setStateValidityChecker(isStateValid);
  si->setStateValidityChecker(isStateValid_path);
  si->setMotionValidator(std::make_shared<CustomMotionValidator>(si));
 

  

  //////////////
  // Main loop
  /////////////
  bool finding_safe_path = false;
  ompl::geometric::PathGeometric PathAlongTether(si);
  ompl::geometric::PathGeometric EntTetherConfiguration(si);
  ompl::geometric::PathGeometric P_t_path(si);
  ompl::geometric::PathGeometric PathSafe(si);
  ompl::geometric::PathGeometric PathIntermediate(si);
  int i_safe_path = 0;
  EntTetherConfiguration = P_t_path;
  bool path_is_safe = false;
  bool first_compute = true;

   auto *state_rov =
        si_t->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
   
   
   auto *state_tms =
        si_t->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();     
        
    state_tms->values[0] = initial_x; // x-coordinate
    state_tms->values[1] = initial_y; // y-coordinate
    state_tms->values[2] = initial_z; // z-coordinate


    if (si_t->isValid(state_tms)) {
      P_t.append(state_tms);
      P_t_path.append(state_tms);
  }
  ros::Time node_start_time = ros::Time::now();

  while (ros::ok()) {
    
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-50);
    bounds.setHigh(50);
    space->setBounds(bounds);

    si->setStateValidityChecker(isStateValid_path);

    // Set the custom motion validator
    si->setMotionValidator(std::make_shared<CustomMotionValidator>(si));
    si->setup();

    si_t->setStateValidityChecker(isStateValid);

    // Set the custom motion validator
    si_t->setMotionValidator(std::make_shared<CustomMotionValidator>(si_t));
    si_t->setup();


    state_rov->values[0] = current_pos_att[0]; // x-coordinate
    state_rov->values[1] = current_pos_att[1]; // y-coordinate
    state_rov->values[2] = current_pos_att[2]; // z-coordinate
    // ROS_INFO("Current position: [%f, %f, %f]", current_pos_att[0],
    // current_pos_att[1],
    //         current_pos_att[2]);

   if ((ros::Time::now() - node_start_time).toSec() > 2.0) {
        if (si->isValid(state_rov)) {
            P_t.append(state_rov);
            P_t_path.append(state_rov);
        }
    }
    
    /////////////////////
    // COMPUTE TETHER MODEL
    //////////////////////
    ompl::geometric::PathSimplifier simplifier(si_t);
    ompl::geometric::PathSimplifier simplifier_path(si);

    // Measure the time taken by the ropeRRTtether method
    auto start_time = std::chrono::high_resolution_clock::now();

    // calculate tether P_t
    bool tether_computed = simplifier.ropeRRTtether(P_t, contactPoints, delta,
                                                    equivalenceTolerance);

  bool path_computed = simplifier_path.ropeRRTtether(P_t_path, contactPoints, delta,
    equivalenceTolerance);

    auto end_time = std::chrono::high_resolution_clock::now();
    time_tether_model_computation = end_time - start_time;

    /////////////////////
    // GlOBAL PlANNER
    /////////////////////

    // go to the next way point (TODO make a function)
   
    if (count > 0 && count < way_point_traj.size()) {
      way_point = way_point_traj[count];
      //ROS_INFO("Waypoint %d: [%f, %f, %f]", count, way_point[0], way_point[1],
             //  way_point[2]);
    }
    double dist = distance(way_point, current_pos_att);

    if (dist < 0.05 &&
        count < way_point_traj.size() - 1 && finding_safe_path == false) {
        ROS_INFO("Waypoint reached:");
      goal_reached = true;
      count++; 
    }
    //ROS_INFO("Count: %d", count);


    // ompl::geometric::PathGeometric Path_sample(si);
    //ompl::geometric::PathGeometric Path_safe(si);
    double Tether_length = planner.findTetherLength(P_t);
   // ROS_INFO("Tether length is %d", Tether_length);
   std::cout << "Tether length is " << planner.findTetherLength(P_t) << std::endl;
   std::cout << "Max allowed Tether length is " << L_max << std::endl;
   std::cout << "EntTetherConfiguration size" << EntTetherConfiguration.getStateCount() << std::endl;

   
   if ((planner.findTetherLength(P_t)>L_max && finding_safe_path == false) 
    || (planner.findTetherLength(P_t)>L_max && PathAlongTether.getStateCount()<3) )  
  // if ((planner.findTetherLength(P_t)>L_max && finding_safe_path == false))
   {
    //std::cout << "EntTetherConfiguration size" << EntTetherConfiguration.getStateCount() << std::endl;

    EntTetherConfiguration = P_t;
    finding_safe_path = true;
    i_safe_path = 0;
    //path_is_safe = false;
    if (EntTetherConfiguration.getStateCount()>2){
      PathAlongTether = planner.SearchAlternativePath( 
       EntTetherConfiguration, way_point, si, L_max); 
       std::cout << "Alternative Path Calculated" <<std::endl;

       //bool is_collision_free = planner.checkPath(path, si);
    
     }
     else{
      PathAlongTether = planner.InvertTetherPath(P_t,si);
      std::cout << "Path length is small inverting tether" <<std::endl;

    }
    
    PathIntermediate = PathAlongTether;
    

}
   
   std::cout << "Path Along Tether size" << PathAlongTether.getStateCount() << std::endl;


    double safe_distance = 1.0;


     //Offsetting path

   /////
  //Offsetting path
  ///////
  

  if( !path_is_safe  && PathIntermediate.getStateCount() > 2){
     std::cout <<"Offsetting Path"<<std::endl;

      //PathIntermediate = planner.PopPathWaypoint(
      // PathIntermediate, way_point_traj, si_t, safe_distance * 2.0);
      // PathIntermediate = planner.PopPath(PathIntermediate, esdf_layer, safe_distance * 1.0);

      std::cout <<"Popped Path Waypoint"<<std::endl;


      //planner.PopPathSample(PathIntermediate, si, 2.6);

      std::cout <<"Popped Path PopPathSample"<<std::endl;

      //PathIntermediate = planner.smoothPathWithSpline(PathIntermediate, si);
     // PathIntermediate = planner.smoothPathWithPolynomial(PathIntermediate, si);
      //planner.PopPathSample(PathIntermediate, si, 2.0);
      //PathIntermediate = planner.smoothPathWithPolynomial(PathIntermediate, si);
      planner.PopTetherCentroid(PathIntermediate, si, delta);
      std::cout <<"Smoothed Path"<<std::endl;

    } 
      
  
  
   ///////////
   // RRT from exit point 
   ///////////
   
    if (PathSafe.getStateCount() > 2 && i_safe_path == PathSafe.getStateCount()-2) {
     const auto *last_state = PathIntermediate.getState(PathIntermediate.getStateCount() - 1)->as<ompl::base::RealVectorStateSpace::StateType>();

     // Define the goal state
     ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(si->getStateSpace());
     state_goal->values[0] = way_point[0];
     state_goal->values[1] = way_point[1];
     state_goal->values[2] = way_point[2];
    
     // Create a ScopedState for the last state
     ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_last(si->getStateSpace());
     state_last->values[0] = last_state->values[0];
     state_last->values[1] = last_state->values[1];
     state_last->values[2] = last_state->values[2];
    
  //   ///////////////////////////////////////////////////////////////
  //   // Find the shortest path from the last state to the goal state
  //   ///////////////////////////////////////////////////////////////

   PathIntermediate = planner.FindShortestPath(state_last, state_goal, si);
    
   bool calculated_final_segment = simplifier.ropeRRTtether( PathIntermediate, contactPoints, delta,
    equivalenceTolerance);
    //  calculated_final_segment = simplifier.ropeShortcutPath( PathIntermediate, delta,
    //    equivalenceTolerance);
    //  Append the resulting path to PathIntermediate
     
     //if( !path_is_safe  && PathIntermediate.getStateCount() > 1){
      std::cout <<"Offsetting last segment in path"<<std::endl;
     //  PathIntermediate = planner.PopPathWaypoint(
     //   PathIntermediate, way_point_traj, si_t, safe_distance * 1.0);
      //PathIntermediate = planner.PopPath(PathIntermediate, esdf_layer, safe_distance * 0.5);
     planner.PopTetherCentroid(PathIntermediate, si, delta * 1.5);


       planner.PopPathSample(PathIntermediate, si, delta);
      // PathIntermediate = planner.smoothPathWith5thOrderPolynomial(PathIntermediate, si);
       PathIntermediate = planner.smoothPathWithPolynomial(PathIntermediate, si);
       PathSafe =  PathIntermediate ;      
       i_safe_path = 0;
     //} 


   }






 
 if (!path_is_safe || PathSafe.getStateCount() < 3 || i_safe_path == 0 ){
  
 PathSafe = PathIntermediate;
 ROS_INFO("\033[1;31mPath is safe: %s\033[0m", path_is_safe ? "true" : "false");
 ROS_INFO("\033[1;31mPathSafe state count: %zu\033[0m", PathSafe.getStateCount());
 ROS_INFO("\033[1;31mi_safe_path: %d\033[0m", i_safe_path);

}

if (finding_safe_path) {
  ROS_INFO("\033[1;31mfinding_safe_path is true\033[0m"); // Red color
} else {
  ROS_INFO("\033[1;31mfinding_safe_path is false\033[0m"); // Red color
}



 path_is_safe = planner.checkPath(PathSafe, si);
 

     if (finding_safe_path==true && path_is_safe == true) {
       way_point = planner.getPointAlongPath(PathSafe, i_safe_path);
      
       if (distance(way_point, current_pos_att) < 0.1){
       i_safe_path++;
      }
    }
     else{
      way_point = way_point_traj[count];          
     }
     
     if (i_safe_path >= PathSafe.getStateCount() || 
     PathSafe.getStateCount() == 0) 
      {
      finding_safe_path = false;
      i_safe_path = 0;
   
      }

 
  
    // Print whether finding_safe_path is true or false
if (finding_safe_path) {
  ROS_INFO("finding_safe_path is true");
} else {
  ROS_INFO("finding_safe_path is false");
}
    

    for (std::size_t i = 0; i < Path_sample.getStateCount(); ++i) {
      const auto *state =
          Path_sample.getState(i)
              ->as<ompl::base::RealVectorStateSpace::StateType>();
    }

    if (TA_Planner_ON == true) {

      for (std::size_t i = 0; i < Path_sample.getStateCount(); ++i) {
        const auto *state =
            Path_sample.getState(i)
                ->as<ompl::base::RealVectorStateSpace::StateType>();
      }
    }

    //////////
    // Publish
    //////////
    publishAllData();
    //if (!finding_safe_path){
     // PathSafe.clear();
     // PathAlongTether.clear();
   // }
   if (finding_safe_path) {
    publishTetherPath(safe_planner_path_pub, PathSafe, "world", tetherColor);
    publishTetherPath(planner_path_pub, PathAlongTether, "world", tetherColor);
} else {
    ompl::geometric::PathGeometric empty_path(si);
    publishTetherPath(safe_planner_path_pub, empty_path, "world", tetherColor);
    publishTetherPath(planner_path_pub, empty_path, "world", tetherColor);
}


   // }

    //printPath(PathSafe);

   
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

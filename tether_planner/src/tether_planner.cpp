
#include "tether_planner.hpp"
#include <boost/type_traits/is_reference.hpp>
#include <filesystem>
#include <ompl/geometric/PathGeometric.h>
#include <vector>
#include "publishers.hpp"

/*
TetherPlanner::TetherPlanner()
{
   //Constructor
   std::cout<<"Tether Planner Initialized"<<std::endl;

}
*/

TetherPlanner::TetherPlanner(double delta, double equivalenceTolerance)
    : delta_(delta), equivalenceTolerance_(equivalenceTolerance) {
  std::cout << "Tether Planner Initialized" << std::endl;
}

std::vector<double> TetherPlanner::SearchRandomDirection() {
  double theta = static_cast<double>(rand()) / RAND_MAX * 2.0 *
                 M_PI; // Random angle in [0, 2*pi]
  double phi =
      static_cast<double>(rand()) / RAND_MAX * M_PI; // Random angle in [0, pi]

  std::vector<double> direction(3);
  direction[0] = sin(phi) * cos(theta);
  direction[1] = sin(phi) * sin(theta);
  direction[2] = cos(phi);

  return direction; // Already a unit vector
}

ompl::geometric::PathGeometric TetherPlanner::findNextGoal(
    const ompl::geometric::PathGeometric &tether,
    const std::vector<double> &current_position,
    const std::vector<double> &goal, double &L_max,
    const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {

  // init states and bounds

  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(-30);
  bounds.setHigh(30);
  std::vector<double> point_exit(3); // Change to std::vector<double> of size 3

  ompl::geometric::PathGeometric Path_ref(si);
  ompl::geometric::SimpleSetup ss_rrt(si);
  ompl::geometric::PathGeometric Path_replan(si);
  bool ENT = 0;

  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(space);
  state_goal->values[0] = goal[0]; // x-coordinate
  state_goal->values[1] = goal[1]; // y-coordinate
  state_goal->values[2] = goal[2]; // z-coordinate

  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_rov_scoped(
      space);
  state_rov_scoped->values[0] = current_position[0]; // x-coordinate
  state_rov_scoped->values[1] = current_position[1]; // y-coordinate
  state_rov_scoped->values[2] = current_position[2]; // z-coordinate
  ompl::base::PlannerStatus solved_rrt;

  std::vector<double> exit_point;
  double L = findTetherLength(tether);
  std::vector<double> goal_next;

  std::vector<double> goal_last = goal_next;
  UpdateExitPointsList(tether);

  std::cout << "Current tether length: " << L << std::endl;
  if (L > 0 && exit_points_list_.size() > 0) {
    if (L > L_max && ENT == 0) {
      std::cout << "Tether constraint violated. Activating entanglement."
                << std::endl;
      // Tether constraint violated - activation entanglement, going back along
      // path and find exit point
      UpdateExitPointsList(tether);
      std::cout << "Updated Exit Points List." << std::endl;

      std::cout << "exit_points_list_.size()." << exit_points_list_.size()
                << std::endl;

      ENT = 1;
      for (int i = 0; i < exit_points_list_.size(); i++) {
        double L_replan_length = findTetherLengthReplannedPath(
            tether, exit_points_list_[i], goal, space, si);
        // double L_replan_length = 10.0;
        std::cout << "Replanned tether length: " << L_replan_length
                  << std::endl;
        if (L_replan_length < L_max) {
          // Found exit point, replanning path to goal
          std::cout << "Found exit point. Replanning path to goal."
                    << std::endl;
          Path_replan = ReplanPath(tether, exit_points_list_[i], goal, si);
          point_exit = exit_points_list_[i];
          break;
        }
      }
    } else if (L < L_max && ENT == 0) {
      std::cout << "Tether constraint not violated. Going directly to goal."
                << std::endl;
      // Tether constraint not violated - go directly to goal
      Path_ref = FindShortestPath(state_rov_scoped, state_goal, si);
    } else if (ENT == 1 && exit_points_list_.size() > 0) {
      std::cout << "Following replanned path." << std::endl;
      goal_next =
          FindNextPointAlongPath(current_position, goal_last, Path_replan);
      if (isEqual(current_position, point_exit)) {
        std::cout << "Reached exit point. Deactivating entanglement."
                  << std::endl;
        ENT = 0;
      }
    }

    // std::cout << "Next goal: [" << goal_next[0] << ", " << goal_next[1] << ",
    // " << goal_next[2] << "]" << std::endl;
  }

  // std_msgs::ColorRGBA ropepathColor;
  // ropepathColor.r = 0.6f;  // Red
  // ropepathColor.g = 0.6f;  // Green
  // ropepathColor.b = 0.0f;  // Blue
  // ropepathColor.a = 1.0f;  // Alpha (transparency)

  // publishPath(rov_path_pub, Path_replan, "world", "rov_path", ropepathColor);

  return Path_replan;
}

ompl::geometric::PathGeometric TetherPlanner::InvertTetherPath(
    const ompl::geometric::PathGeometric &tether,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {

  ompl::geometric::PathGeometric tether_inverse(
      si); // Reverse tether path concatenated with Path from base to Goal

  for (int i = tether.getStateCount() - 1; i >= 0; --i) {
    // Clone the state to ensure a new instance is created
    auto *state = tether.getSpaceInformation()->cloneState(tether.getState(i));
    tether_inverse.append(state);
  }

  return tether_inverse;
}

double
TetherPlanner::findTetherLength(const ompl::geometric::PathGeometric &path) {
  double length = 0.0;

  // Debugging print to check the number of states in the path
  // std::cout << "Number of states in path: " << path.getStateCount() <<
  // std::endl;

  // Iterate through the states in the path
  for (std::size_t i = 1; i < path.getStateCount(); ++i) {
    // Get the current and previous states
    const auto *state1 =
        path.getState(i - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *state2 =
        path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();

    // Convert the states to Eigen vectors
    Eigen::Vector3d p1(state1->values[0], state1->values[1], state1->values[2]);
    Eigen::Vector3d p2(state2->values[0], state2->values[1], state2->values[2]);

    // Calculate the distance between the states and add to the total length
    double segment_length = (p2 - p1).norm();
    length += segment_length;

    // Debugging prints
    // std::cout << "State " << i-1 << ": [" << p1[0] << ", p1[1] << ", " <<
    // p1[2] << "]" << std::endl;
    ////std::cout << "State " << i << ": [" << p2[0] << ", p2[1] << ", " <<
    ///p2[2] << "]" << std::endl;
    // std::cout << "Segment length: " << segment_length << std::endl;
  }

  // std::cout << "Total tether length: " << length << std::endl;
  return length;
}

double TetherPlanner::findTetherLengthReplannedPath(
    const ompl::geometric::PathGeometric &tether,
    const std::vector<double> &exit_point, const std::vector<double> &goal,
    const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  double length = 0.0;
  // find length till exitpoint
  for (std::size_t i = 1; i < tether.getStateCount(); ++i) {

    // Get the current and previous states
    const auto *state1 =
        tether.getState(i - 1)
            ->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *state2 =
        tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();

    if (!isEqual({state1->values[0], state1->values[1], state1->values[2]},
                 exit_point)) {

      // Convert the states to Eigen vectors
      Eigen::Vector3d p1(state1->values[0], state1->values[1],
                         state1->values[2]);
      Eigen::Vector3d p2(state2->values[0], state2->values[1],
                         state2->values[2]);

      // Calculate the distance between the states and add to the total length
      length += (p2 - p1).norm();
    } else {
      // ompl::geometric::PathGeometric Path_from_exit =
      // FindShortestPath(exit_point, goal, space, si);
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(
          space);
      state_goal->values[0] = goal[0]; // x-coordinate
      state_goal->values[1] = goal[1]; // y-coordinate
      state_goal->values[2] = goal[2]; // z-coordinate

      ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
          state_rov_scoped(space);
      state_rov_scoped->values[0] = current_pos_att[0]; // x-coordinate
      state_rov_scoped->values[1] = current_pos_att[1]; // y-coordinate
      state_rov_scoped->values[2] = current_pos_att[2]; // z-coordinate

      ompl::geometric::PathGeometric Path_from_exit =
          FindShortestPath(state_rov_scoped, state_goal, si);

      double L_from_exit = findTetherLength(Path_from_exit);
      length += L_from_exit;
      break;
    }
  }

  return length;
}

bool TetherPlanner::isEqual(const std::vector<double> &point1,
                            const std::vector<double> &point2) const {
  const double epsilon = 1e-3; // Tolerance for floating-point comparison
  return (std::abs(point1[0] - point2[0]) < epsilon &&
          std::abs(point1[1] - point2[1]) < epsilon &&
          std::abs(point1[2] - point2[2]) < epsilon);
}

ompl::geometric::PathGeometric TetherPlanner::FindShortestPath(
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &start,
    const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::SimpleSetup ss_rrt(si);
  ss_rrt.setStartAndGoalStates(start, goal);
  auto planner_rg = std::make_shared<ompl::geometric::RRTstar>(si);
  ss_rrt.setPlanner(planner_rg);

  // Solve the planning problem
  ompl::base::PlannerStatus solved_rrt =
      ss_rrt.solve(ompl::base::timedPlannerTerminationCondition(1.0));
  if (solved_rrt) {
    return ss_rrt.getSolutionPath();
  } else {
    // Return an empty path if the planning problem is not solved
    return ompl::geometric::PathGeometric(si);
  }
}

void TetherPlanner::UpdateExitPointsList(
    const ompl::geometric::PathGeometric &tether) {
  // Update the list of potential exit points for replanning
  exit_points_list_.clear();
  bool in_collision = false;

  for (int i = tether.getStateCount() - 1; i >= 0; --i) {
    if (!isStateValid(tether.getState(i))) {
      in_collision = true;
    } else if (in_collision) {
      // Add the first non-collision point after a collision to the exit points
      // list
      std::vector<double> exit_point(3);
      const auto *state =
          tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      exit_point[0] = state->values[0];
      exit_point[1] = state->values[1];
      exit_point[2] = state->values[2];
      exit_points_list_.push_back(exit_point);

      // Reset the collision flag to find the next segment
      in_collision = false;
    }
  }
}

std::vector<double> TetherPlanner::FindNextPointAlongPath(
    const std::vector<double> &current_position,
    const std::vector<double> &goal_last,
    const ompl::geometric::PathGeometric &path_replan) {
  bool found_goal_last = false;

  // Iterate through the path from the end to the beginning
  for (int i = path_replan.getStateCount() - 1; i >= 0; --i) {
    const auto *state = path_replan.getState(i)
                            ->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> point(3);
    point[0] = state->values[0];
    point[1] = state->values[1];
    point[2] = state->values[2];

    // Check if the current position is equal to the last goal point
    if (found_goal_last) {
      // Return the next point along the path
      return point;
    }

    // Check if the current point is equal to the last goal point
    if (isEqual(point, goal_last)) {
      found_goal_last = true;
    }
  }

  // If no next point is found, return the last goal point
  return goal_last;
}

ompl::geometric::PathGeometric TetherPlanner::ReplanPath(
    const ompl::geometric::PathGeometric &tether,
    const std::vector<double> &exit_point, const std::vector<double> &goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric replan_path(si);

  // Iterate through the tether from end to start
  for (int i = tether.getStateCount() - 1; i >= 0; --i) {
    const auto *state =
        tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> point(3);
    point[0] = state->values[0];
    point[1] = state->values[1];
    point[2] = state->values[2];

    // Add the state to the replan path
    replan_path.append(state);

    // Check if the current point is equal to the exit point
    if (isEqual(point, exit_point)) {
      break;
    }
  }

  // Create the shortest path from the exit point to the goal
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_exit(
      si->getStateSpace());
  state_exit->values[0] = exit_point[0];
  state_exit->values[1] = exit_point[1];
  state_exit->values[2] = exit_point[2];

  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(
      si->getStateSpace());
  state_goal->values[0] = goal[0];
  state_goal->values[1] = goal[1];
  state_goal->values[2] = goal[2];

  ompl::geometric::PathGeometric shortest_path =
      FindShortestPath(state_exit, state_goal, si);

  // Concatenate the replan path with the shortest path to the goal
  replan_path.append(shortest_path);

  ROS_INFO("Replanned Path:");
  for (std::size_t i = 0; i < replan_path.getStateCount(); ++i) {
    const auto *state = replan_path.getState(i)
                            ->as<ompl::base::RealVectorStateSpace::StateType>();
    ROS_INFO("State %zu: [%f, %f, %f]", i, state->values[0], state->values[1],
             state->values[2]);
  }

  return replan_path;
}

ompl::geometric::PathGeometric TetherPlanner::CalculateAlternativePath_i(
    const int node_n, ompl::geometric::PathGeometric tether,
    const std::vector<double> goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric Alternative_Path(si);
  ompl::geometric::PathGeometric Path_segment1(si);
  ompl::geometric::PathGeometric Path_segment3(si);

  // ompl::geometric::PathGeometric Path_segment2(si);

  // Calculate Path_segment1
  // ROS_INFO("Calculating Path_segment1...");
  for (int i = 0; i < node_n; i++) {
    const auto *state = tether.getState(tether.getStateCount() - i - 1)
                            ->as<ompl::base::RealVectorStateSpace::StateType>();
    Path_segment1.append(state);
    // ROS_INFO("Path_segment1 State %d: [x: %f, y: %f, z: %f]", i,
    // state->values[0], state->values[1], state->values[2]);
  }

  // Calculate Path_segment3 (the rest of the tether path)
  for (int i = tether.getStateCount() - node_n - 1; i >= 0; --i) {
    const auto *state =
        tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    Path_segment3.append(state);
  }



  // Calculate Path_segment2
  // ROS_INFO("Calculating Path_segment2...");
  auto *state_goal =
      si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
  state_goal->values[0] = goal[0]; // x-coordinate
  state_goal->values[1] = goal[1]; // y-coordinate
  state_goal->values[2] = goal[2]; // z-coordinate
  // ROS_INFO("Goal State: [x: %f, y: %f, z: %f]", state_goal->values[0],
  // state_goal->values[1], state_goal->values[2]);

  // Path_segment2 = tether;

  // ROS_INFO("Calculating Path_segment2...");

  /**/
  ompl::geometric::PathGeometric Path_segment13 = Path_segment1;
  Path_segment13.append(Path_segment3);
  ompl::geometric::PathGeometric Path_segment2 =
      computePathSegment2(Path_segment13, goal, si);

  // Concatenate Path_segment1 and Path_segment2
  // ROS_INFO("Concatenating Path_segment1 and Path_segment2...");
  Alternative_Path = Path_segment1;
  Alternative_Path.append(Path_segment2);
  std::vector<ompl::base::State *> contactPoints;
  ompl::geometric::PathSimplifier simplifier(si);

  simplifier.ropeRRTtether(Alternative_Path, contactPoints, delta_,
                           equivalenceTolerance_);

  Alternative_Path_Tether_Length =
      findTetherLength(Path_segment2) + findTetherLength(Path_segment3);

  // Print the states in the Alternative_Path
  // ROS_INFO("Alternative Path:");
  for (std::size_t i = 0; i < Alternative_Path.getStateCount(); ++i) {
    const auto *state = Alternative_Path.getState(i)
                            ->as<ompl::base::RealVectorStateSpace::StateType>();
    // ROS_INFO("State %zu: [x: %f, y: %f, z: %f]", i, state->values[0],
    // state->values[1], state->values[2]);
  }

  return Alternative_Path;
}


ompl::geometric::PathGeometric TetherPlanner::CalculateAlternativePath_iRRT(
  const int node_n, ompl::geometric::PathGeometric tether,
  const std::vector<double> goal,
  const std::shared_ptr<ompl::base::SpaceInformation> &si) {
ompl::geometric::PathGeometric Alternative_Path(si);
ompl::geometric::PathGeometric Path_segment1(si);
ompl::geometric::PathGeometric Path_segment3(si);

// ompl::geometric::PathGeometric Path_segment2(si);

// Calculate Path_segment1
// ROS_INFO("Calculating Path_segment1...");
for (int i = 0; i < node_n; i++) {
  const auto *state = tether.getState(tether.getStateCount() - i - 1)
                          ->as<ompl::base::RealVectorStateSpace::StateType>();
  Path_segment1.append(state);
  // ROS_INFO("Path_segment1 State %d: [x: %f, y: %f, z: %f]", i,
  // state->values[0], state->values[1], state->values[2]);
}

// Calculate Path_segment3 (the rest of the tether path)
for (int i = tether.getStateCount() - node_n - 1; i >= 0; --i) {
  const auto *state =
      tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
  Path_segment3.append(state);
}



// Calculate Path_segment2
// ROS_INFO("Calculating Path_segment2...");
auto *state_goal =
    si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
state_goal->values[0] = goal[0]; // x-coordinate
state_goal->values[1] = goal[1]; // y-coordinate
state_goal->values[2] = goal[2]; // z-coordinate
// ROS_INFO("Goal State: [x: %f, y: %f, z: %f]", state_goal->values[0],
// state_goal->values[1], state_goal->values[2]);

// Path_segment2 = tether;

// ROS_INFO("Calculating Path_segment2...");

/**/
ompl::geometric::PathGeometric Path_segment13 = Path_segment1;
Path_segment13.append(Path_segment3);
ompl::geometric::PathGeometric Path_segment2 =
    computePathSegment2(Path_segment13, goal, si);

// Concatenate Path_segment1 and Path_segment2
// ROS_INFO("Concatenating Path_segment1 and Path_segment2...");
Alternative_Path = Path_segment1;
Alternative_Path.append(Path_segment2);
std::vector<ompl::base::State *> contactPoints;
ompl::geometric::PathSimplifier simplifier(si);

simplifier.ropeRRTtether(Alternative_Path, contactPoints, delta_,
                         equivalenceTolerance_);

Alternative_Path_Tether_Length =
    findTetherLength(Path_segment2) + findTetherLength(Path_segment3);

// Print the states in the Alternative_Path
// ROS_INFO("Alternative Path:");
for (std::size_t i = 0; i < Alternative_Path.getStateCount(); ++i) {
  const auto *state = Alternative_Path.getState(i)
                          ->as<ompl::base::RealVectorStateSpace::StateType>();
  // ROS_INFO("State %zu: [x: %f, y: %f, z: %f]", i, state->values[0],
  // state->values[1], state->values[2]);
}

//return Alternative_Path;
return Path_segment1;

}



std::vector<double>
TetherPlanner::getNextGoal(const ompl::geometric::PathGeometric &path) {
  std::vector<double> next_goal(3);

  if (path.getStateCount() > 0) {
    const auto *state =
        path.getState(0)->as<ompl::base::RealVectorStateSpace::StateType>();
    next_goal[0] = state->values[0];
    next_goal[1] = state->values[1];
    next_goal[2] = state->values[2];
  } else {
    ROS_WARN("Path is empty. Returning default goal.");
    next_goal = {0.0, 0.0, 0.0}; // Default goal if path is empty
  }

  ROS_INFO("Next goal: [x: %f, y: %f, z: %f]", next_goal[0], next_goal[1],
           next_goal[2]);

  return next_goal;
}

ompl::geometric::PathGeometric TetherPlanner::SearchAlternativePath(
    ompl::geometric::PathGeometric tether, const std::vector<double> goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si, const double L_max)

{
  ompl::geometric::PathGeometric Alternative_Path(si);
  ompl::geometric::PathGeometric Alternative_Path_P1(si);

  ompl::geometric::PathGeometric Alternative_Path_1(si);
  bool short_cut = false;
 
  for (int i = 3; i < tether.getStateCount()-2; i++) {
    Alternative_Path = CalculateAlternativePath_i(i, tether, goal, si);
    Alternative_Path_1 = CalculateAlternativePath_i(i-3, tether, goal, si);
    Alternative_Path_P1 = CalculateAlternativePath_iRRT(i+2, tether, goal, si);



    double Alternative_Path_1_Length = findTetherLength(Alternative_Path_1);

    if (Alternative_Path_Tether_Length < (L_max * 0.7)) {
        // Set the exit point to the state at point i
        /*
        const auto *state_exit_ptr = tether.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> exit_point = {state_exit_ptr->values[0], state_exit_ptr->values[1], state_exit_ptr->values[2]};

        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_exit(si->getStateSpace());
        state_exit->values[0] = exit_point[0];
        state_exit->values[1] = exit_point[1];
        state_exit->values[2] = exit_point[2];

        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_goal(
            si->getStateSpace());
        state_goal->values[0] = goal[0];
        state_goal->values[1] = goal[1];
        state_goal->values[2] = goal[2];
  
       ompl::geometric::PathGeometric PathFromExitToGoal =
        FindShortestPath(state_exit, state_goal, si);
        Alternative_Path_P1.append(PathFromExitToGoal);
      */

        if (Alternative_Path_Tether_Length < Alternative_Path_1_Length*0.65 ) {
          if(i<3){
          Alternative_Path_P1 = CalculateAlternativePath_iRRT(tether.getStateCount()-1, tether, goal, si);
          }
      short_cut = true;
      break;
      }
    }
  }

  if (!short_cut) {
    ROS_WARN("No alternative path found within the tether length constraint. "
             "Adding goal point to the path.");
    const auto *end_state =
        tether.getState(tether.getStateCount() - 1)
            ->as<ompl::base::RealVectorStateSpace::StateType>();
    ompl::geometric::PathGeometric single_point_path(si);
    single_point_path.append(end_state);

    // Add the goal point to the path
    auto *state_goal =
        si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
    state_goal->values[0] = goal[0]; // x-coordinate
    state_goal->values[1] = goal[1]; // y-coordinate
    state_goal->values[2] = goal[2]; // z-coordinate
    single_point_path.append(state_goal);

    return single_point_path;
  }


  // Ensure Alternative_Path_P1 has at least 3 states
  if (Alternative_Path_P1.getStateCount() < 3) {
      Alternative_Path_P1 = CalculateAlternativePath_iRRT(tether.getStateCount()-1, tether, goal, si);
  }


  return Alternative_Path_P1;
}

ompl::geometric::PathGeometric TetherPlanner::computePathSegment2(
    const ompl::geometric::PathGeometric &Path_segment1,
    const std::vector<double> &goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si
    ) {
  ompl::geometric::PathGeometric Path_segment2 =
      InvertTetherPath(Path_segment1, si);

  auto *state_goal =
      si->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
  state_goal->values[0] = goal[0]; // x-coordinate
  state_goal->values[1] = goal[1]; // y-coordinate
  state_goal->values[2] = goal[2]; // z-coordinate

  // ROS_INFO("Goal State: [x: %f, y: %f, z: %f]", state_goal->values[0],
  // state_goal->values[1], state_goal->values[2]);

  Path_segment2.append(state_goal);

  ompl::geometric::PathSimplifier simplifier(si);
  simplifier.ropeShortcutPath(Path_segment2, delta_, equivalenceTolerance_);

  return Path_segment2;
}

std::vector<double>
TetherPlanner::computePerpendicularUnitVector(const std::vector<double> &v1,
                                              const std::vector<double> &v2) {
  // Compute the cross product of v1 and v2
  std::vector<double> cross_v1_v2 = crossProduct(v1, v2);

  // Compute the cross product of v1 and cross_v1_v2
  std::vector<double> perpendicular_vector = crossProduct(v1, cross_v1_v2);

  // Normalize the resulting vector to make it a unit vector
  std::vector<double> unit_vector = normalize(perpendicular_vector);

  return unit_vector;
}

std::vector<double> TetherPlanner::MoveGoalToSafeZone(
    const std::vector<double> &node_n1, const std::vector<double> &node_n2,
    const std::vector<double> &node_n3, double delta_safe,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  std::vector<double> new_goal;
  bool found_valid_goal = false;

  // Compute the vectors (n1 - n2) and (n3 - n2)
  std::vector<double> v1 = {node_n1[0] - node_n2[0], node_n1[1] - node_n2[1],
                            node_n1[2] - node_n2[2]};
  std::vector<double> v2 = {node_n3[0] - node_n2[0], node_n3[1] - node_n2[1],
                            node_n3[2] - node_n2[2]};

  // Compute the unit vector that is perpendicular to (n1 - n2) and also
  // perpendicular to the cross product of the two vectors
  std::vector<double> unit_vector = computePerpendicularUnitVector(v1, v2);

  // Move the goal by delta_safe in the direction of the computed unit vector
  new_goal = {node_n2[0] + delta_safe * unit_vector[0],
              node_n2[1] + delta_safe * unit_vector[1],
              node_n2[2] + delta_safe * unit_vector[2]};

  // Check if the new goal is valid and at least delta_safe away from obstacles
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_new_goal(
      si->getStateSpace());
  state_new_goal->values[0] = new_goal[0];
  state_new_goal->values[1] = new_goal[1];
  state_new_goal->values[2] = new_goal[2];

  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_node_n2(
      si->getStateSpace());
  state_node_n2->values[0] = node_n2[0];
  state_node_n2->values[1] = node_n2[1];
  state_node_n2->values[2] = node_n2[2];

  if (si->isValid(state_new_goal.get()) &&
      si->distance(state_new_goal.get(), state_node_n2.get()) >= delta_safe) {
    found_valid_goal = true;
  } else {
    // Try random directions until a valid state is found
    while (!found_valid_goal) {
      std::vector<double> direction = SearchRandomDirection();
      new_goal = {node_n2[0] + delta_safe * direction[0],
                  node_n2[1] + delta_safe * direction[1],
                  node_n2[2] + delta_safe * direction[2]};

      state_new_goal->values[0] = new_goal[0];
      state_new_goal->values[1] = new_goal[1];
      state_new_goal->values[2] = new_goal[2];

      if (si->isValid(state_new_goal.get()) &&
          si->distance(state_new_goal.get(), state_node_n2.get()) >=
              delta_safe) {
        found_valid_goal = true;
      }
    }
  }

  return new_goal;
}

/*
std::vector<double> TetherPlanner::MoveGoalToSafeZone(const std::vector<double>
&node_n1, const std::vector<double> &node_n2, const std::vector<double>
&node_n3, double delta_safe)
{
// Compute the vectors (n1 - n2) and (n3 - n2)
std::vector<double> v1 = {node_n1[0] - node_n2[0], node_n1[1] - node_n2[1],
node_n1[2] - node_n2[2]}; std::vector<double> v2 = {node_n3[0] - node_n2[0],
node_n3[1] - node_n2[1], node_n3[2] - node_n2[2]};

// Compute the unit vector that is perpendicular to (n1 - n2) and also
perpendicular to the cross product of the two vectors std::vector<double>
unit_vector = computePerpendicularUnitVector(v1, v2);

// Move the goal by delta_safe in the direction of the computed unit vector
std::vector<double> new_goal = {node_n2[0] + delta_safe * unit_vector[0],
 node_n2[1] + delta_safe * unit_vector[1],
 node_n2[2] + delta_safe * unit_vector[2]};

return new_goal;
}
*/

std::vector<double> TetherPlanner::GetNextPointAlongPath(
    const ompl::geometric::PathGeometric &path,
    const std::vector<double> current_position, const std::vector<double> goal,
    const std::shared_ptr<ompl::base::SpaceInformation> &si) {

  std::vector<double> next_goal = {0.0, 0.0, 0.0};
  std::vector<double> node_n1 = {0.0, 0.0, 0.0};
  std::vector<double> node_n2 = {0.0, 0.0, 0.0};
  std::vector<double> node_n3 = {0.0, 0.0, 0.0};
  int n = 0;
  double delta_safe = 1.0;

  if (path.getStateCount() > 10) {
    const auto *state =
        path.getState(n)->as<ompl::base::RealVectorStateSpace::StateType>();

    next_goal[0] = state->values[0];
    next_goal[1] = state->values[1];
    next_goal[2] = state->values[2];
  } else {
    next_goal = goal;
  }

  // Check if next_goal is valid
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_next_goal(
      si->getStateSpace());
  state_next_goal->values[0] = next_goal[0];
  state_next_goal->values[1] = next_goal[1];
  state_next_goal->values[2] = next_goal[2];

  if (!si->isValid(state_next_goal.get())) {
    ROS_WARN(
        "Next goal is not valid. Perturbing goal state to find a valid state.");

    const auto *state_n1 =
        path.getState(n)->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *state_n2 =
        path.getState(n + 1)->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto *state_n3 =
        path.getState(n + 2)->as<ompl::base::RealVectorStateSpace::StateType>();

    node_n1[0] = state_n1->values[0];
    node_n1[1] = state_n1->values[1];
    node_n1[2] = state_n1->values[2];

    node_n2[0] = state_n2->values[0];
    node_n2[1] = state_n2->values[1];
    node_n2[2] = state_n2->values[2];

    node_n3[0] = state_n3->values[0];
    node_n3[1] = state_n3->values[1];
    node_n3[2] = state_n3->values[2];

    // Move the goal to a safe zone
    //next_goal = MoveGoalToSafeZone(node_n1, node_n2, node_n3, delta_safe, si);

    // Update state_next_goal with the new next_goal
    state_next_goal->values[0] = next_goal[0];
    state_next_goal->values[1] = next_goal[1];
    state_next_goal->values[2] = next_goal[2];
  }

  return next_goal;
}

ompl::geometric::PathGeometric TetherPlanner::OffsetPath(
    const ompl::geometric::PathGeometric &path,
    const std::shared_ptr<ompl::base::SpaceInformation> &si,
    double delta_safe) {
  ompl::geometric::PathGeometric offset_path(si);
  std::vector<double> direction;

  // Check if the path length is greater than 10
  if (path.getStateCount() <= 10) {
    ROS_WARN(
        "Path length is not greater than 10. Returning the original path.");
    return path;
  }

  // Find a valid direction for the first point
  bool found_valid_direction = false;
  while (!found_valid_direction) {
    direction = SearchRandomDirection();
    const auto *state =
        path.getState(0)->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> new_node = {
        state->values[0] + delta_safe * direction[0],
        state->values[1] + delta_safe * direction[1],
        state->values[2] + delta_safe * direction[2]};

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_new(
        si->getStateSpace());
    state_new->values[0] = new_node[0];
    state_new->values[1] = new_node[1];
    state_new->values[2] = new_node[2];

    if (si->isValid(state_new.get())) {
      found_valid_direction = true;
    }
  }

  // Apply the valid direction to the first 10 points in the path
  for (std::size_t i = 0; i < 10; ++i) {
    const auto *state =
        path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> node = {state->values[0], state->values[1],
                                state->values[2]};
    std::vector<double> new_node = {node[0] + delta_safe * direction[0],
                                    node[1] + delta_safe * direction[1],
                                    node[2] + delta_safe * direction[2]};

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_new(
        si->getStateSpace());
    state_new->values[0] = new_node[0];
    state_new->values[1] = new_node[1];
    state_new->values[2] = new_node[2];

    if (!si->isValid(state_new.get())) {
      // If the direction is not valid for any point, find a new valid direction
      found_valid_direction = false;
      while (!found_valid_direction) {
        direction = SearchRandomDirection();
        new_node = {node[0] + delta_safe * direction[0],
                    node[1] + delta_safe * direction[1],
                    node[2] + delta_safe * direction[2]};

        state_new->values[0] = new_node[0];
        state_new->values[1] = new_node[1];
        state_new->values[2] = new_node[2];

        if (si->isValid(state_new.get())) {
          found_valid_direction = true;
        }
      }
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_offset(
        si->getStateSpace());
    state_offset->values[0] = new_node[0];
    state_offset->values[1] = new_node[1];
    state_offset->values[2] = new_node[2];
    offset_path.append(
        state_offset.get()); // Use .get() to get the underlying state pointer
  }

  // Append the remaining points without modification
  for (std::size_t i = 10; i < path.getStateCount(); ++i) {
    const auto *state =
        path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_offset(
        si->getStateSpace());
    state_offset->values[0] = state->values[0];
    state_offset->values[1] = state->values[1];
    state_offset->values[2] = state->values[2];
    offset_path.append(state_offset.get());
  }

  return offset_path;
}

// stl model offset
ompl::geometric::PathGeometric TetherPlanner::OffSetTetherPath_STL(
    const ompl::geometric::PathGeometric &path,
    const std::vector<Triangle> &inspection_model_stl, double offset_distance) {
  ompl::geometric::PathGeometric offset_path(path.getSpaceInformation());

  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    const auto *state =
        path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::Vector3d node(state->values[0], state->values[1], state->values[2]);

    // Find the closest triangle
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector3f closest_normal;

    for (const auto &triangle : inspection_model_stl) {
      Eigen::Vector3f v1 = triangle.vertex1;
      Eigen::Vector3f v2 = triangle.vertex2;
      Eigen::Vector3f v3 = triangle.vertex3;
      Eigen::Vector3f p = node.cast<float>();

      // Compute the distance from the node to the triangle
      Eigen::Vector3f v0 = v1 - p;
      Eigen::Vector3f v1v2 = v2 - v1;
      Eigen::Vector3f v1v3 = v3 - v1;
      Eigen::Vector3f n = v1v2.cross(v1v3).normalized();
      double distance = std::abs(v0.dot(n));

      if (distance < min_distance) {
        min_distance = distance;
        closest_normal = triangle.normal;
      }
    }

    // Offset the node in the direction of the closest triangle's normal
    Eigen::Vector3d offset_node =
        node + offset_distance * closest_normal.cast<double>();

    // Create a new state and set its values
    auto *offset_state =
        path.getSpaceInformation()
            ->getStateSpace()
            ->as<ompl::base::RealVectorStateSpace>()
            ->allocState()
            ->as<ompl::base::RealVectorStateSpace::StateType>();
    offset_state->values[0] = offset_node.x();
    offset_state->values[1] = offset_node.y();
    offset_state->values[2] = offset_node.z();

    // Add the offset state to the new path
    offset_path.append(offset_state);
  }

  return offset_path;
}



ompl::geometric::PathGeometric TetherPlanner::PopPath(
  const ompl::geometric::PathGeometric &path, const nvblox::EsdfLayer &esdf_layer, 
  double step_size) {
  ompl::geometric::PathGeometric new_path(path.getSpaceInformation());

  // Determine the number of points to process
  std::size_t num_points_to_process = path.getStateCount();
  //num_points_to_process = 2;
  // Apply gradient adjustment to the first 10 points or the entire path if it has fewer than 10 points
  for (std::size_t i = 0; i < num_points_to_process; ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      Eigen::Vector3f point(state->values[0], state->values[1], state->values[2]);

      // Get the ESDF gradient at the current point
      Eigen::Vector3f closest_center = findClosestTriangleCenter(point, inspection_model_stl);

      Eigen::Vector3f gradient = getESDFGradient(esdf_layer, closest_center) * 1.0 ;
      ROS_ERROR("ESDF Gradient at state %zu: [%f, %f, %f]", i, gradient.x(), gradient.y(), gradient.z());
      

     // publishVector(point, gradient/10.0, vector_pub, "world", i);
     publishVector(closest_center, -gradient, vector_pub, "world", i);

      // Move the point in the direction of the gradient
      Eigen::Vector3f new_point = point - step_size * gradient;

      // Create a new state and set its values
      auto *new_state = path.getSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = new_point.x();
      new_state->values[1] = new_point.y();
      new_state->values[2] = new_point.z();

      // Add the new state to the new path
      new_path.append(new_state);
  }

  // Append the remaining points without modification if the path has more than 10 points
  for (std::size_t i = num_points_to_process; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      auto *new_state = path.getSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = state->values[0];
      new_state->values[1] = state->values[1];
      new_state->values[2] = state->values[2];

      // Add the new state to the new path
      new_path.append(new_state);
  }

  return new_path;
}

ompl::geometric::PathGeometric TetherPlanner::PopPathWithNormals(const ompl::geometric::PathGeometric &path, const std::vector<Triangle> &stl_model, double step_size) {
  ompl::geometric::PathGeometric new_path(path.getSpaceInformation());

  // Check if the path is empty or has fewer than 3 points
  if (path.getStateCount() < 3) {
      ROS_WARN("Path is empty or has fewer than 3 points. Returning the original path.");
      return path;
  }

  // Determine the number of points to process (all except the first two and the last one)
  std::size_t num_points_to_process = path.getStateCount() - 1;

  // Append the first two states without modification
  for (std::size_t i = 0; i < 2; ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      auto *new_state = path.getSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = state->values[0];
      new_state->values[1] = state->values[1];
      new_state->values[2] = state->values[2];
      new_path.append(new_state);
  }

  // Apply normal vector adjustment to all points except the first two and the last one
  for (std::size_t i = 2; i < num_points_to_process-1; ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      Eigen::Vector3f point(state->values[0], state->values[1], state->values[2]);

      // Find the closest normal vector at the current point
      Eigen::Vector3f closest_center = findClosestTriangleCenter(point, stl_model);

      Eigen::Vector3f normal = - findClosestNormal(point, stl_model);
      ROS_ERROR("Closest Normal at state %zu: [%f, %f, %f]", i, normal.x(), normal.y(), normal.z());
      publishVector(closest_center, -normal, vector_pub, "world", i);

      // Move the point in the direction of the normal vector
      Eigen::Vector3f new_point = point + step_size * normal;

      // Create a new state and set its values
      auto *new_state = path.getSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = new_point.x();
      new_state->values[1] = new_point.y();
      new_state->values[2] = new_point.z();

      // Add the new state to the new path
      new_path.append(new_state);
  }

  // Append the last state without modification
  const auto *last_state = path.getState(path.getStateCount() - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
  auto *new_last_state = path.getSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
  new_last_state->values[0] = last_state->values[0];
  new_last_state->values[1] = last_state->values[1];
  new_last_state->values[2] = last_state->values[2];
  new_path.append(new_last_state);

  return new_path;
}






// Function to find the closest waypoint from the first point along the path to the waypoints list
std::vector<double> 
TetherPlanner::findClosestWaypointToPath(const ompl::geometric::PathGeometric &path, 
  const std::vector<std::vector<double>> &way_point_traj,
   int i, std::vector<double> last_waypoint) {


  if (path.getStateCount() == 0 || way_point_traj.empty()) {
      ROS_WARN("Path or way_point_traj is empty. Returning default waypoint.");
      return {0.0, 0.0, 0.0}; // Default waypoint if path or way_point_traj is empty
  }

  // Get the first point along the path
  const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> first_point = {state->values[0], state->values[1], state->values[2]};

  // Initialize the minimum distance and closest waypoint
  double min_distance = std::numeric_limits<double>::max();
  std::vector<double> closest_waypoint;

  // Iterate through the waypoints to find the closest one
  for (const auto &waypoint : way_point_traj) {

    if (waypoint == last_waypoint) {
      continue;
  }

      double distance = calculateDistance(first_point, waypoint);
      if (distance < min_distance) {
          min_distance = distance;
          closest_waypoint = waypoint;
      }
  }

  return closest_waypoint;
}






ompl::geometric::PathGeometric TetherPlanner::PopPathWaypoint(
  const ompl::geometric::PathGeometric &path,
  const std::vector<std::vector<double>> &way_point_traj,
  const std::shared_ptr<ompl::base::SpaceInformation> &si,
  double step_size) {

  ompl::geometric::PathGeometric new_path(si);

  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      std::vector<double> current_point = {state->values[0], state->values[1], state->values[2]};

      // Find the closest waypoint
      std::vector<double> closest_waypoint = findClosestWaypointToPath(path, way_point_traj, i, current_point);

      // Calculate the direction vector towards the closest waypoint
      Eigen::Vector3d direction(closest_waypoint[0] - current_point[0],
                                closest_waypoint[1] - current_point[1],
                                closest_waypoint[2] - current_point[2]);
      direction.normalize();

      // Move the point in the direction of the closest waypoint
      Eigen::Vector3d new_point(current_point[0] + step_size * direction[0],
                                current_point[1] + step_size * direction[1],
                                current_point[2] + step_size * direction[2]);

      // Check if the new point is valid
      ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state_new(si->getStateSpace());
      state_new->values[0] = new_point.x();
      state_new->values[1] = new_point.y();
      state_new->values[2] = new_point.z();

      if (si->isValid(state_new.get())) {
          // Create a new state and set its values
          auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
          new_state->values[0] = new_point.x();
          new_state->values[1] = new_point.y();
          new_state->values[2] = new_point.z();

          // Add the new state to the new path
          new_path.append(new_state);
      } else {
          // If the new point is not valid, keep the original point
          new_path.append(state);
      }
  }

  return new_path;
}


std::vector<double> TetherPlanner::getPointAlongPath(
  const ompl::geometric::PathGeometric &path, int i) {
std::vector<double> point(3, 0.0);

if (i >= 0 && i < path.getStateCount()) {
  const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
  point[0] = state->values[0];
  point[1] = state->values[1];
  point[2] = state->values[2];
} else {
  ROS_WARN("Index %d is out of bounds. Returning default point [0, 0, 0].", i);
}

return point;
}



ompl::geometric::PathGeometric TetherPlanner::smoothPath(const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric smoothed_path(si);

  if (path.getStateCount() < 3) {
      ROS_WARN("Path is too short to smooth. Returning the original path.");
      return path;
  }

  // Add the first point
  smoothed_path.append(path.getState(0));

  // Smooth the path by averaging points
  for (std::size_t i = 1; i < path.getStateCount() - 1; ++i) {
      const auto *prev_state = path.getState(i - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
      const auto *curr_state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      const auto *next_state = path.getState(i + 1)->as<ompl::base::RealVectorStateSpace::StateType>();

      Eigen::Vector3d prev_point(prev_state->values[0], prev_state->values[1], prev_state->values[2]);
      Eigen::Vector3d curr_point(curr_state->values[0], curr_state->values[1], curr_state->values[2]);
      Eigen::Vector3d next_point(next_state->values[0], next_state->values[1], next_state->values[2]);

      Eigen::Vector3d smoothed_point = (prev_point + curr_point + next_point) / 3.0;

      // Create a new state and set its values
      auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = smoothed_point.x();
      new_state->values[1] = smoothed_point.y();
      new_state->values[2] = smoothed_point.z();

      // Add the new state to the smoothed path
      smoothed_path.append(new_state);
  }

  // Add the last point
  smoothed_path.append(path.getState(path.getStateCount() - 1));

  return smoothed_path;
}


ompl::geometric::PathGeometric TetherPlanner::smoothPathWithPolynomial(const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric smoothed_path(si);

  if (path.getStateCount() < 4) {
      // ROS_WARN("Path is too short to smooth with polynomial fitting. Returning the original path.");
      return path;
  }

  // Extract the points from the path
  std::vector<double> x, y, z;
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      x.push_back(state->values[0]);
      y.push_back(state->values[1]);
      z.push_back(state->values[2]);
  }

  // Fit a 3rd order polynomial to the points (excluding the first and last points)
  Eigen::VectorXd coeffs_x = fitPolynomial(std::vector<double>(x.begin() + 1, x.end() - 1), 3);
  Eigen::VectorXd coeffs_y = fitPolynomial(std::vector<double>(y.begin() + 1, y.end() - 1), 3);
  Eigen::VectorXd coeffs_z = fitPolynomial(std::vector<double>(z.begin() + 1, z.end() - 1), 3);

  // Add the first point to the smoothed path
  smoothed_path.append(path.getState(0));

  // Generate the smoothed path for the points in between
  for (std::size_t i = 1; i < path.getStateCount() - 1; ++i) {
      double t = static_cast<double>(i - 1) / (path.getStateCount() - 3);
      double smoothed_x = evaluatePolynomial(coeffs_x, t);
      double smoothed_y = evaluatePolynomial(coeffs_y, t);
      double smoothed_z = evaluatePolynomial(coeffs_z, t);

      // Create a new state and set its values
      auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = smoothed_x;
      new_state->values[1] = smoothed_y;
      new_state->values[2] = smoothed_z;

      // Add the new state to the smoothed path
      smoothed_path.append(new_state);
  }

  // Add the last point to the smoothed path
  smoothed_path.append(path.getState(path.getStateCount() - 1));

  return smoothed_path;
}



Eigen::VectorXd TetherPlanner::fitPolynomial5thOrder(const std::vector<double> &points) {
  int n = points.size();
  Eigen::MatrixXd A(n, 6); // 5th order polynomial has 6 coefficients
  Eigen::VectorXd b(n);

  for (int i = 0; i < n; ++i) {
      double t = static_cast<double>(i) / (n - 1);
      for (int j = 0; j <= 5; ++j) {
          A(i, j) = std::pow(t, j);
      }
      b(i) = points[i];
  }

  // Solve for the polynomial coefficients
  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
  return coeffs;
}


ompl::geometric::PathGeometric TetherPlanner::smoothPathWith5thOrderPolynomial(const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric smoothed_path(si);

  if (path.getStateCount() < 6) {
      ROS_WARN("Path is too short to smooth with 5th order polynomial fitting. Returning the original path.");
      return path;
  }

  // Define the segment length
  const std::size_t segment_length = 6;

  // Iterate through the path in segments
  for (std::size_t start = 0; start < path.getStateCount(); start += segment_length - 1) {
      std::size_t end = std::min(start + segment_length, path.getStateCount());

      // Extract the points from the segment
      std::vector<double> x, y, z;
      for (std::size_t i = start; i < end; ++i) {
          const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
          x.push_back(state->values[0]);
          y.push_back(state->values[1]);
          z.push_back(state->values[2]);
      }

      // Fit a 5th order polynomial to the points
      Eigen::VectorXd coeffs_x = fitPolynomial5thOrder(x);
      Eigen::VectorXd coeffs_y = fitPolynomial5thOrder(y);
      Eigen::VectorXd coeffs_z = fitPolynomial5thOrder(z);

      // Generate the smoothed segment
      for (std::size_t i = 0; i < x.size(); ++i) {
          double t = static_cast<double>(i) / (x.size() - 1);
          double smoothed_x = evaluatePolynomial(coeffs_x, t);
          double smoothed_y = evaluatePolynomial(coeffs_y, t);
          double smoothed_z = evaluatePolynomial(coeffs_z, t);

          // Create a new state and set its values
          auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
          new_state->values[0] = smoothed_x;
          new_state->values[1] = smoothed_y;
          new_state->values[2] = smoothed_z;

          // Add the new state to the smoothed path
          smoothed_path.append(new_state);
      }

      // Ensure the last point of the segment is included
      if (end < path.getStateCount()) {
          const auto *state = path.getState(end - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
          auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
          new_state->values[0] = state->values[0];
          new_state->values[1] = state->values[1];
          new_state->values[2] = state->values[2];
          smoothed_path.append(new_state);
      }
  }

  return smoothed_path;
}



ompl::geometric::PathGeometric TetherPlanner::smoothPathWithPolynomial2(
  const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric smoothed_path(si);

  if (path.getStateCount() < 4) {
      //ROS_WARN("Path is too short to smooth with polynomial fitting. Returning the original path.");
      return path;
  }

  // Define the segment length
  const std::size_t segment_length = 4;

  // Iterate through the path in segments
  for (std::size_t start = 0; start < path.getStateCount(); start += segment_length - 1) {
      std::size_t end = std::min(start + segment_length, path.getStateCount());

      // Extract the points from the segment
      std::vector<double> x, y, z;
      for (std::size_t i = start; i < end; ++i) {
          const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
          x.push_back(state->values[0]);
          y.push_back(state->values[1]);
          z.push_back(state->values[2]);
      }

      // Fit a 3rd order polynomial to the points
      Eigen::VectorXd coeffs_x = fitPolynomial(x, 3);
      Eigen::VectorXd coeffs_y = fitPolynomial(y, 3);
      Eigen::VectorXd coeffs_z = fitPolynomial(z, 3);

      // Generate the smoothed segment
      for (std::size_t i = 0; i < x.size(); ++i) {
          double t = static_cast<double>(i) / (x.size() - 1);
          double smoothed_x = evaluatePolynomial(coeffs_x, t);
          double smoothed_y = evaluatePolynomial(coeffs_y, t);
          double smoothed_z = evaluatePolynomial(coeffs_z, t);

          // Create a new state and set its values
          auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
          new_state->values[0] = smoothed_x;
          new_state->values[1] = smoothed_y;
          new_state->values[2] = smoothed_z;

          // Add the new state to the smoothed path
          smoothed_path.append(new_state);
      }

      // Ensure the last point of the segment is included
      if (end < path.getStateCount()) {
          const auto *state = path.getState(end - 1)->as<ompl::base::RealVectorStateSpace::StateType>();
          auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
          new_state->values[0] = state->values[0];
          new_state->values[1] = state->values[1];
          new_state->values[2] = state->values[2];
          smoothed_path.append(new_state);
      }
  }

  return smoothed_path;
}


Eigen::VectorXd TetherPlanner::fitPolynomial(const std::vector<double> &points, int order) {
    int n = points.size();
    Eigen::MatrixXd A(n, order + 1);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1);
        for (int j = 0; j <= order; ++j) {
            A(i, j) = std::pow(t, j);
        }
        b(i) = points[i];
    }

    // Solve for the polynomial coefficients
    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
    return coeffs;
}

double TetherPlanner::evaluatePolynomial(const Eigen::VectorXd &coeffs, double t) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * std::pow(t, i);
    }
    return result;
}




bool TetherPlanner::checkPath(const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i);
      if (!si->isValid(state)) {
          ROS_WARN("Collision detected at state %zu", i);
          return false;
      }
  }
  ROS_INFO("Path is collision-free");
  return true;
}





ompl::geometric::PathGeometric TetherPlanner::smoothPathWithSpline(const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) {
  ompl::geometric::PathGeometric smoothed_path(si);

  if (path.getStateCount() < 4) {
      ROS_WARN("Path is too short to smooth with spline fitting. Returning the original path.");
      return path;
  }

  // Extract the points from the path into Eigen matrices, excluding the first and last points
  Eigen::MatrixXd points(path.getStateCount() - 2, 3);
  for (std::size_t i = 1; i < path.getStateCount() - 1; ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      points(i - 1, 0) = state->values[0];
      points(i - 1, 1) = state->values[1];
      points(i - 1, 2) = state->values[2];
  }

  // Fit a cubic spline to the points
  Eigen::Spline3d spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points.transpose(), 3);

  // Add the first point to the smoothed path
  smoothed_path.append(path.getState(0));

  // Generate the smoothed path for the points in between
  for (std::size_t i = 1; i < path.getStateCount() - 1; ++i) {
      double t = static_cast<double>(i - 1) / (path.getStateCount() - 3);
      Eigen::Vector3d smoothed_point = spline(t);

      // Create a new state and set its values
      auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
      new_state->values[0] = smoothed_point.x();
      new_state->values[1] = smoothed_point.y();
      new_state->values[2] = smoothed_point.z();

      // Add the new state to the smoothed path
      smoothed_path.append(new_state);
  }

  // Add the last point to the smoothed path
  smoothed_path.append(path.getState(path.getStateCount() - 1));

  return smoothed_path;
}



void TetherPlanner::PopPathFirstPoint(
  ompl::geometric::PathGeometric &path,
  const std::vector<double> &current_position,
  const std::shared_ptr<ompl::base::SpaceInformation> &si,
  double delta) {

  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      Eigen::Vector3d point(state->values[0], state->values[1], state->values[2]);

      // Check if the state is valid
      if (!si->isValid(state)) {
          // Move the point towards the current position by delta
          Eigen::Vector3d current_pos(current_position[0], current_position[1], current_position[2]);
          Eigen::Vector3d direction = (current_pos - point).normalized();
          Eigen::Vector3d new_point = point + delta * direction;

          // Update the state with the new values
          state->values[0] = new_point.x();
          state->values[1] = new_point.y();
          state->values[2] = new_point.z();

          // Exit the loop after popping the first invalid point
          break;
      }
  }
}



void TetherPlanner::PopPathSample(
  ompl::geometric::PathGeometric &path,
  const std::shared_ptr<ompl::base::SpaceInformation> &si,
  double delta) {

  // Ensure the path has at least 3 points to avoid issues with the first and last points
  if (path.getStateCount() < 3) {
      ROS_WARN("Path is too short to pop. Returning the original path.");
      return;
  }

  for (std::size_t i = 1; i < path.getStateCount() - 1; ++i) { // Start from 1 and end at getStateCount() - 1
      auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      Eigen::Vector3d point(state->values[0], state->values[1], state->values[2]);

      bool valid_state_found = false;
      Eigen::Vector3d new_point;

      // Sample directions
      for (int j = 0; j < 100; ++j) { // Try 100 random directions
          Eigen::Vector3d direction = Eigen::Vector3d::Random().normalized();
          new_point = point + delta * direction;

          // Check if the new point is valid
          auto *new_state = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->allocState()->as<ompl::base::RealVectorStateSpace::StateType>();
          new_state->values[0] = new_point.x();
          new_state->values[1] = new_point.y();
          new_state->values[2] = new_point.z();

          if (si->isValid(new_state)) {
              valid_state_found = true;
              break;
          }
      }

      if (valid_state_found) {
          // Update the state with the new values
          state->values[0] = new_point.x();
          state->values[1] = new_point.y();
          state->values[2] = new_point.z();
      }
  }
}



std::vector<double> TetherPlanner::calculatePathCentroid(const ompl::geometric::PathGeometric &path) {
  std::vector<double> centroid(3, 0.0);

  if (path.getStateCount() == 0) {
      ROS_WARN("Path is empty. Returning default centroid [0, 0, 0].");
      return centroid;
  }

  // Sum the coordinates of all states in the path
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      centroid[0] += state->values[0];
      centroid[1] += state->values[1];
      centroid[2] += state->values[2];
  }

  // Divide by the number of states to get the average coordinates
  centroid[0] /= path.getStateCount();
  centroid[1] /= path.getStateCount();
  centroid[2] /= path.getStateCount();

  return centroid;
}




void TetherPlanner::PopTetherCentroid(ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si, double delta) {
  // Calculate the centroid of the path
  std::vector<double> centroid = calculatePathCentroid(path);

  // Iterate through the states in the path
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      Eigen::Vector3d point(state->values[0], state->values[1], state->values[2]);
      Eigen::Vector3d centroid_point(centroid[0], centroid[1], centroid[2]);

      // Calculate the direction vector from the centroid to the point
      Eigen::Vector3d direction = (point - centroid_point).normalized();

      // Move the point away from the centroid by delta
      Eigen::Vector3d new_point = point + delta * direction;

      // Update the state with the new values
      state->values[0] = new_point.x();
      state->values[1] = new_point.y();
      state->values[2] = new_point.z();
  }
}
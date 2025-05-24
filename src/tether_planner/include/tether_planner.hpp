#ifndef TETHER_PLANNER_HPP
#define TETHER_PLANNER_HPP

#include "global_vars.hpp"
#include "helper_functions.hpp"
#include <Eigen/Dense>
#include <memory>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <vector>
#include <unsupported/Eigen/Splines>

class TetherPlanner {

public:
  // TetherPlanner( ); // Constructor declaration
  TetherPlanner(double delta,
                double equivalenceTolerance); // Constructor declaration



  std::vector<std::vector<double>>
      exit_points_list_; // list of potential exit points for replanning

  double delta_;                // Step size
  double equivalenceTolerance_; // Equivalence tolerance
  double Alternative_Path_Tether_Length = 100000;

  double Direct_Path_Tether_Length = 100000;

  ompl::geometric::PathGeometric
  findNextGoal(const ompl::geometric::PathGeometric &tether,
               const std::vector<double> &current_position,
               const std::vector<double> &goal, double &L_max,
               const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
               const std::shared_ptr<ompl::base::SpaceInformation> &si);

  ompl::geometric::PathGeometric
  ReplanPath(const ompl::geometric::PathGeometric &tether,
             const std::vector<double> &current_position,
             const std::vector<double> &goal, double &L_max,
             const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
             const std::shared_ptr<ompl::base::SpaceInformation> &si);

  ompl::geometric::PathGeometric
  DirectPath(const std::vector<double> &current_position,
             const std::vector<double> &goal,
             const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
             const std::shared_ptr<ompl::base::SpaceInformation> &si);

  ompl::geometric::PathGeometric
  InvertTetherPath(const ompl::geometric::PathGeometric &tether,
                   const std::shared_ptr<ompl::base::SpaceInformation> &si);
  double findTetherLength(const ompl::geometric::PathGeometric &path);

  bool isEqual(const std::vector<double> &point1,
               const std::vector<double> &point2) const;

  void UpdateExitPointsList(const ompl::geometric::PathGeometric &tether);

  ompl::geometric::PathGeometric FindShortestPath(
      const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &start,
      const ompl::base::ScopedState<ompl::base::RealVectorStateSpace> &goal,
      const std::shared_ptr<ompl::base::SpaceInformation> &si);

  double findTetherLengthReplannedPath(
      const ompl::geometric::PathGeometric &tether,
      const std::vector<double> &exit_point, const std::vector<double> &goal,
      const std::shared_ptr<ompl::base::RealVectorStateSpace> &space,
      const std::shared_ptr<ompl::base::SpaceInformation> &si);

  std::vector<double>
  FindNextPointAlongPath(const std::vector<double> &current_position,
                         const std::vector<double> &goal_last,
                         const ompl::geometric::PathGeometric &path_replan);

  ompl::geometric::PathGeometric
  ReplanPath(const ompl::geometric::PathGeometric &tether,
             const std::vector<double> &exit_point,
             const std::vector<double> &goal,
             const std::shared_ptr<ompl::base::SpaceInformation> &si);

  ompl::geometric::PathGeometric CalculateAlternativePath_i(
      const int node_n, ompl::geometric::PathGeometric tether,
      const std::vector<double> goal,
      const std::shared_ptr<ompl::base::SpaceInformation> &si);

  ompl::geometric::PathGeometric
  SearchAlternativePath(ompl::geometric::PathGeometric tether,
                        const std::vector<double> goal,
                        const std::shared_ptr<ompl::base::SpaceInformation> &si,
                        const double L_max);

  std::vector<double> getNextGoal(const ompl::geometric::PathGeometric &path);

  ompl::geometric::PathGeometric
  computePathSegment2(const ompl::geometric::PathGeometric &Path_segment1,
                      const std::vector<double> &goal,
                      const std::shared_ptr<ompl::base::SpaceInformation> &si);

  std::vector<double> GetNextPointAlongPath(
      const ompl::geometric::PathGeometric &path,
      const std::vector<double> current_position,
      const std::vector<double> goal,
      const std::shared_ptr<ompl::base::SpaceInformation> &si);

  std::vector<double>
  MoveGoalToSafeZone(const std::vector<double> &node_n1,
                     const std::vector<double> &node_n2,
                     const std::vector<double> &node_n3, double delta_safe,
                     const std::shared_ptr<ompl::base::SpaceInformation> &si);

  std::vector<double>
  computePerpendicularUnitVector(const std::vector<double> &v1,
                                 const std::vector<double> &v2);

  std::vector<double> SearchRandomDirection();

  ompl::geometric::PathGeometric
  OffsetPath(const ompl::geometric::PathGeometric &path,
             const std::shared_ptr<ompl::base::SpaceInformation> &si,
             double delta_safe);


  ompl::geometric::PathGeometric OffSetTetherPath_STL(const ompl::geometric::PathGeometric &path,
                const std::vector<Triangle> &inspection_model_stl,
                double offset_distance);


                ompl::geometric::PathGeometric PopPathWithNormals(const ompl::geometric::PathGeometric &path, 
                    const std::vector<Triangle> &stl_model, double step_size);



                    ompl::geometric::PathGeometric PopPath(const ompl::geometric::PathGeometric &path, const nvblox::EsdfLayer &esdf_layer, double step_size);



                    std::vector<double> 
                    findClosestWaypointToPath(const ompl::geometric::PathGeometric &path, 
                      const std::vector<std::vector<double>> &way_point_traj,
                       int i,  std::vector<double> last_waypoint);




    ompl::geometric::PathGeometric PopPathWaypoint(
    const ompl::geometric::PathGeometric &path,
    const std::vector<std::vector<double>> &way_point_traj,
    const std::shared_ptr<ompl::base::SpaceInformation> &si,
    double step_size);


    std::vector<double> getPointAlongPath(
        const ompl::geometric::PathGeometric &path, int i);


ompl::geometric::PathGeometric smoothPath(const ompl::geometric::PathGeometric &path,
     const std::shared_ptr<ompl::base::SpaceInformation> &si);


     Eigen::VectorXd fitPolynomial(const std::vector<double> &points, int order);
     double evaluatePolynomial(const Eigen::VectorXd &coeffs, double t);    
     ompl::geometric::PathGeometric smoothPathWithPolynomial(
        const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si);



        bool checkPath(const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si);


        ompl::geometric::PathGeometric smoothPathWithSpline(
            const ompl::geometric::PathGeometric &path, 
            const std::shared_ptr<ompl::base::SpaceInformation> &si);


            void PopPathFirstPoint(
                ompl::geometric::PathGeometric &path,
                const std::vector<double> &current_position,
                const std::shared_ptr<ompl::base::SpaceInformation> &si,
                double delta);


                void PopPathSample(
                    ompl::geometric::PathGeometric &path,
                    const std::shared_ptr<ompl::base::SpaceInformation> &si,
                    double delta);


                    ompl::geometric::PathGeometric smoothPathWithPolynomial2(
                        const ompl::geometric::PathGeometric &path, const std::shared_ptr<ompl::base::SpaceInformation> &si) ;

                        Eigen::VectorXd fitPolynomial5thOrder(const std::vector<double> &points);
                        ompl::geometric::PathGeometric smoothPathWith5thOrderPolynomial(const ompl::geometric::PathGeometric &path, 
                            const std::shared_ptr<ompl::base::SpaceInformation> &si);

                            ompl::geometric::PathGeometric CalculateAlternativePath_iRRT(
                                const int node_n, ompl::geometric::PathGeometric tether,
                                const std::vector<double> goal,
                                const std::shared_ptr<ompl::base::SpaceInformation> &si);



        std::vector<double> calculatePathCentroid(
            const ompl::geometric::PathGeometric &path);


            void PopTetherCentroid(ompl::geometric::PathGeometric &path, 
                const std::shared_ptr<ompl::base::SpaceInformation> &si, double delta);


                };







#endif
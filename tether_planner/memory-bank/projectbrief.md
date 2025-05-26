# Project Brief: Tether Planner

## Core Objective

The `tether_planner` project implements a motion planning system for a tethered underwater Remotely Operated Vehicle (ROV). Its primary goal is to generate safe, collision-free paths for the ROV while actively managing the physical constraints imposed by the tether, specifically its length and configuration.

## Problem Solved

Standard motion planners often don't account for the complexities of a physical tether connecting an ROV to a surface vessel or entry point. Tethers can snag, become taut, or collide with the environment, restricting the ROV's movement or even causing mission failure. This project addresses this by:

1.  **Modeling the Tether:** Explicitly calculating the tether's shape based on the ROV's path and environmental contact points.
2.  **Constraint Checking:** Ensuring the calculated tether configuration does not violate length limits (`L_max`).
3.  **Collision Avoidance:** Checking for collisions for both the ROV and the tether against the environment (represented by point clouds/maps).
4.  **Reactive Planning:** If the primary path (e.g., following waypoints) leads to tether constraint violations, the planner computes an alternative, safer path to reposition the ROV and manage the tether before proceeding.

## Key Functionality

-   ROS node integrating with ROV odometry and potentially other sensors.
-   Utilizes OMPL for core motion planning algorithms.
-   Uses PCL and potentially `nvblox` for environment modeling and collision checking.
-   Implements custom tether modeling logic (e.g., `ropeRRTtether`).
-   Publishes planned paths, tether configurations, and ROV status.
-   Handles waypoint navigation and dynamic replanning based on tether state.

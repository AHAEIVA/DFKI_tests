# System Patterns: Tether Planner

## Architecture Overview

*   High-level diagram or description of the system components (ROS nodes, libraries, data flow).
*   How do the main parts interact? (e.g., Planner Node -> OMPL -> Collision Checker -> Environment Model -> ROS Publishers/Subscribers).

## Key Technical Decisions

*   Why were specific libraries chosen (OMPL, PCL, nvblox)?
*   Significant algorithms used (e.g., RRT*, specific tether modeling approach).
*   Data structures for environment representation (Point Cloud, ESDF).

## Design Patterns

*   Are there specific software design patterns employed (e.g., Strategy, Observer)?
*   Code organization principles (e.g., separation of concerns between planning, collision checking, ROS interface).

## Component Relationships

*   Detailed interaction between specific C++ classes or modules.
*   Dependency graph (if complex).

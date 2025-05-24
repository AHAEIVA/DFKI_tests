# Technical Context: Tether Planner

## Core Technologies

*   **Programming Language:** C++
*   **Build System:** CMake (within ROS Catkin environment)
*   **Operating System:** Linux (Ubuntu assumed, typical for ROS)
*   **Frameworks/Middleware:** ROS (Robot Operating System) - specific version? (e.g., Noetic, Melodic)

## Key Libraries & Dependencies

*   **Motion Planning:** OMPL (Open Motion Planning Library)
*   **Point Cloud Processing:** PCL (Point Cloud Library)
*   **Mapping/ESDF:** nvblox (GPU-accelerated 3D reconstruction) - *Verify if actively used*
*   **ROS Packages:** `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf`, etc.
*   **Math:** Eigen

## Development Setup

*   How is the development environment configured? (e.g., Docker, native Ubuntu install)
*   Required tools (compilers, specific library versions).
*   IDE/Editor preferences (VS Code detected).

## Technical Constraints

*   Hardware limitations (e.g., ROV compute power, sensor capabilities).
*   Real-time performance requirements?
*   Specific ROS version compatibility.

## Deployment

*   How is the planner deployed onto the target system (ROV or simulation)?

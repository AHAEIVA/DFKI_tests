cmake_minimum_required(VERSION 2.8.3)

# Set policy CMP0048 to NEW to allow VERSION to be specified
cmake_policy(SET CMP0048 NEW)

# Set the project version
project(ea_mpc VERSION 1.0.0)

# Default C++14 setup
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(EXECUTABLE_OUTPUT_PATH /home/hakim/tether_planning_ws/src/ea_mpc)


# Find catkin packages and other dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  nav_msgs
  tf
)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Specify paths for ACADO toolkit
set(ACADO_INCLUDE_DIR /home/hakim/tether_planning_ws/src/ea_mpc/externals/ACADOtoolkit/acado)
set(ACADO_ENV_SHARED_LIBRARIES /home/hakim/tether_planning_ws/src/ea_mpc/externals/ACADOtoolkit/build/lib/libacado_toolkit_s.so)

# Include ACADO headers
include_directories(${ACADO_INCLUDE_DIR})
link_directories(${ACADO_ENV_SHARED_LIBRARIES})

# Define the MPC solver library
add_library(mpc_solver
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/nmpc_qpoases_interface.cpp
  solver/nmpc_integrator.c
  solver/nmpc_solver.c
  solver/nmpc_auxiliary_functions.c
)

target_include_directories(mpc_solver PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/solver
  ${PROJECT_SOURCE_DIR}/solver/qpoases
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE
  ${PROJECT_SOURCE_DIR}/solver/qpoases/SRC
)

# Generate messages
add_message_files(
  FILES
  PwmCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/solver
  ${PROJECT_SOURCE_DIR}/solver/qpoases
)

# Code generation executable
add_executable(code_gen_brov_ea_mpc src/ocp/bluerov_ea_mpc.cpp)


target_include_directories(code_gen_brov_ea_mpc PUBLIC
${PROJECT_SOURCE_DIR}/code_gen
${PROJECT_SOURCE_DIR}/externals/ACADOtoolkit/acado/code_generation
${PROJECT_SOURCE_DIR}/externals/ACADOtoolkit/acado
${PROJECT_SOURCE_DIR}/externals/ACADOtoolkit/

)


target_link_libraries(code_gen_brov_ea_mpc ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver   ${catkin_LIBRARIES})


set_target_properties(code_gen_brov_ea_mpc PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY /home/hakim/tether_planning_ws/src/ea_mpc
)

# Main EA MPC executable
add_executable(ea_mpc src/ea_mpc_main.cpp src/ea_mpc.cpp)
target_link_libraries(ea_mpc mpc_solver ${ACADO_ENV_SHARED_LIBRARIES} ${catkin_LIBRARIES})

# Install targets
install(TARGETS
  code_gen_brov_ea_mpc
  ea_mpc
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
catkin_install_python(PROGRAMS scripts/thrust_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

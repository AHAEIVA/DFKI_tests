#pragma once 
#include "range.hpp"
#include "global_vars.hpp"

#include "nvblox/core/types.h"
#include "nvblox/mapper/mapper.h"
//#include <nvblox/serialization/layer_serializer_gpu.h>
//#include "nvblox/serialization/mesh_serializer_gpu.h"

class CollisionChecker
{
public:
    nvblox::Mapper& mapper;
    Eigen::AlignedBox3f bb;
    double step;
    double collision_threshold; // Add collision_threshold as a member variable

    CollisionChecker(nvblox::Mapper& input_mapper, double collision_threshold); // Modify constructor to accept collision_threshold
public:
bool isCollisionFree(const Eigen::Vector3d& point, double eps); // Add eps parameter to the method declaration
Eigen::Vector3d getRandomPoint();
    Eigen::Vector3d getRandomPointFree();
};
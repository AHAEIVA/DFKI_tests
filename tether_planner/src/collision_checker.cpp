#include "collision_checker.hpp"
#include <nvblox/mesh/mesh.h>
#include <nvblox/nvblox.h>
#include <random>

CollisionChecker::CollisionChecker(nvblox::Mapper& input_mapper, double collision_threshold)
    : mapper(input_mapper)
    , bb(nvblox::getAABBOfAllocatedBlocks(mapper.tsdf_layer())) // Use TSDF layer
    , step(mapper.voxel_size_m() * 3.0)
    , collision_threshold(collision_threshold) // Initialize collision_threshold
{
}

bool CollisionChecker::isCollisionFree(const Eigen::Vector3d& point, double eps)
{
    auto r = mapper.tsdf_layer().getVoxel(point.cast<float>());
    Eigen::Vector3f bb_min = bb.min();
    Eigen::Vector3f bb_max = bb.max();

    max_point = bb_max;
    min_point = bb_min;
    //std::cout << "bb_max!!!! "  << bb_max <<std::endl;
    //std::cout << "bb_min!!!! "  << bb_min <<std::endl;

    bool is_present = r.second;
    if (!is_present)
    {
        //std::cout << "Out of bounds "  << std::endl;
        return true;
    }
    nvblox::TsdfVoxel voxel = r.first;

    // Output the distance
    std::cout << "Distance at point (" << point.transpose() << "): " << voxel.distance << std::endl;

    return voxel.distance > eps; // Check the distance in the TSDF voxel
}

Eigen::Vector3d CollisionChecker::getRandomPoint()
{
    return bb.sample().cast<double>();
}

Eigen::Vector3d CollisionChecker::getRandomPointFree()
{
    Eigen::Vector3d point = getRandomPoint();
    while (!isCollisionFree(point, collision_threshold))
    {
        point = getRandomPoint();
    }
    return point;
}
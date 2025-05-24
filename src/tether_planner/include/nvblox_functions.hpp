#ifndef NVBLOX_FUNCTIONS_HPP
#define NVBLOX_FUNCTIONS_HPP

#include "nvblox/core/types.h"
#include "nvblox/mapper/mapper.h"
#include <nvblox/integrators/esdf_slicer.h>
#include <nvblox/mesh/mesh.h>
#include <nvblox/map/unified_3d_grid.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>
#include "range.hpp"
#include "nvblox/primitives/scene.h"
//#include "nvblox/io/layer_cake_io.h"
//#include "nvblox/nvblox.h"
#include "nvblox/serialization/layer_serializer_gpu.h"
#include "nvblox/map/layer.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <nvblox/io/mesh_io.h>  // For saving mesh (optional, if you want to visualize)
//#include "nvblox/io/layer_cake_io.h"

// Function to expand an Eigen::AlignedBox3f by a given factor
void expandAlignedBox3f(Eigen::AlignedBox3f& box, float factor);

// Function to create a map from EIVA data
std::unique_ptr<nvblox::Mapper> mapFromEIVA();


std::unique_ptr<nvblox::Mapper> mapFromPipe(double scale_factor, Eigen::Matrix3f rotation, Eigen::Vector3f translation);
// Function to read vectors from a file
std::vector<Eigen::Vector3f> readVectorsFromFile(const std::string& filename);

// Function to create a map from a saved .nvblx file
std::unique_ptr<nvblox::Mapper> mapFromCake(std::string savePath);

// Function to create a map from primitives
std::unique_ptr<nvblox::Mapper> mapFromPrimitives();

std::unique_ptr<nvblox::Mapper> mapFromPointCloud(const std::vector<Eigen::Vector3f>& points);
std::vector<Eigen::Vector3f> pclToEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void saveEsdfLayer(const std::unique_ptr<nvblox::Mapper>& mapper, const std::string& file_path) ;

#endif // NVBLOX_FUNCTIONS_HPP

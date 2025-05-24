#include "nvblox_functions.hpp"

using namespace std::chrono_literals;

void expandAlignedBox3f(Eigen::AlignedBox3f& box, float factor) {
    // Calculate the center of the box
    Eigen::Vector3f center = box.center();

    // Calculate the new size by scaling the current size
    Eigen::Vector3f newSize = box.sizes() * factor;

    // Calculate the new min and max corners
    Eigen::Vector3f newMin = center - newSize / 2.0f;
    Eigen::Vector3f newMax = center + newSize / 2.0f;

    // Update the box with the new corners
    box = Eigen::AlignedBox3f(newMin, newMax);
}






std::unique_ptr<nvblox::Mapper> mapFromEIVA()
{
    std::string path = "D:\\workspace\\TestingUtils\\data\\EivaLogo.xyz";
    std::string pathCahce = "D:\\fun\\Eivamap.nvblx";
    if (std::filesystem::exists(pathCahce))
    {
        return mapFromCake(pathCahce);
    }

    std::vector<Eigen::Vector3f> points = readVectorsFromFile(path);
    double voxel_size_m = 0.007;
    std::unique_ptr<nvblox::Mapper> mapper = std::make_unique<nvblox::Mapper>(voxel_size_m, nvblox::MemoryType::kDevice);
    nvblox::primitives::Scene scene;
    // Create a map that's a box with a sphere in the middle.
    scene.aabb() = nvblox::AxisAlignedBoundingBox(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    int i = 0;
    
    for (const auto& p : points)
    {
        scene.aabb().extend(p);
        if (i % 10 == 0)
        {
            scene.addPrimitive(std::make_unique<nvblox::primitives::Sphere>(p, 0.2f));
        }
        i++;
        
    }
    expandAlignedBox3f(scene.aabb(),1.5f);
    // Get the ground truth TSDF from this scene (otherwise integrate individual frames).
    // We need to create a temp layer unfortunately.
    nvblox::TsdfLayer gt_tsdf(voxel_size_m, nvblox::MemoryType::kHost);
    scene.generateLayerFromScene(4 * voxel_size_m, &gt_tsdf);
    mapper->tsdf_layer() = std::move(gt_tsdf);
    // Set the max computed distance to 5 meters.
    mapper->esdf_integrator().max_esdf_distance_m(5.0f);
    // Generate the ESDF from everything in the TSDF.
    mapper->updateEsdf(nvblox::UpdateFullLayer::kYes);
    mapper->mesh_integrator().integrateMeshFromDistanceField(mapper->tsdf_layer(), &mapper->mesh_layer());
    mapper->saveLayerCake(pathCahce);
    return mapper;
}


std::unique_ptr<nvblox::Mapper> mapFromPipe(double scale_factor, Eigen::Matrix3f rotation, Eigen::Vector3f translation) {
    std::string path = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.xyz";
    std::string pathCache = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.nvblx";  // New cache path

    // Check if the cache exists
    if (std::filesystem::exists(pathCache)) {
        return mapFromCake(pathCache);  // Load map from cache if available
    }

    // Read point cloud from file
    std::vector<Eigen::Vector3f> points = readVectorsFromFile(path);

    // Print the size of the points vector
    std::cout << "Number of points in the point cloud: " << points.size() << std::endl;

    // Apply the transformation (rotation and translation) to each point
    for (auto& point : points) {
        point = scale_factor * (rotation * point + translation);
    }

    // Further increase voxel size to reduce map size even more
    double voxel_size_m = 0.08;  // Larger voxel size to drastically reduce the map size

    // Create a Mapper object with reduced memory usage
    std::unique_ptr<nvblox::Mapper> mapper = std::make_unique<nvblox::Mapper>(voxel_size_m, nvblox::MemoryType::kDevice);

    // Initialize scene and set the bounding box to be dynamic
    nvblox::primitives::Scene scene;
    if (points.empty()) {
        std::cerr << "Error: No points in the point cloud" << std::endl;
        return nullptr;  // Early exit if no points
    }

    // Set the bounding box to tightly enclose the transformed point cloud
    scene.aabb() = nvblox::AxisAlignedBoundingBox(points[0], points[0]);
    for (const auto& p : points) {
        scene.aabb().extend(p);  // Dynamically expand the bounding box
    }

    // Print the updated bounding box min and max values
    Eigen::Vector3f bbox_min = scene.aabb().min();
    Eigen::Vector3f bbox_max = scene.aabb().max();
    std::cout << "Updated Bounding box min: " << bbox_min.transpose() << std::endl;
    std::cout << "Updated Bounding box max: " << bbox_max.transpose() << std::endl;

    // Add primitives to the scene
    for (const auto& point : points) {
        scene.addPrimitive(std::make_unique<nvblox::primitives::Sphere>(point, 0.5f));  // Larger spheres for more coverage
    }

    // Generate TSDF layer with a smaller truncation distance to save memory
    nvblox::TsdfLayer gt_tsdf(voxel_size_m, nvblox::MemoryType::kHost);
    scene.generateLayerFromScene(2 * voxel_size_m, &gt_tsdf);  // Use a smaller truncation distance
    mapper->tsdf_layer() = std::move(gt_tsdf);

    // Compute ESDF and Mesh with a reduced range
    mapper->esdf_integrator().max_esdf_distance_m(0.1f);  // Use a smaller ESDF distance to reduce memory usage

    // Compute ESDF and Mesh
    mapper->updateEsdf(nvblox::UpdateFullLayer::kYes);
    mapper->mesh_integrator().integrateMeshFromDistanceField(
        mapper->tsdf_layer(), &mapper->mesh_layer()
    );

    // Save the generated map with a new minimal cache path
    mapper->saveLayerCake(pathCache);

    // Return the created mapper object
    return mapper;
}






std::vector<Eigen::Vector3f> readVectorsFromFile(const std::string& filename) {
    std::vector<Eigen::Vector3f> vectors;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return vectors;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z;
        // Only read 3 values (x, y, z) from each line
        if (!(iss >> x >> y >> z)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        vectors.emplace_back(-x, -y, -z);  // Store as negative values
    }

    file.close();
    return vectors;
}


std::unique_ptr<nvblox::Mapper> mapFromCake(std::string savePath)
{
    std::unique_ptr<nvblox::Mapper> mapper = std::make_unique<nvblox::Mapper>(savePath, nvblox::MemoryType::kDevice);
    return mapper;
}

std::unique_ptr<nvblox::Mapper> mapFromPrimitives()
{
    double voxel_size_m = 0.5;
    std::unique_ptr<nvblox::Mapper> mapper = std::make_unique<nvblox::Mapper>(voxel_size_m, nvblox::MemoryType::kDevice);
    nvblox::primitives::Scene scene;
    // Create a map that's a box with a sphere in the middle.
    scene.aabb() = nvblox::AxisAlignedBoundingBox(Eigen::Vector3f(-10.0f, -10.0f, -10.0f),Eigen::Vector3f(10.0f, 10.0f, 10.0f));
    
    scene.addPrimitive(std::make_unique<nvblox::primitives::Cube>(Eigen::Vector3f(0.0f, 0.0f, 2.0f), Eigen::Vector3f(2.0f, 2.0f, 5.0f)));
    scene.addPrimitive(std::make_unique<nvblox::primitives::Sphere>(Eigen::Vector3f(0.0f, 0.0f, 2.0f), 2.0f));
    // Get the ground truth TSDF from this scene (otherwise integrate individual frames).
    // We need to create a temp layer unfortunately.
    nvblox::TsdfLayer gt_tsdf(voxel_size_m, nvblox::MemoryType::kHost);
    scene.generateLayerFromScene(4 * voxel_size_m, &gt_tsdf);
    mapper->tsdf_layer() = std::move(gt_tsdf);
    // Set the max computed distance to 5 meters.
    mapper->esdf_integrator().max_esdf_distance_m(5.0f);
    // Generate the ESDF from everything in the TSDF.
    mapper->updateEsdf(nvblox::UpdateFullLayer::kYes);
    mapper->mesh_integrator().integrateMeshFromDistanceField(mapper->tsdf_layer(), &mapper->mesh_layer());
    return mapper;
}




// Function to convert PointCloud to a vector of Eigen::Vector3f
std::vector<Eigen::Vector3f> pclToEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<Eigen::Vector3f> points;
    for (const auto& point : cloud->points) {
        points.emplace_back(point.x, point.y, point.z);
    }
    return points;
}

std::unique_ptr<nvblox::Mapper> mapFromPointCloud(const std::vector<Eigen::Vector3f>& points)
{
    double voxel_size_m = 0.1;  // Adjust the voxel size as needed
    std::unique_ptr<nvblox::Mapper> mapper = std::make_unique<nvblox::Mapper>(voxel_size_m, nvblox::MemoryType::kDevice);

    // Initialize the ESDF integrator
    nvblox::EsdfIntegrator esdf_integrator;

    // Create the required layers
    nvblox::TsdfLayer tsdf_layer(voxel_size_m, nvblox::MemoryType::kDevice);
    nvblox::EsdfLayer esdf_layer(voxel_size_m, nvblox::MemoryType::kDevice);

    // Loop through each point in the point cloud
    for (const auto& eigen_point : points)
    {
        // Create a vector with the point (needed by integrateSlice)
        std::vector<Eigen::Matrix<int, 3, 1>> voxel_positions;
        voxel_positions.push_back(eigen_point.cast<int>()); // Convert to integer coordinates for voxel grid

        // Use the TSDF layer with integrateSlice
        esdf_integrator.integrateSlice(tsdf_layer, voxel_positions, &esdf_layer);
    }

    // Set the ESDF layer in the mapper
    mapper->esdf_layer() = std::move(esdf_layer);

    // Generate the ESDF
    mapper->updateEsdf(nvblox::UpdateFullLayer::kYes);
    std::string save_file_path = "/home/hakim/tether_planning_ws/src/tether_planner/input_data/dfki_pipe.nvblx";
   // mapper->saveLayerCake(save_file_path);
    return mapper;
}


// void saveEsdfLayer(const std::unique_ptr<nvblox::Mapper>& mapper, const std::string& file_path) {
//     // Assuming 'getLayerCake' is the correct method to access the LayerCake
//     auto layer_cake = mapper->getLayerCake(); // Adjust this as needed
//     nvblox::io::writeLayerCakeToFile(file_path, layer_cake);
// }



Eigen::Vector3f getESDFGradient(const nvblox::EsdfLayer& esdf_layer, 
    const Eigen::Vector3f& position) {
    float voxel_size = 0.2f;
    float delta = 0.5f * voxel_size;
    Eigen::Vector3f gradient = Eigen::Vector3f::Zero();
    
    auto get_distance = [&](const Eigen::Vector3f& pos, float& dist) -> bool {
        auto voxel_result = esdf_layer.getVoxel(pos);
        if (voxel_result.second && voxel_result.first.observed) {
            dist = std::sqrt(voxel_result.first.squared_distance_vox) * voxel_size;
            return true;
        }
        return false;
    };
    
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3f pos_offset = position;
        Eigen::Vector3f neg_offset = position;
        pos_offset[i] += delta;
        neg_offset[i] -= delta;
        
        float pos_dist = 0.0f, neg_dist = 0.0f;
        bool pos_valid = get_distance(pos_offset, pos_dist);
        bool neg_valid = get_distance(neg_offset, neg_dist);
        
        if (pos_valid && neg_valid) {
            gradient[i] = (pos_dist - neg_dist) / (2.0f * delta);
        } else if (pos_valid) {
            gradient[i] = (pos_dist - 0.0f) / delta;
        } else if (neg_valid) {
            gradient[i] = (0.0f - neg_dist) / delta;
        }
    }
    
    return gradient;
}
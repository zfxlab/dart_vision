#include "dart_lidar_localization/io/model_loader.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <filesystem>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <stdexcept>
#include <vector>

namespace dart_vision::lidar::localization {

std::string ModelLoader::resolvePath(const std::string& resource_path) {
    constexpr char package_prefix[] = "package://";
    if (resource_path.rfind(package_prefix, 0U) != 0U) {
        return std::filesystem::absolute(std::filesystem::path(resource_path)).string();
    }

    const std::string remainder = resource_path.substr(sizeof(package_prefix) - 1U);
    const auto separator = remainder.find('/');
    if (separator == std::string::npos || separator == 0U || separator + 1U >= remainder.size()) {
        throw std::invalid_argument("Invalid package resource path: " + resource_path);
    }
    const std::string package_name = remainder.substr(0U, separator);
    const std::string relative_path = remainder.substr(separator + 1U);
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) /
            relative_path)
        .string();
}

PointCloud::Ptr ModelLoader::load(const std::string& resource_path) {
    if (resource_path.empty()) {
        throw std::invalid_argument("model_path must not be empty");
    }
    const std::string path = resolvePath(resource_path);
    std::string extension = std::filesystem::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });

    PointCloud::Ptr cloud(new PointCloud);
    int result = -1;
    if (extension == ".pcd") {
        result = pcl::io::loadPCDFile<PointT>(path, *cloud);
    } else if (extension == ".ply") {
        result = pcl::io::loadPLYFile<PointT>(path, *cloud);
    } else {
        throw std::invalid_argument("Model must be a .pcd or .ply file: " + path);
    }
    if (result < 0) {
        throw std::runtime_error("Failed to load registration model: " + path);
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, valid_indices);
    if (cloud->empty()) {
        throw std::runtime_error("Registration model contains no finite points: " + path);
    }
    return cloud;
}

} // namespace dart_vision::lidar::localization

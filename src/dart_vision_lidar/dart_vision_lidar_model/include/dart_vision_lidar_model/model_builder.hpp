#ifndef DART_VISION_LIDAR_MODEL_MODEL_BUILDER_HPP
#define DART_VISION_LIDAR_MODEL_MODEL_BUILDER_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "dart_vision_lidar_model/model_sampler.hpp"

namespace dart_vision::lidar {

struct ModelBuildEntry {
    std::string name;
    std::filesystem::path input_mesh;
    std::filesystem::path output_pcd;
    ModelSamplingOptions sampling;
    bool overwrite{false};
};

struct ModelBuildConfig {
    std::filesystem::path config_path;
    std::filesystem::path config_directory;
    std::vector<ModelBuildEntry> models;
};

struct ModelBuildRecord {
    std::string name;
    std::filesystem::path output_pcd;
    ModelSamplingStats stats;
};

struct ModelBuildReport {
    std::vector<ModelBuildRecord> models;
    std::size_t built_count{0U};
};

/**
 * Loads and validates an ordinary (non-ROS-parameter) YAML file. Relative mesh
 * and PCD paths are resolved against the directory containing the YAML file.
 * Throws std::invalid_argument or std::runtime_error on invalid input.
 */
ModelBuildConfig loadModelBuildConfig(const std::filesystem::path& config_path);

/**
 * Builds every entry. Output directories are created as needed. Throws on the
 * first failed model.
 */
ModelBuildReport buildModels(const ModelBuildConfig& config);

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_MODEL_MODEL_BUILDER_HPP

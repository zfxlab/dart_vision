#include "dart_lidar_localization/calibration/result_writer.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <yaml-cpp/yaml.h>

namespace dart_vision::lidar::localization {
namespace {

Eigen::Vector3d rpyFromTransform(const Eigen::Isometry3d& transform) {
    const Eigen::Vector3d ypr = transform.linear().eulerAngles(2, 1, 0);
    return Eigen::Vector3d(ypr.z(), ypr.y(), ypr.x());
}

void emitVector3(YAML::Emitter& output, const Eigen::Vector3d& value) {
    output << YAML::Flow << YAML::BeginSeq << value.x() << value.y() << value.z() << YAML::EndSeq;
}

void emitTransform(YAML::Emitter& output, const Eigen::Isometry3d& transform) {
    output << YAML::BeginMap << YAML::Key << "xyz_m" << YAML::Value;
    emitVector3(output, transform.translation());
    output << YAML::Key << "rpy_rad" << YAML::Value;
    emitVector3(output, rpyFromTransform(transform));
    output << YAML::EndMap;
}

template <typename LevelT, typename AccessorT>
void emitLevelValues(YAML::Emitter& output, const std::vector<LevelT>& levels, AccessorT accessor) {
    output << YAML::Flow << YAML::BeginSeq;
    for (const auto& level : levels) {
        output << accessor(level);
    }
    output << YAML::EndSeq;
}

bool validFilenamePart(const std::string& value) {
    if (value.empty()) {
        return false;
    }
    for (const char character : value) {
        const bool valid =
            (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') || character == '-' || character == '_';
        if (!valid) {
            return false;
        }
    }
    return true;
}

std::filesystem::path chooseOutputPath(const std::filesystem::path& directory,
                                       const std::string& stem) {
    std::filesystem::path candidate = directory / (stem + ".yaml");
    for (std::size_t suffix = 1U; std::filesystem::exists(candidate); ++suffix) {
        candidate = directory / (stem + "_" + std::to_string(suffix) + ".yaml");
    }
    return candidate;
}

const char* dofName(const DofMode mode) {
    switch (mode) {
        case DofMode::kSixDof:
            return "six_dof";
        case DofMode::kXyzYaw:
            return "xyzyaw";
        case DofMode::kXyYaw:
            return "xyyaw";
    }
    return "unknown";
}

} // namespace

std::string ResultWriter::write(const std::string& directory,
                                const std::string& filename_prefix,
                                const LocalizationRecord& record) {
    if (directory.empty() || !validFilenamePart(filename_prefix) ||
        !validFilenamePart(record.profile) || record.filename_timestamp.empty()) {
        throw std::invalid_argument("invalid localization output path or filename part");
    }
    const std::filesystem::path output_directory =
        std::filesystem::absolute(std::filesystem::path(directory));
    std::filesystem::create_directories(output_directory);
    const auto output_path =
        chooseOutputPath(output_directory,
                         filename_prefix + "_" + record.profile + "_" + record.filename_timestamp +
                             "_m" + std::to_string(record.measurement_id));

    YAML::Emitter output;
    output.SetFloatPrecision(9);
    output.SetDoublePrecision(15);
    output << YAML::BeginMap;
    output << YAML::Key << "generated_at" << YAML::Value << record.generated_at;
    output << YAML::Key << "profile" << YAML::Value << record.profile;
    output << YAML::Key << "confirmed" << YAML::Value << record.confirmed;
    output << YAML::Key << "measurement_id" << YAML::Value << record.measurement_id;
    output << YAML::Key << "source" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "stamp_sec" << YAML::Value << record.source_stamp_sec;
    output << YAML::Key << "stamp_nanosec" << YAML::Value << record.source_stamp_nanosec;
    output << YAML::Key << "frame_id" << YAML::Value << record.source_frame;
    output << YAML::Key << "bag_name" << YAML::Value << record.bag_name;
    output << YAML::EndMap;
    output << YAML::Key << "frames" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "reference" << YAML::Value << record.reference_frame;
    output << YAML::Key << "base" << YAML::Value << record.base_frame;
    output << YAML::EndMap;
    output << YAML::Key << "models" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "base_path" << YAML::Value << record.base_model_path;
    output << YAML::EndMap;

    output << YAML::Key << "base" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "available" << YAML::Value << record.result.success();
    output << YAML::Key << "status_code" << YAML::Value << static_cast<int>(record.result.status);
    output << YAML::Key << "message" << YAML::Value << record.result.message;
    output << YAML::Key << "six_dof" << YAML::Value;
    emitTransform(output, record.result.observation_from_model);
    output << YAML::Key << "rmse_m" << YAML::Value << record.result.metrics.rmse_m;
    output << YAML::Key << "overlap_ratio" << YAML::Value << record.result.metrics.overlap_ratio;
    output << YAML::Key << "correspondence_count" << YAML::Value
           << record.result.metrics.correspondence_count;
    output << YAML::Key << "translation_from_initial_m" << YAML::Value
           << record.result.metrics.translation_from_initial_m;
    output << YAML::Key << "yaw_from_initial_rad" << YAML::Value
           << record.result.metrics.yaw_from_initial_rad;
    output << YAML::Key << "iterations" << YAML::Value << record.result.total_iterations;
    output << YAML::Key << "elapsed_ms" << YAML::Value << record.result.elapsed_ms;
    output << YAML::EndMap;

    output << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "base" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "coarse_enabled" << YAML::Value << record.base_parameters.coarse_enabled;
    output << YAML::Key << "dof" << YAML::Value << dofName(record.base_parameters.dof_mode);
    output << YAML::Key << "max_total_time_ms" << YAML::Value
           << record.base_parameters.max_total_time_ms;
    output << YAML::Key << "min_input_points" << YAML::Value
           << record.base_parameters.min_input_points;
    output << YAML::Key << "ndt_voxel_leaf_size_m" << YAML::Value;
    emitLevelValues(output, record.base_parameters.ndt_levels, [](const auto& level) {
        return level.voxel_leaf_size_m;
    });
    output << YAML::Key << "ndt_resolution_m" << YAML::Value;
    emitLevelValues(output, record.base_parameters.ndt_levels, [](const auto& level) {
        return level.resolution_m;
    });
    output << YAML::Key << "ndt_step_size_m" << YAML::Value;
    emitLevelValues(output, record.base_parameters.ndt_levels, [](const auto& level) {
        return level.step_size_m;
    });
    output << YAML::Key << "ndt_transformation_epsilon" << YAML::Value;
    emitLevelValues(output, record.base_parameters.ndt_levels, [](const auto& level) {
        return level.transformation_epsilon;
    });
    output << YAML::Key << "ndt_max_iterations" << YAML::Value;
    emitLevelValues(output, record.base_parameters.ndt_levels, [](const auto& level) {
        return level.max_iterations;
    });
    output << YAML::Key << "gicp_voxel_leaf_size_m" << YAML::Value;
    emitLevelValues(output, record.base_parameters.gicp_levels, [](const auto& level) {
        return level.voxel_leaf_size_m;
    });
    output << YAML::Key << "gicp_max_correspondence_distance_m" << YAML::Value;
    emitLevelValues(output, record.base_parameters.gicp_levels, [](const auto& level) {
        return level.max_correspondence_distance_m;
    });
    output << YAML::Key << "gicp_transformation_epsilon" << YAML::Value;
    emitLevelValues(output, record.base_parameters.gicp_levels, [](const auto& level) {
        return level.transformation_epsilon;
    });
    output << YAML::Key << "gicp_fitness_epsilon" << YAML::Value;
    emitLevelValues(output, record.base_parameters.gicp_levels, [](const auto& level) {
        return level.fitness_epsilon;
    });
    output << YAML::Key << "gicp_max_iterations" << YAML::Value;
    emitLevelValues(output, record.base_parameters.gicp_levels, [](const auto& level) {
        return level.max_iterations;
    });
    output << YAML::Key << "validation" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "correspondence_distance_m" << YAML::Value
           << record.base_parameters.validation.correspondence_distance_m;
    output << YAML::Key << "max_rmse_m" << YAML::Value
           << record.base_parameters.validation.max_rmse_m;
    output << YAML::Key << "min_overlap_ratio" << YAML::Value
           << record.base_parameters.validation.min_overlap_ratio;
    output << YAML::Key << "min_correspondences" << YAML::Value
           << record.base_parameters.validation.min_correspondences;
    output << YAML::Key << "enforce_initial_deviation_limits" << YAML::Value
           << record.base_parameters.validation.enforce_initial_deviation_limits;
    output << YAML::Key << "max_translation_from_initial_m" << YAML::Value
           << record.base_parameters.validation.max_translation_from_initial_m;
    output << YAML::Key << "max_yaw_from_initial_rad" << YAML::Value
           << record.base_parameters.validation.max_yaw_from_initial_rad;
    output << YAML::Key << "reject_if_search_boundary_hit" << YAML::Value
           << record.base_parameters.validation.reject_if_search_boundary_hit;
    output << YAML::Key << "boundary_ratio" << YAML::Value
           << record.base_parameters.validation.boundary_ratio;
    output << YAML::EndMap << YAML::EndMap;
    output << YAML::EndMap << YAML::EndMap;
    if (!output.good()) {
        throw std::runtime_error("failed to serialize localization record");
    }

    const std::filesystem::path temporary_path = output_path.string() + ".tmp";
    {
        std::ofstream stream(temporary_path, std::ios::out | std::ios::trunc);
        if (!stream) {
            throw std::runtime_error("failed to open localization result");
        }
        stream << output.c_str() << '\n';
        if (!stream) {
            throw std::runtime_error("failed to write localization result");
        }
    }
    std::error_code error;
    std::filesystem::rename(temporary_path, output_path, error);
    if (error) {
        std::filesystem::remove(temporary_path);
        throw std::runtime_error("failed to finalize localization result: " + error.message());
    }
    return output_path.string();
}

} // namespace dart_vision::lidar::localization

#include "dart_vision_lidar_model/model_builder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <yaml-cpp/yaml.h>

namespace dart_vision::lidar {
namespace {

constexpr int kSchemaVersion = 2;

std::string describe(const YAML::Node& node, const std::string& context) {
    if (node.Mark().line >= 0) {
        return context + " (line " + std::to_string(node.Mark().line + 1) + ")";
    }
    return context;
}

YAML::Node
requireNode(const YAML::Node& parent, const std::string& key, const std::string& context) {
    const YAML::Node child = parent[key];
    if (!child.IsDefined()) {
        throw std::invalid_argument(context + ": missing required key '" + key + "'");
    }
    return child;
}

template <typename ValueT> ValueT parseScalar(const YAML::Node& node, const std::string& context) {
    if (!node.IsScalar()) {
        throw std::invalid_argument(describe(node, context) + " must be a scalar");
    }
    try {
        return node.as<ValueT>();
    } catch (const YAML::Exception& error) {
        throw std::invalid_argument(describe(node, context) + ": " + error.what());
    }
}

std::string parseNonEmptyString(const YAML::Node& node, const std::string& context) {
    std::string value = parseScalar<std::string>(node, context);
    if (value.empty()) {
        throw std::invalid_argument(describe(node, context) + " must not be empty");
    }
    return value;
}

double parseFiniteDouble(const YAML::Node& node, const std::string& context) {
    const double value = parseScalar<double>(node, context);
    if (!std::isfinite(value)) {
        throw std::invalid_argument(describe(node, context) + " must be finite");
    }
    return value;
}

std::size_t parsePositiveSize(const YAML::Node& node, const std::string& context) {
    const std::int64_t value = parseScalar<std::int64_t>(node, context);
    if (value <= 0) {
        throw std::invalid_argument(describe(node, context) + " must be greater than zero");
    }
    if (static_cast<std::uint64_t>(value) > std::numeric_limits<std::size_t>::max()) {
        throw std::invalid_argument(describe(node, context) + " is too large");
    }
    return static_cast<std::size_t>(value);
}

std::uint32_t parseSeed(const YAML::Node& node, const std::string& context) {
    const std::int64_t value = parseScalar<std::int64_t>(node, context);
    if (value < 0 ||
        static_cast<std::uint64_t>(value) > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument(describe(node, context) + " must fit uint32");
    }
    return static_cast<std::uint32_t>(value);
}

std::filesystem::path resolvePath(const std::filesystem::path& config_directory,
                                  const YAML::Node& node,
                                  const std::string& context) {
    std::filesystem::path path(parseNonEmptyString(node, context));
    if (path.is_relative()) {
        path = config_directory / path;
    }
    return path.lexically_normal();
}

std::string lowercaseExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(
        extension.begin(), extension.end(), extension.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return extension;
}

std::filesystem::path resolveMeshPath(const std::filesystem::path& config_directory,
                                      const YAML::Node& node,
                                      const std::string& context) {
    const auto path = resolvePath(config_directory, node, context);
    const std::string extension = lowercaseExtension(path);
    if (extension != ".ply" && extension != ".stl") {
        throw std::invalid_argument(describe(node, context) +
                                    " must use the '.ply' or '.stl' extension");
    }
    return path;
}

std::filesystem::path resolvePcdPath(const std::filesystem::path& config_directory,
                                     const YAML::Node& node,
                                     const std::string& context) {
    const auto path = resolvePath(config_directory, node, context);
    if (lowercaseExtension(path) != ".pcd") {
        throw std::invalid_argument(describe(node, context) + " must use the '.pcd' extension");
    }
    return path;
}

void validateInput(const ModelBuildEntry& entry) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(entry.input_mesh, error) || error) {
        throw std::runtime_error(
            "model '" + entry.name +
            "' input mesh is not a readable regular file: " + entry.input_mesh.string());
    }
    if (entry.input_mesh == entry.output_pcd) {
        throw std::invalid_argument("model '" + entry.name +
                                    "' input and output paths must differ");
    }
}

} // namespace

ModelBuildConfig loadModelBuildConfig(const std::filesystem::path& config_path) {
    if (config_path.empty()) {
        throw std::invalid_argument("config path must not be empty");
    }

    std::error_code error;
    const std::filesystem::path absolute_config =
        std::filesystem::absolute(config_path, error).lexically_normal();
    if (error || !std::filesystem::is_regular_file(absolute_config)) {
        throw std::runtime_error("model config is not a readable regular file: " +
                                 config_path.string());
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(absolute_config.string());
    } catch (const YAML::Exception& yaml_error) {
        throw std::runtime_error("failed to parse model config '" + absolute_config.string() +
                                 "': " + yaml_error.what());
    }
    if (!root.IsMap()) {
        throw std::invalid_argument("model config root must be a map");
    }

    const int schema_version =
        parseScalar<int>(requireNode(root, "schema_version", "config"), "schema_version");
    if (schema_version != kSchemaVersion) {
        throw std::invalid_argument("unsupported schema_version " + std::to_string(schema_version) +
                                    "; expected " + std::to_string(kSchemaVersion));
    }

    const YAML::Node models = requireNode(root, "models", "config");
    if (!models.IsSequence() || models.size() == 0U) {
        throw std::invalid_argument("models must be a non-empty sequence");
    }

    ModelBuildConfig config;
    config.config_path = absolute_config;
    config.config_directory = absolute_config.parent_path();
    config.models.reserve(models.size());

    std::set<std::string> names;
    std::set<std::filesystem::path> output_paths;
    for (std::size_t index = 0U; index < models.size(); ++index) {
        const YAML::Node node = models[index];
        const std::string context = "models[" + std::to_string(index) + "]";
        if (!node.IsMap()) {
            throw std::invalid_argument(describe(node, context) + " must be a map");
        }

        ModelBuildEntry entry;
        entry.name = parseNonEmptyString(requireNode(node, "name", context), context + ".name");
        if (!names.insert(entry.name).second) {
            throw std::invalid_argument("duplicate model name: " + entry.name);
        }
        entry.input_mesh = resolveMeshPath(config.config_directory,
                                           requireNode(node, "input_mesh", context),
                                           context + ".input_mesh");
        entry.output_pcd = resolvePcdPath(config.config_directory,
                                          requireNode(node, "output_pcd", context),
                                          context + ".output_pcd");
        if (!output_paths.insert(entry.output_pcd).second) {
            throw std::invalid_argument("multiple models use output path: " +
                                        entry.output_pcd.string());
        }

        entry.sampling.scale_to_m =
            parseFiniteDouble(requireNode(node, "scale_to_m", context), context + ".scale_to_m");
        if (entry.sampling.scale_to_m <= 0.0) {
            throw std::invalid_argument(context + ".scale_to_m must be greater than zero");
        }
        entry.sampling.sample_count = parsePositiveSize(requireNode(node, "sample_count", context),
                                                        context + ".sample_count");
        entry.sampling.seed =
            parseSeed(requireNode(node, "random_seed", context), context + ".random_seed");
        entry.sampling.voxel_leaf_size_m = parseFiniteDouble(
            requireNode(node, "voxel_leaf_m", context), context + ".voxel_leaf_m");
        if (entry.sampling.voxel_leaf_size_m < 0.0) {
            throw std::invalid_argument(context + ".voxel_leaf_m must be non-negative");
        }
        entry.overwrite =
            parseScalar<bool>(requireNode(node, "overwrite", context), context + ".overwrite");
        validateInput(entry);
        config.models.push_back(std::move(entry));
    }
    return config;
}

ModelBuildReport buildModels(const ModelBuildConfig& config) {
    if (config.models.empty()) {
        throw std::invalid_argument("model build config contains no models");
    }

    // Preflight every output before sampling so an overwrite-policy error
    // cannot leave only a prefix of the model list regenerated.
    for (const auto& entry : config.models) {
        validateInput(entry);
        std::error_code error;
        const bool output_exists = std::filesystem::exists(entry.output_pcd, error);
        if (error) {
            throw std::runtime_error("cannot inspect output path for model '" + entry.name +
                                     "': " + error.message());
        }
        if (output_exists && !entry.overwrite) {
            throw std::runtime_error(
                "model '" + entry.name +
                "' output already exists and overwrite is false: " + entry.output_pcd.string());
        }
        if (output_exists && !std::filesystem::is_regular_file(entry.output_pcd)) {
            throw std::runtime_error(
                "model '" + entry.name +
                "' output path is not a regular file: " + entry.output_pcd.string());
        }
    }

    ModelBuildReport report;
    report.models.reserve(config.models.size());
    for (const auto& entry : config.models) {
        ModelBuildRecord record;
        record.name = entry.name;
        record.output_pcd = entry.output_pcd;

        std::error_code error;
        const auto output_directory = entry.output_pcd.parent_path();
        if (!output_directory.empty()) {
            std::filesystem::create_directories(output_directory, error);
            if (error) {
                throw std::runtime_error("failed to create output directory for model '" +
                                         entry.name + "': " + error.message());
            }
        }

        const ModelSamplingResult sampled =
            sampleMeshFile(entry.input_mesh.string(), entry.sampling);
        savePcd(entry.output_pcd.string(), *sampled.cloud);
        record.stats = sampled.stats;
        ++report.built_count;
        report.models.push_back(std::move(record));
    }
    return report;
}

} // namespace dart_vision::lidar

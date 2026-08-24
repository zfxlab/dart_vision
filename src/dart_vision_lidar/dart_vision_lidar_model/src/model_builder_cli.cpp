#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "dart_vision_lidar_model/model_builder.hpp"

namespace {

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " --config MODELS.yaml\n"
              << "\n"
              << "Build all PLY/STL model entries from an ordinary YAML file.\n"
              << "Relative input and output paths are resolved against the YAML directory.\n";
}

std::filesystem::path parseArguments(int argc, char** argv) {
    std::filesystem::path config;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "-h" || argument == "--help") {
            printUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--config") {
            if (++index >= argc) {
                throw std::invalid_argument("--config requires a YAML path");
            }
            if (!config.empty()) {
                throw std::invalid_argument("--config may be specified only once");
            }
            config = argv[index];
            continue;
        }
        throw std::invalid_argument("unknown argument: " + argument);
    }
    if (config.empty()) {
        throw std::invalid_argument("--config is required");
    }
    return config;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto config_path = parseArguments(argc, argv);
        const auto config = dart_vision::lidar::loadModelBuildConfig(config_path);
        const auto report = dart_vision::lidar::buildModels(config);
        for (const auto& model : report.models) {
            std::cout << "BUILT " << model.name << ": " << model.stats.point_count_after_voxel
                      << " points -> " << model.output_pcd << "\n";
        }
        std::cout << "Completed: " << report.built_count << " built\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "model builder error: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
}

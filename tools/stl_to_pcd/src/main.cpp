#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "stl_to_pcd/converter.hpp"

namespace {

struct Arguments {
    std::filesystem::path input;
    std::filesystem::path output;
    stl_to_pcd::ConversionOptions options;
};

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " INPUT.stl OUTPUT.pcd [options]\n\n"
              << "Uniformly sample an STL surface and save it as a binary PCD.\n\n"
              << "Options:\n"
              << "  -n, --samples N   Candidate surface points before voxel filtering"
                 " (default: 100000)\n"
              << "  -s, --scale S     Coordinate scale applied before sampling (default: 1.0)\n"
              << "  -v, --voxel V     Voxel leaf size in scaled units; 0 disables (default: 0)\n"
              << "      --seed N      Random seed in uint32 range (default: 0)\n"
              << "      --overwrite   Replace an existing output PCD\n"
              << "  -h, --help        Show this help\n";
}

std::string requireValue(int argc, char** argv, int& index, const std::string& option) {
    if (++index >= argc) {
        throw std::invalid_argument(option + " requires a value");
    }
    return argv[index];
}

std::uint64_t parseUnsigned(const std::string& text, const std::string& option) {
    std::uint64_t value = 0U;
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto parsed = std::from_chars(begin, end, value);
    if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != end) {
        throw std::invalid_argument(option + " requires a non-negative integer, got: " + text);
    }
    return value;
}

double parseDouble(const std::string& text, const std::string& option) {
    std::size_t consumed = 0U;
    double value = 0.0;
    try {
        value = std::stod(text, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument(option + " requires a number, got: " + text);
    }
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument(option + " requires a finite number, got: " + text);
    }
    return value;
}

Arguments parseArguments(int argc, char** argv) {
    Arguments arguments;
    int positional_count = 0;

    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "-h" || argument == "--help") {
            printUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "-n" || argument == "--samples") {
            const auto value = parseUnsigned(requireValue(argc, argv, index, argument), argument);
            if (value == 0U || value > std::numeric_limits<std::size_t>::max()) {
                throw std::invalid_argument(argument + " must fit size_t and be greater than zero");
            }
            arguments.options.sample_count = static_cast<std::size_t>(value);
            continue;
        }
        if (argument == "-s" || argument == "--scale") {
            arguments.options.scale =
                parseDouble(requireValue(argc, argv, index, argument), argument);
            continue;
        }
        if (argument == "-v" || argument == "--voxel") {
            arguments.options.voxel_size =
                parseDouble(requireValue(argc, argv, index, argument), argument);
            continue;
        }
        if (argument == "--seed") {
            const auto value = parseUnsigned(requireValue(argc, argv, index, argument), argument);
            if (value > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument("--seed must fit uint32");
            }
            arguments.options.seed = static_cast<std::uint32_t>(value);
            continue;
        }
        if (argument == "--overwrite") {
            arguments.options.overwrite = true;
            continue;
        }
        if (!argument.empty() && argument.front() == '-') {
            throw std::invalid_argument("unknown option: " + argument);
        }

        if (positional_count == 0) {
            arguments.input = argument;
        } else if (positional_count == 1) {
            arguments.output = argument;
        } else {
            throw std::invalid_argument("too many positional arguments");
        }
        ++positional_count;
    }

    if (positional_count != 2) {
        throw std::invalid_argument("INPUT.stl and OUTPUT.pcd are required");
    }
    return arguments;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Arguments arguments = parseArguments(argc, argv);
        const auto stats =
            stl_to_pcd::convert(arguments.input, arguments.output, arguments.options);

        std::cout << "Converted " << arguments.input << " -> " << arguments.output << '\n'
                  << "Triangles: " << stats.valid_triangle_count << " valid, "
                  << stats.skipped_triangle_count << " skipped\n"
                  << "Surface area: " << stats.surface_area << " (scaled units^2)\n"
                  << "Points: " << stats.sampled_point_count << " sampled, "
                  << stats.output_point_count << " written\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "stl_to_pcd: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

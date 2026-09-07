#include "stl_to_pcd/converter.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace stl_to_pcd {
namespace {

struct Triangle {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;
    double area;
};

std::string lowercaseExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });
    return extension;
}

void validateOptions(const ConversionOptions& options) {
    if (options.sample_count == 0U) {
        throw std::invalid_argument("sample count must be greater than zero");
    }
    if (!std::isfinite(options.scale) || options.scale <= 0.0) {
        throw std::invalid_argument("scale must be finite and greater than zero");
    }
    if (!std::isfinite(options.voxel_size) || options.voxel_size < 0.0) {
        throw std::invalid_argument("voxel size must be finite and non-negative");
    }
}

bool isFinite(const Eigen::Vector3d& point) {
    return point.array().isFinite().all();
}

std::vector<Triangle>
collectTriangles(const pcl::PolygonMesh& mesh, double scale, ConversionStats& stats) {
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);
    if (vertices.empty()) {
        throw std::runtime_error("STL mesh contains no XYZ vertices");
    }

    std::vector<Triangle> triangles;
    triangles.reserve(mesh.polygons.size());

    for (const auto& polygon : mesh.polygons) {
        // STL is triangular. Reject malformed polygons instead of silently
        // changing their topology.
        ++stats.input_triangle_count;
        if (polygon.vertices.size() != 3U) {
            ++stats.skipped_triangle_count;
            continue;
        }

        const auto index_a = static_cast<std::size_t>(polygon.vertices[0]);
        const auto index_b = static_cast<std::size_t>(polygon.vertices[1]);
        const auto index_c = static_cast<std::size_t>(polygon.vertices[2]);
        if (index_a >= vertices.size() || index_b >= vertices.size() ||
            index_c >= vertices.size()) {
            ++stats.skipped_triangle_count;
            continue;
        }

        const auto& source_a = vertices[index_a];
        const auto& source_b = vertices[index_b];
        const auto& source_c = vertices[index_c];
        const Eigen::Vector3d a = scale * Eigen::Vector3d(source_a.x, source_a.y, source_a.z);
        const Eigen::Vector3d b = scale * Eigen::Vector3d(source_b.x, source_b.y, source_b.z);
        const Eigen::Vector3d c = scale * Eigen::Vector3d(source_c.x, source_c.y, source_c.z);
        const double area = 0.5 * (b - a).cross(c - a).norm();

        if (!isFinite(a) || !isFinite(b) || !isFinite(c) || !std::isfinite(area) ||
            area <= std::numeric_limits<double>::min()) {
            ++stats.skipped_triangle_count;
            continue;
        }

        triangles.push_back(Triangle{a, b, c, area});
        stats.surface_area += area;
    }

    stats.valid_triangle_count = triangles.size();
    if (triangles.empty() || !std::isfinite(stats.surface_area) || stats.surface_area <= 0.0) {
        throw std::runtime_error("STL mesh contains no finite, non-degenerate triangles");
    }
    return triangles;
}

pcl::PolygonMesh loadStl(const std::filesystem::path& input_stl) {
    if (input_stl.empty()) {
        throw std::invalid_argument("input STL path must not be empty");
    }
    if (lowercaseExtension(input_stl) != ".stl") {
        throw std::invalid_argument("input file must use the .stl extension: " +
                                    input_stl.string());
    }

    std::error_code error;
    if (!std::filesystem::is_regular_file(input_stl, error) || error) {
        throw std::runtime_error("input STL is not a readable regular file: " + input_stl.string());
    }

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(input_stl.string(), mesh) <= 0) {
        throw std::runtime_error("failed to load STL mesh: " + input_stl.string());
    }
    return mesh;
}

void savePcd(const std::filesystem::path& output_pcd,
             const pcl::PointCloud<pcl::PointXYZ>& cloud,
             bool overwrite) {
    if (output_pcd.empty()) {
        throw std::invalid_argument("output PCD path must not be empty");
    }
    if (lowercaseExtension(output_pcd) != ".pcd") {
        throw std::invalid_argument("output file must use the .pcd extension: " +
                                    output_pcd.string());
    }
    if (cloud.empty()) {
        throw std::invalid_argument("refusing to save an empty point cloud");
    }

    std::error_code error;
    const bool output_exists = std::filesystem::exists(output_pcd, error);
    if (error) {
        throw std::runtime_error("cannot inspect output path: " + error.message());
    }
    if (output_exists && !overwrite) {
        throw std::runtime_error("output PCD already exists; pass --overwrite to replace it: " +
                                 output_pcd.string());
    }
    if (output_exists && !std::filesystem::is_regular_file(output_pcd, error)) {
        throw std::runtime_error("output path exists but is not a regular file: " +
                                 output_pcd.string());
    }

    const auto parent = output_pcd.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent, error);
        if (error) {
            throw std::runtime_error("failed to create output directory: " + error.message());
        }
    }

    if (pcl::io::savePCDFileBinary(output_pcd.string(), cloud) < 0) {
        throw std::runtime_error("failed to save PCD: " + output_pcd.string());
    }
}

} // namespace

SamplingResult sampleSurface(const pcl::PolygonMesh& mesh, const ConversionOptions& options) {
    validateOptions(options);

    SamplingResult result;
    const auto triangles = collectTriangles(mesh, options.scale, result.stats);

    std::vector<double> weights;
    weights.reserve(triangles.size());
    for (const auto& triangle : triangles) {
        weights.push_back(triangle.area);
    }

    std::mt19937 random_engine(options.seed);
    std::discrete_distribution<std::size_t> triangle_distribution(weights.begin(), weights.end());
    std::uniform_real_distribution<double> unit_distribution(0.0, 1.0);

    result.cloud->reserve(options.sample_count);
    for (std::size_t index = 0U; index < options.sample_count; ++index) {
        const Triangle& triangle = triangles[triangle_distribution(random_engine)];

        // sqrt(u) produces uniform barycentric coordinates over triangle area.
        const double root_u = std::sqrt(unit_distribution(random_engine));
        const double v = unit_distribution(random_engine);
        const Eigen::Vector3d point =
            (1.0 - root_u) * triangle.a + root_u * (1.0 - v) * triangle.b + root_u * v * triangle.c;
        result.cloud->emplace_back(static_cast<float>(point.x()),
                                   static_cast<float>(point.y()),
                                   static_cast<float>(point.z()));
    }

    result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
    result.cloud->height = 1U;
    result.cloud->is_dense = true;
    result.stats.sampled_point_count = result.cloud->size();

    if (options.voxel_size > 0.0) {
        const float leaf = static_cast<float>(options.voxel_size);
        if (!std::isfinite(leaf) || leaf <= 0.0F) {
            throw std::invalid_argument("voxel size cannot be represented as a positive float");
        }
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(leaf, leaf, leaf);
        voxel_grid.setInputCloud(result.cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_grid.filter(*filtered);
        result.cloud = std::move(filtered);
    }

    result.stats.output_point_count = result.cloud->size();
    return result;
}

ConversionStats convert(const std::filesystem::path& input_stl,
                        const std::filesystem::path& output_pcd,
                        const ConversionOptions& options) {
    validateOptions(options);
    if (lowercaseExtension(output_pcd) != ".pcd") {
        throw std::invalid_argument("output file must use the .pcd extension: " +
                                    output_pcd.string());
    }

    std::error_code error;
    if (std::filesystem::exists(output_pcd, error) && !options.overwrite) {
        throw std::runtime_error("output PCD already exists; pass --overwrite to replace it: " +
                                 output_pcd.string());
    }
    if (error) {
        throw std::runtime_error("cannot inspect output path: " + error.message());
    }

    const SamplingResult result = sampleSurface(loadStl(input_stl), options);
    savePcd(output_pcd, *result.cloud, options.overwrite);
    return result.stats;
}

} // namespace stl_to_pcd

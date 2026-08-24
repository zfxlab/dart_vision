#include "dart_vision_lidar_model/model_sampler.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dart_vision::lidar {
namespace {

constexpr double kMinimumTriangleAreaM2 = 1.0e-16;

struct Triangle {
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    Eigen::Vector3d c;
    double area_m2;
};

bool isFinite(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

void validateOptions(const ModelSamplingOptions& options) {
    if (options.sample_count == 0U) {
        throw std::invalid_argument("sample_count must be greater than zero");
    }
    if (!std::isfinite(options.scale_to_m) || options.scale_to_m <= 0.0) {
        throw std::invalid_argument("scale_to_m must be finite and greater than zero");
    }
    if (!std::isfinite(options.voxel_leaf_size_m) || options.voxel_leaf_size_m < 0.0) {
        throw std::invalid_argument("voxel_leaf_size_m must be finite and non-negative");
    }
}

std::vector<Triangle>
collectTriangles(const pcl::PolygonMesh& mesh, double scale_to_m, ModelSamplingStats& stats) {
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);
    if (vertices.empty()) {
        throw std::runtime_error("mesh contains no XYZ vertices");
    }

    stats.input_polygon_count = mesh.polygons.size();
    std::vector<Triangle> triangles;
    triangles.reserve(mesh.polygons.size());

    for (const auto& polygon : mesh.polygons) {
        if (polygon.vertices.size() < 3U) {
            ++stats.skipped_invalid_triangle_count;
            continue;
        }

        // Fan triangulation also preserves the common three-index triangle case.
        for (std::size_t index = 1U; index + 1U < polygon.vertices.size(); ++index) {
            ++stats.candidate_triangle_count;
            const auto index_a = static_cast<std::size_t>(polygon.vertices[0U]);
            const auto index_b = static_cast<std::size_t>(polygon.vertices[index]);
            const auto index_c = static_cast<std::size_t>(polygon.vertices[index + 1U]);
            if (index_a >= vertices.size() || index_b >= vertices.size() ||
                index_c >= vertices.size()) {
                ++stats.skipped_invalid_triangle_count;
                continue;
            }

            const auto& point_a = vertices[index_a];
            const auto& point_b = vertices[index_b];
            const auto& point_c = vertices[index_c];
            const Eigen::Vector3d a = scale_to_m * Eigen::Vector3d(point_a.x, point_a.y, point_a.z);
            const Eigen::Vector3d b = scale_to_m * Eigen::Vector3d(point_b.x, point_b.y, point_b.z);
            const Eigen::Vector3d c = scale_to_m * Eigen::Vector3d(point_c.x, point_c.y, point_c.z);

            if (!isFinite(a) || !isFinite(b) || !isFinite(c)) {
                ++stats.skipped_invalid_triangle_count;
                continue;
            }

            const double area_m2 = 0.5 * (b - a).cross(c - a).norm();
            if (!std::isfinite(area_m2) || area_m2 <= kMinimumTriangleAreaM2) {
                ++stats.skipped_degenerate_triangle_count;
                continue;
            }
            triangles.push_back(Triangle{a, b, c, area_m2});
            stats.surface_area_m2 += area_m2;
        }
    }

    stats.sampled_triangle_count = triangles.size();
    if (triangles.empty() || !std::isfinite(stats.surface_area_m2) ||
        stats.surface_area_m2 <= 0.0) {
        throw std::runtime_error("mesh contains no finite, non-degenerate triangles");
    }
    return triangles;
}

} // namespace

pcl::PolygonMesh loadMesh(const std::string& mesh_path) {
    if (mesh_path.empty()) {
        throw std::invalid_argument("mesh path must not be empty");
    }

    std::string extension = std::filesystem::path(mesh_path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });

    pcl::PolygonMesh mesh;
    if (extension == ".ply") {
        if (pcl::io::loadPLYFile(mesh_path, mesh) < 0) {
            throw std::runtime_error("failed to load PLY mesh: " + mesh_path);
        }
    } else if (extension == ".stl") {
        if (pcl::io::loadPolygonFileSTL(mesh_path, mesh) <= 0) {
            throw std::runtime_error("failed to load STL mesh: " + mesh_path);
        }
    } else {
        throw std::invalid_argument("unsupported mesh extension '" + extension +
                                    "' (expected .ply or .stl): " + mesh_path);
    }
    return mesh;
}

ModelSamplingResult sampleMesh(const pcl::PolygonMesh& mesh, const ModelSamplingOptions& options) {
    validateOptions(options);

    ModelSamplingResult result;
    const std::vector<Triangle> triangles =
        collectTriangles(mesh, options.scale_to_m, result.stats);

    std::vector<double> cumulative_areas;
    cumulative_areas.reserve(triangles.size());
    double cumulative_area = 0.0;
    for (const auto& triangle : triangles) {
        cumulative_area += triangle.area_m2;
        cumulative_areas.push_back(cumulative_area);
    }

    result.cloud->reserve(options.sample_count);
    std::mt19937 random_engine(options.seed);
    std::uniform_real_distribution<double> area_distribution(0.0,
                                                             std::nextafter(cumulative_area, 0.0));
    std::uniform_real_distribution<double> unit_distribution(0.0, 1.0);

    for (std::size_t point_index = 0U; point_index < options.sample_count; ++point_index) {
        const double selected_area = area_distribution(random_engine);
        const auto area_it =
            std::upper_bound(cumulative_areas.begin(), cumulative_areas.end(), selected_area);
        const std::size_t triangle_index =
            static_cast<std::size_t>(std::distance(cumulative_areas.begin(), area_it));
        const Triangle& triangle = triangles[std::min(triangle_index, triangles.size() - 1U)];

        // sqrt(u) turns two independent uniform variates into uniform barycentric
        // coordinates over triangle area.
        const double root_u = std::sqrt(unit_distribution(random_engine));
        const double v = unit_distribution(random_engine);
        const Eigen::Vector3d point_m = (1.0 - root_u) * triangle.a +
                                        (root_u * (1.0 - v)) * triangle.b +
                                        (root_u * v) * triangle.c;
        result.cloud->emplace_back(static_cast<float>(point_m.x()),
                                   static_cast<float>(point_m.y()),
                                   static_cast<float>(point_m.z()));
    }

    result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
    result.cloud->height = 1U;
    result.cloud->is_dense = true;
    result.stats.point_count_before_voxel = result.cloud->size();

    if (options.voxel_leaf_size_m > 0.0) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        const float leaf_size = static_cast<float>(options.voxel_leaf_size_m);
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.setInputCloud(result.cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_grid.filter(*filtered_cloud);
        result.cloud = std::move(filtered_cloud);
    }
    result.stats.point_count_after_voxel = result.cloud->size();
    return result;
}

ModelSamplingResult sampleMeshFile(const std::string& mesh_path,
                                   const ModelSamplingOptions& options) {
    return sampleMesh(loadMesh(mesh_path), options);
}

void savePcd(const std::string& pcd_path, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    if (pcd_path.empty()) {
        throw std::invalid_argument("PCD path must not be empty");
    }
    if (cloud.empty()) {
        throw std::invalid_argument("refusing to save an empty point cloud");
    }

    if (pcl::io::savePCDFileBinary(pcd_path, cloud) < 0) {
        throw std::runtime_error("failed to save PCD cloud: " + pcd_path);
    }
}

} // namespace dart_vision::lidar

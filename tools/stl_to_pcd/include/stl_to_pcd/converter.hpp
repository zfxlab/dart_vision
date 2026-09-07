#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace stl_to_pcd {

struct ConversionOptions {
    std::size_t sample_count{100000U};
    double scale{1.0};
    double voxel_size{0.0};
    std::uint32_t seed{0U};
    bool overwrite{false};
};

struct ConversionStats {
    std::size_t input_triangle_count{0U};
    std::size_t valid_triangle_count{0U};
    std::size_t skipped_triangle_count{0U};
    std::size_t sampled_point_count{0U};
    std::size_t output_point_count{0U};
    double surface_area{0.0};
};

struct SamplingResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    ConversionStats stats;
};

// scale and voxel_size use the same target coordinate unit. Scaling is applied
// before surface sampling and voxel filtering.
SamplingResult sampleSurface(const pcl::PolygonMesh& mesh, const ConversionOptions& options);

ConversionStats convert(const std::filesystem::path& input_stl,
                        const std::filesystem::path& output_pcd,
                        const ConversionOptions& options);

} // namespace stl_to_pcd

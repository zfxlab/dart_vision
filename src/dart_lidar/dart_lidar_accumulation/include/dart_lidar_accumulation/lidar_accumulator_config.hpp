#ifndef DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_CONFIG_HPP
#define DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_CONFIG_HPP

#include <Eigen/Core>
#include <cstddef>
#include <string>

namespace dart_vision::lidar {

struct TransformConfig {
    bool crop_box_enabled{true};
    Eigen::Vector3f crop_box_min_m{-1.5F, -1.5F, -0.5F};
    Eigen::Vector3f crop_box_max_m{1.5F, 1.5F, 2.0F};

    [[nodiscard]] std::string validationError() const;
};

struct AccumulationConfig {
    double window_duration_s{0.60};
    std::size_t min_frames{10U};
    std::size_t max_frames{12U};
    std::size_t max_points{100000U};
    bool output_voxel_grid_enabled{true};
    double output_voxel_leaf_size_m{0.015};

    [[nodiscard]] std::string validationError() const;
};

struct PublishConfig {
    double rate_hz{2.0};
    std::size_t min_new_frames{8U};
    bool immediately_when_ready{true};

    [[nodiscard]] std::string validationError() const;
};

struct LidarAccumulatorConfig {
    std::string input_topic{"lidar/preprocessed"};
    std::string output_topic{"lidar/accumulated"};
    std::string accumulation_frame{"base_nominal_link"};
    TransformConfig transform;
    AccumulationConfig accumulation;
    PublishConfig publish;

    [[nodiscard]] std::string validationError() const;
};

} // namespace dart_vision::lidar

#endif // DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_CONFIG_HPP

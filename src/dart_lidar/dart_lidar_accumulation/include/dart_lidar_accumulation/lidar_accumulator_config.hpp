#ifndef DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_CONFIG_HPP
#define DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_CONFIG_HPP

#include <Eigen/Core>
#include <cstddef>
#include <string>

namespace dart_vision::lidar {

enum class RuntimeMode { kTriggeredOnline, kBagOffline };

struct StabilityConfig {
    double hold_s{0.06};
    double yaw_span_rad{0.001745};
    double abort_yaw_deviation_rad{0.001745};
    double controller_timeout_s{0.2};
    double post_stable_delay_s{0.07};
    double measurement_deadline_s{1.0};
    double localization_reserve_s{0.4};

    [[nodiscard]] std::string validationError() const;
};

struct TransformConfig {
    bool crop_box_enabled{true};
    Eigen::Vector3f crop_box_min_m{-1.5F, -1.5F, -0.5F};
    Eigen::Vector3f crop_box_max_m{1.5F, 1.5F, 2.0F};

    [[nodiscard]] std::string validationError() const;
};

struct AccumulationConfig {
    double duration_s{0.35};
    std::size_t min_frames{6U};
    std::size_t max_frames{8U};
    std::size_t max_points{100000U};
    bool output_voxel_grid_enabled{true};
    double output_voxel_leaf_size_m{0.01};

    [[nodiscard]] std::string validationError() const;
};

struct LidarAccumulatorConfig {
    RuntimeMode mode{RuntimeMode::kTriggeredOnline};
    std::string mode_name{"triggered_online"};
    std::string input_topic{"lidar/preprocessed"};
    std::string output_topic{"debug/lidar_accumulated"};
    std::string controller_state_topic{"controller_state"};
    std::string accumulation_frame{"base_nominal_link"};
    StabilityConfig stability;
    TransformConfig transform;
    AccumulationConfig accumulation;

    [[nodiscard]] std::string validationError() const;
};

} // namespace dart_vision::lidar

#endif // DART_LIDAR_ACCUMULATION_LIDAR_ACCUMULATOR_CONFIG_HPP

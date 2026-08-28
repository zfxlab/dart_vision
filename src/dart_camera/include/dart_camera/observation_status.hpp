#ifndef DART_CAMERA_OBSERVATION_STATUS_HPP
#define DART_CAMERA_OBSERVATION_STATUS_HPP

#include <cstdint>

#include "dart_interfaces/msg/camera_observation.hpp"

namespace dart_vision::camera {

[[nodiscard]] constexpr std::uint8_t selectObservationStatus(const bool target_detected,
                                                             const bool calibrated,
                                                             const bool bearing_valid) noexcept {
    using Observation = dart_interfaces::msg::CameraObservation;
    if (!target_detected) {
        return Observation::STATUS_NO_TARGET;
    }
    if (!calibrated) {
        return Observation::STATUS_NOT_CALIBRATED;
    }
    if (!bearing_valid) {
        return Observation::STATUS_INTERNAL_ERROR;
    }
    return Observation::STATUS_OK;
}

} // namespace dart_vision::camera

#endif // DART_CAMERA_OBSERVATION_STATUS_HPP

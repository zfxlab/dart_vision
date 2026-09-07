#ifndef DART_LIDAR_LOCALIZATION_IO_RESULT_WRITER_HPP
#define DART_LIDAR_LOCALIZATION_IO_RESULT_WRITER_HPP

#include "dart_lidar_localization/localization_types.hpp"

#include <cstdint>
#include <string>

namespace dart_vision::lidar::localization {

struct LocalizationRecord {
    std::string generated_at;
    std::string filename_timestamp;
    std::string profile;
    std::uint64_t measurement_id{0U};
    std::int32_t source_stamp_sec{0};
    std::uint32_t source_stamp_nanosec{0U};
    std::string source_frame;
    std::string reference_frame;
    std::string base_frame;
    std::string rail_frame;
    std::string base_model_path;
    std::string module_model_path;
    std::string bag_name;
    LocalizationResult result;
    BaseRegistrationParameters base_parameters;
    ModuleLocalizationParameters module_parameters;
};

class ResultWriter {
public:
    [[nodiscard]] static std::string write(const std::string& directory,
                                           const std::string& filename_prefix,
                                           const LocalizationRecord& record);
};

} // namespace dart_vision::lidar::localization

#endif // DART_LIDAR_LOCALIZATION_IO_RESULT_WRITER_HPP

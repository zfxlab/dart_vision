#include <cstdlib>
#include <filesystem>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "dart_lidar_localization/calibration/confirmation_gate.hpp"
#include "dart_lidar_localization/calibration/result_writer.hpp"

namespace dart_vision::lidar::localization {
TEST(ConfirmationGate, RequiresFreshConsistentResultsAndResetsOnFailure) {
    ConfirmationGate gate;
    BaseRegistrationResult result;
    result.status = BaseRegistrationStatus::kSuccess;
    EXPECT_FALSE(gate.observe(result, 1000000000LL));
    EXPECT_FALSE(gate.observe(result, 1000000000LL));
    EXPECT_FALSE(gate.observe(result, 1500000000LL)); // 重叠窗口，不计数。
    EXPECT_FALSE(gate.observe(result, 2000000000LL));
    EXPECT_TRUE(gate.observe(result, 3000000000LL));
    result.status = BaseRegistrationStatus::kQualityRejected;
    EXPECT_FALSE(gate.observe(result, 4000000000LL));
    result.status = BaseRegistrationStatus::kSuccess;
    EXPECT_FALSE(gate.observe(result, 5000000000LL));
}

TEST(ConfirmationGate, RejectsDriftAndResetsOnBagRewind) {
    ConfirmationGate gate;
    BaseRegistrationResult result;
    result.status = BaseRegistrationStatus::kSuccess;
    EXPECT_FALSE(gate.observe(result, 1000000000LL));
    result.observation_from_model.translation().x() = 0.008;
    EXPECT_FALSE(gate.observe(result, 2000000000LL));
    result.observation_from_model.translation().x() = 0.016;
    EXPECT_FALSE(gate.observe(result, 3000000000LL)); // 相对首帧超过1cm。
    EXPECT_FALSE(gate.observe(result, 1000000000LL)); // 回放倒退重新确认。
    EXPECT_FALSE(gate.observe(result, 2000000000LL));
    EXPECT_TRUE(gate.observe(result, 3000000000LL));
}

TEST(ResultWriter, ExportsCandidatePoseWithoutModuleAndNeverOverwrites) {
    char directory[] = "/tmp/dart_calibration_test_XXXXXX";
    ASSERT_NE(mkdtemp(directory), nullptr);
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() {
            std::filesystem::remove_all(path);
        }
    } cleanup{directory};
    LocalizationRecord record;
    record.profile = "calibration";
    record.filename_timestamp = "test";
    record.result.status = BaseRegistrationStatus::kQualityRejected;
    record.result.observation_from_model.translation() = Eigen::Vector3d(0.3, -0.2, 0.1);
    const auto first = ResultWriter::write(directory, "base", record);
    const auto second = ResultWriter::write(directory, "base", record);
    EXPECT_NE(first, second);
    const auto document = YAML::LoadFile(first);
    EXPECT_FALSE(document["base"]["available"].as<bool>());
    EXPECT_FALSE(document["confirmed"].as<bool>());
    EXPECT_FALSE(document["module"]);
    EXPECT_DOUBLE_EQ(document["base"]["six_dof"]["xyz_m"][0].as<double>(), 0.3);
    EXPECT_DOUBLE_EQ(document["base"]["six_dof"]["xyz_m"][1].as<double>(), -0.2);
    EXPECT_TRUE(document["parameters"]["base"]);
}
} // namespace dart_vision::lidar::localization

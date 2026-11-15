#include "dart_camera/green_light_detector_node.hpp"

#include <cmath>
#include <functional>
#include <gtest/gtest.h>
#include <thread>

using dart_vision::camera::GreenLightDetectorNode;
using Detection = dart_interfaces::msg::GreenLightDetection;

class CenterFilterNode : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("profiles.default.center.method", "contour_centroid")});
        node = std::make_shared<GreenLightDetectorNode>(options);
        input = std::make_shared<rclcpp::Node>("ema_test_input");
        const auto qos = rclcpp::SensorDataQoS();
        images = input->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
        infos = input->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
        controllers =
            input->create_publisher<dart_interfaces::msg::ControllerState>("controller_state", qos);
        filtered_subscription = input->create_subscription<Detection>(
            "detection", qos, [this](Detection::ConstSharedPtr msg) { filtered = msg; });
        executor.add_node(node);
        executor.add_node(input);
        ASSERT_TRUE(wait([&] {
            return images->get_subscription_count() && infos->get_subscription_count() &&
                   controllers->get_subscription_count() &&
                   filtered_subscription->get_publisher_count();
        }));
        info.header.frame_id = "test_optical";
        info.width = info.height = 160;
        info.distortion_model = "plumb_bob";
        info.k = {100, 0, 60, 0, 100, 60, 0, 0, 1};
        infos->publish(info);
        setMode(1);
    }

    bool wait(const std::function<bool()>& ready) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        do {
            executor.spin_some();
            if (ready())
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        } while (std::chrono::steady_clock::now() < deadline);
        return false;
    }

    void drain() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
        ASSERT_TRUE(wait([&] { return std::chrono::steady_clock::now() >= deadline; }));
    }

    void setMode(std::uint8_t mode) {
        dart_interfaces::msg::ControllerState controller;
        controller.target_mode = mode;
        controllers->publish(controller);
        drain();
    }

    void observe(int x, double seconds, int radius = 15, bool invalid_encoding = false) {
        sensor_msgs::msg::Image image;
        image.header.frame_id = info.header.frame_id;
        image.header.stamp =
            rclcpp::Time(1794744624000000000LL + static_cast<std::int64_t>(seconds * 1e9));
        image.width = image.height = 160;
        image.encoding = invalid_encoding ? "invalid" : "bgr8";
        image.step = 480;
        image.data.resize(160 * 480, 0);
        if (radius > 0)
            for (int y = 60 - radius; y <= 60 + radius; ++y)
                for (int xx = x - radius; xx <= x + radius; ++xx)
                    if ((xx - x) * (xx - x) + (y - 60) * (y - 60) <= radius * radius)
                        image.data[y * image.step + xx * 3 + 1] = 230;
        filtered.reset();
        images->publish(image);
        ASSERT_TRUE(wait([&] { return filtered && filtered->header.stamp == image.header.stamp; }));
    }

    std::shared_ptr<GreenLightDetectorNode> node;
    rclcpp::Node::SharedPtr input;
    rclcpp::executors::SingleThreadedExecutor executor;
    sensor_msgs::msg::CameraInfo info;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infos;
    rclcpp::Publisher<dart_interfaces::msg::ControllerState>::SharedPtr controllers;
    rclcpp::Subscription<Detection>::SharedPtr filtered_subscription;
    Detection::ConstSharedPtr filtered;
};

TEST_F(CenterFilterNode, SmoothsPixelsAndRecomputesRay) {
    ASSERT_NO_FATAL_FAILURE(observe(60, 0));
    ASSERT_EQ(filtered->status, Detection::DETECTED);
    EXPECT_NEAR(filtered->center_x_px, 60, 1e-5);
    const auto score = filtered->score;
    ASSERT_NO_FATAL_FAILURE(observe(70, 0.1));
    ASSERT_EQ(filtered->status, Detection::DETECTED);
    EXPECT_NEAR(filtered->center_x_px, 60 + 10 * (1 - std::exp(-0.1)), 1e-5);
    EXPECT_NEAR(filtered->unit_ray.x / filtered->unit_ray.z, (filtered->center_x_px - 60) / 100.0,
                1e-7);
    EXPECT_EQ(filtered->header.frame_id, info.header.frame_id);
    EXPECT_EQ(filtered->score, score);
}

TEST_F(CenterFilterNode, InvalidDetectionsResetInsteadOfPublishingStaleCoordinates) {
    double time = 0;
    for (const auto status : {Detection::CLOSED, Detection::NO_TARGET, Detection::ERROR}) {
        ASSERT_NO_FATAL_FAILURE(observe(60, time));
        ASSERT_NO_FATAL_FAILURE(observe(70, time + 0.1));
        ASSERT_NO_FATAL_FAILURE(observe(70, time + 0.2, status == Detection::CLOSED ? 0 : 2,
                                        status == Detection::ERROR));
        ASSERT_EQ(filtered->status, status);
        EXPECT_FLOAT_EQ(filtered->center_x_px, 0);
        EXPECT_FLOAT_EQ(filtered->center_y_px, 0);
        EXPECT_DOUBLE_EQ(filtered->unit_ray.z, 0);
        ASSERT_NO_FATAL_FAILURE(observe(80, time + 0.3));
        ASSERT_EQ(filtered->status, Detection::DETECTED);
        EXPECT_NEAR(filtered->center_x_px, 80, 1e-5);
        time += 0.4;
    }
}

TEST_F(CenterFilterNode, ModeChangesWithinOneProfileAndCalibrationChangesResetHistory) {
    ASSERT_NO_FATAL_FAILURE(observe(60, 0));
    ASSERT_NO_FATAL_FAILURE(observe(70, 0.1));
    setMode(2);
    ASSERT_NO_FATAL_FAILURE(observe(80, 0.2));
    EXPECT_NEAR(filtered->center_x_px, 80, 1e-5);
    ASSERT_NO_FATAL_FAILURE(observe(70, 0.3));
    EXPECT_GT(filtered->center_x_px, 79);
    info.k[0] = 200;
    infos->publish(info);
    drain();
    ASSERT_NO_FATAL_FAILURE(observe(60, 0.4));
    EXPECT_NEAR(filtered->center_x_px, 60, 1e-5);
    setMode(255);
    ASSERT_NO_FATAL_FAILURE(observe(70, 0.5));
    EXPECT_EQ(filtered->status, Detection::ERROR);
    setMode(1);
    ASSERT_NO_FATAL_FAILURE(observe(80, 0.6));
    EXPECT_NEAR(filtered->center_x_px, 80, 1e-5);
}

TEST_F(CenterFilterNode, RuntimeUpdatesAreValidatedAndResetOnlyWhenAccepted) {
    ASSERT_NO_FATAL_FAILURE(observe(60, 0));
    EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter("center_filter.enabled", false),
                                                  rclcpp::Parameter("center_filter.tau_x_s", 0.0)})
                     .successful);
    EXPECT_TRUE(node->get_parameter("center_filter.enabled").as_bool());
    EXPECT_FALSE(node->set_parameters_atomically(
                         {rclcpp::Parameter("center_filter.enabled", false),
                          rclcpp::Parameter("profiles.default.geometry.min_radius_px", -1.0)})
                     .successful);
    ASSERT_NO_FATAL_FAILURE(observe(70, 0.1));
    EXPECT_NEAR(filtered->center_x_px, 60 + 10 * (1 - std::exp(-0.1)), 1e-5);
    ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("center_filter.tau_x_s", 2.0)})
                    .successful);
    ASSERT_NO_FATAL_FAILURE(observe(80, 0.2));
    EXPECT_NEAR(filtered->center_x_px, 80, 1e-5);
    ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("center_filter.enabled", false)})
                    .successful);
    ASSERT_NO_FATAL_FAILURE(observe(60, 0.3));
    ASSERT_EQ(filtered->status, Detection::DETECTED);
    EXPECT_NEAR(filtered->center_x_px, 60, 1e-5);
    EXPECT_NEAR(filtered->unit_ray.x, 0, 1e-7);
    ASSERT_NO_FATAL_FAILURE(observe(80, 0.35));
    EXPECT_NEAR(filtered->center_x_px, 80, 1e-5);
    EXPECT_NEAR(filtered->unit_ray.x / filtered->unit_ray.z, 0.2, 1e-7);
    ASSERT_TRUE(node->set_parameters_atomically({rclcpp::Parameter("center_filter.enabled", true)})
                    .successful);
    ASSERT_NO_FATAL_FAILURE(observe(70, 0.4));
    EXPECT_NEAR(filtered->center_x_px, 70, 1e-5);
}

TEST_F(CenterFilterNode, RejectsInvalidStartupSettings) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("center_filter.tau_y_s", -1.0)});
    EXPECT_THROW({ GreenLightDetectorNode invalid(options); }, std::invalid_argument);
}

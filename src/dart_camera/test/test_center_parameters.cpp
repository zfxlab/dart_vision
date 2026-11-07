#include "dart_camera/green_light_detector_node.hpp"

#include <gtest/gtest.h>
#include <thread>

class CenterParameters : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(CenterParameters, AcceptsStartupOverrideAndRuntimeSwitch) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("profiles.default.center.method", "circle_fit")});
    auto node = std::make_shared<dart_vision::camera::GreenLightDetectorNode>(options);
    EXPECT_EQ(node->get_parameter("profiles.default.center.method").as_string(), "circle_fit");
    const auto result = node->set_parameters_atomically(
        {rclcpp::Parameter("profiles.default.center.method", "ellipse_fit")});
    EXPECT_TRUE(result.successful) << result.reason;
    EXPECT_EQ(node->get_parameter("profiles.default.center.method").as_string(), "ellipse_fit");
}

TEST_F(CenterParameters, RejectsUnknownMethodAndRollsBackInvalidBatch) {
    auto node =
        std::make_shared<dart_vision::camera::GreenLightDetectorNode>(rclcpp::NodeOptions{});
    EXPECT_FALSE(node->set_parameters_atomically(
                         {rclcpp::Parameter("profiles.default.center.method", "typo")})
                     .successful);
    EXPECT_EQ(node->get_parameter("profiles.default.center.method").as_string(),
              "brightness_weighted");
    EXPECT_FALSE(node->set_parameters_atomically(
                         {rclcpp::Parameter("profiles.default.center.method", "circle_fit"),
                          rclcpp::Parameter("profiles.default.geometry.min_radius_px", -1.0)})
                     .successful);
    EXPECT_EQ(node->get_parameter("profiles.default.center.method").as_string(),
              "brightness_weighted");
}

TEST_F(CenterParameters, RejectsUnknownMethodAtStartup) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("profiles.default.center.method", "typo")});
    EXPECT_THROW(
        { dart_vision::camera::GreenLightDetectorNode node(options); }, std::invalid_argument);
}

TEST_F(CenterParameters, RuntimeSwitchChangesPublishedCenterAndInvalidBatchLeavesItUnchanged) {
    using Detection = dart_interfaces::msg::GreenLightDetection;
    auto node =
        std::make_shared<dart_vision::camera::GreenLightDetectorNode>(rclcpp::NodeOptions{});
    auto input = std::make_shared<rclcpp::Node>("center_method_test_input");
    const auto qos = rclcpp::SensorDataQoS();
    auto images = input->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    auto infos = input->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    auto controllers =
        input->create_publisher<dart_interfaces::msg::ControllerState>("controller_state", qos);
    Detection::ConstSharedPtr last;
    auto detections = input->create_subscription<Detection>(
        "detection", qos,
        [&last](Detection::ConstSharedPtr message) { last = std::move(message); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(input);

    sensor_msgs::msg::Image image;
    image.header.frame_id = "test_optical_frame";
    image.width = image.height = 120;
    image.encoding = "bgr8";
    image.step = 360;
    image.data.resize(120 * 360, 0);
    for (int y = 45; y <= 75; ++y)
        for (int x = 45; x <= 75; ++x)
            if ((x - 60) * (x - 60) + (y - 60) * (y - 60) <= 225)
                image.data[y * image.step + x * 3 + 1] = 140 + 2 * (x - 60);
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = image.header.frame_id;
    info.width = info.height = 120;
    info.distortion_model = "plumb_bob";
    info.k = {100, 0, 60, 0, 100, 60, 0, 0, 1};
    dart_interfaces::msg::ControllerState controller;
    controller.target_mode = 1;

    const auto observe = [&](bool weighted) {
        last.reset();
        const auto started = input->now();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < deadline) {
            image.header.stamp = input->now();
            info.header.stamp = image.header.stamp;
            controller.header.stamp = image.header.stamp;
            infos->publish(info);
            controllers->publish(controller);
            images->publish(image);
            executor.spin_some();
            if (last && rclcpp::Time(last->header.stamp) >= started &&
                last->status == Detection::DETECTED &&
                (weighted ? last->center_x_px > 60.5F
                          : std::abs(last->center_x_px - 60.0F) < 1e-4F))
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    };
    ASSERT_TRUE(observe(true));
    EXPECT_GT(last->unit_ray.x, 0.005);
    ASSERT_TRUE(node->set_parameters_atomically(
                        {rclcpp::Parameter("profiles.default.center.method", "contour_centroid")})
                    .successful);
    ASSERT_TRUE(observe(false));
    EXPECT_NEAR(last->unit_ray.x, 0.0, 1e-7);
    EXPECT_FALSE(
        node->set_parameters_atomically(
                {rclcpp::Parameter("profiles.default.center.method", "brightness_weighted"),
                 rclcpp::Parameter("profiles.default.geometry.min_radius_px", -1.0)})
            .successful);
    EXPECT_TRUE(observe(false));
}

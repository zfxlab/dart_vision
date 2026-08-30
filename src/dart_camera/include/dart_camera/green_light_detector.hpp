#ifndef DART_CAMERA_GREEN_LIGHT_DETECTOR_HPP
#define DART_CAMERA_GREEN_LIGHT_DETECTOR_HPP

#include <cstddef>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace dart_vision::camera {

/// 二值掩膜的形态学清理参数。
struct MaskCleanupConfig {
    int close_kernel_size{3}; ///< 闭运算椭圆核的边长，0表示不进行，此外需为奇数。
    int close_iterations{1}; ///< 闭运算迭代次数。

    int open_kernel_size{0}; ///< 开运算椭圆核的边长，0表示不进行，此外需为奇数。
    int open_iterations{1}; ///< 开运算迭代次数。

    bool fill_holes{true}; ///< 是否填充前景区域内部完全封闭的孔洞。

    [[nodiscard]] bool isConfigValid() const;
};

/// 绿色圆形光源的分割与候选筛选参数。
struct GreenLightDetectorConfig {
    double min_hue{35.0};        ///< HSV 色相下限，采用 OpenCV 的 [0, 179] 范围。
    double max_hue{95.0};        ///< HSV 色相上限，采用 OpenCV 的 [0, 179] 范围。
    double min_saturation{45.0}; ///< HSV 饱和度下限，范围为 [0, 255]。
    double min_value{100.0};     ///< HSV 亮度及绿色通道亮度下限，范围为 [0, 255]。

    double min_green_excess{25.0}; ///< 最小绿色优势值，计算式为 2G - R - B。

    double min_radius_px{4.0};   ///< 候选最小外接圆半径下限。
    double max_radius_px{20.0};  ///< 候选最小外接圆半径上限。
    double min_circularity{0.7}; ///< 最小圆度；圆度定义为 4*pi*面积/周长^2。

    double max_aspect_ratio{1.55}; ///< 外接矩形长短边比上限，必须不小于 1。

    double min_fill_ratio{0.45}; ///< 轮廓面积与最小外接圆面积之比的下限。

    double min_inner_brightness{100.0}; ///< 候选圆内绿色通道的最小平均亮度。
    double min_contrast_ratio{1.5}; ///< 圆内与外围圆环平均绿色亮度之比的下限。

    MaskCleanupConfig cleanup; ///< 分割所得二值掩膜的清理参数。

    [[nodiscard]] bool isConfigValid() const;
};

/// 通过全部筛选条件的单个光源候选。
struct GreenLightCandidate {
    cv::Point2f center_px; ///< 候选区域内扣除局部背景后的绿色亮度加权中心。
    double radius_px{};    ///< 最小外接圆的半径。
    double area_px2{};     ///< 轮廓面积。

    double circularity{};  ///< 轮廓圆度 4*pi*面积/周长^2，越接近 1 越圆。
    double aspect_ratio{}; ///< 外接矩形的长边与短边之比。
    double fill_ratio{};   ///< 轮廓面积与最小外接圆面积之比。

    double mean_inner_brightness{}; ///< 最小外接圆内绿色通道的平均亮度。
    double mean_outer_brightness{}; ///< 外接圆半径 1 至 1.5 倍圆环内的平均绿色亮度。
    double contrast_ratio{};        ///< 圆内平均亮度与外围圆环平均亮度之比。

    double mean_radial_error_px{}; ///< 轮廓点到拟合圆的平均径向误差。
    double fit_score{}; ///< 圆拟合得分，范围为 [0, 1]，越大表示拟合越好。
};

/// 一帧图像的完整检测结果。
struct GreenLightDetectionResult {
    std::optional<GreenLightCandidate> target;   ///< 最优候选；没有合格候选时为空。
    std::vector<GreenLightCandidate> candidates; ///< 通过全部筛选条件的候选集合。
    std::size_t contours_count{}; ///< 具备有效轮廓几何量、进入参数筛选的候选数。
    cv::Mat binary_mask; ///< 完成形态学清理后的 CV_8UC1 二值掩膜。
};

class GreenLightDetector {
public:
    explicit GreenLightDetector(const GreenLightDetectorConfig& config);

    /**
     * @brief 检测一帧图像中的绿色圆形光源。
     *
     * @param image 非空的 CV_8UC3 BGR 或 CV_8UC4 BGRA 图像。
     * @return 最优目标、全部合格候选以及清理后的二值掩膜。
     * @throws std::invalid_argument 输入图像的类型或内容不合法。
     */
    [[nodiscard]] GreenLightDetectionResult detect(const cv::Mat& image) const;

private:
    GreenLightDetectorConfig config_;

    struct SegmentationResult {
        cv::Mat green_channel; ///< 原始 BGR 图像的绿色通道。
        cv::Mat binary_mask;   ///< HSV 与绿色优势分割结果的并集。
    };

    struct CandidateExtractionResult {
        std::vector<GreenLightCandidate> accepted_candidates;
        std::size_t contours_count{};
    };

    [[nodiscard]] cv::Mat normalizeInput(const cv::Mat& image) const;

    [[nodiscard]] SegmentationResult makeGreenMask(const cv::Mat& bgr_img) const;

    void cleanMask(cv::Mat& mask) const;

    [[nodiscard]] CandidateExtractionResult
    extractCandidates(const cv::Mat& binary_mask, const cv::Mat& green_channel) const;

    [[nodiscard]] std::optional<GreenLightCandidate>
    selectBestCandidate(const std::vector<GreenLightCandidate>& candidates) const;
};

} // namespace dart_vision::camera

#endif // DART_CAMERA_GREEN_LIGHT_DETECTOR_HPP

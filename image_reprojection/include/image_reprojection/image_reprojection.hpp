#pragma once

#include <array>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <image_transport/subscriber.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace image_reprojection {

class ImageReprojection : public rclcpp::Node {
 public:
  explicit ImageReprojection(const rclcpp::NodeOptions &options);

 private:
  struct InputCameraConfig {
    std::string name;
    std::string image_topic;
    std::string camera_info_topic;
  };

  struct CameraIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int width{0};
    int height{0};
  };

  struct BgrImage {
    int width{0};
    int height{0};
    std::vector<uint8_t> data;
  };

  struct FrameAccumulator {
    rclcpp::Time stamp;
    std::vector<bool> ready;
    std::vector<BgrImage> images;
    std::vector<CameraIntrinsics> intrinsics;
    std::vector<std::string> frame_ids;
    std::vector<rclcpp::Time> arrival_times;
    std::vector<rclcpp::Time> header_stamps;
  };

  struct PixelMapping {
    float u{std::numeric_limits<float>::quiet_NaN()};
    float v{std::numeric_limits<float>::quiet_NaN()};
  };

  enum class AggregationMode {
    WaitForAll,
    LeadWithLatest,
  };

  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  // No message_filters bundles; subscribe separately to images and camera infos

  void loadParameters();
  void setupSubscriptions();
  void configurePlanarCameraInfo();
  void configureEquirectCameraInfo();
  void imageCallback(size_t index, const Image::ConstSharedPtr &image);
  void cameraInfoCallback(size_t index, const CameraInfo::ConstSharedPtr &info);
  void cleanupAccumulators(const rclcpp::Time &current_stamp);
  void processFrame(int64_t frame_key,
                    FrameAccumulator &frame,
                    const std::vector<bool> &camera_mask);
  void processLeadCameraImage(size_t index,
                              const rclcpp::Time &stamp,
                              const rclcpp::Time &arrival_time,
                              BgrImage &&image);
  void exportGstConfigIfReady();
  void markGstConfigDirty();

  bool lookupCameraTransforms(const rclcpp::Time &stamp,
                              const std::vector<std::string> &camera_frames,
                              const std::string &target_frame,
                              std::vector<tf2::Transform> &transforms,
                              bool planar_projection,
                              const std::vector<bool> *camera_mask = nullptr);

  static bool toBgrImage(const Image::ConstSharedPtr &msg, BgrImage &output, const rclcpp::Logger &logger);
  static bool extractIntrinsics(const CameraInfo::ConstSharedPtr &info, CameraIntrinsics &intrinsics, const rclcpp::Logger &logger);
  bool reprojectPlanar(const std::vector<BgrImage> &input_images,
                       const std::vector<CameraIntrinsics> &intrinsics,
                       const std::vector<tf2::Transform> &transforms,
                       const std::vector<bool> *camera_mask,
                       sensor_msgs::msg::Image &output_image) const;
  bool reprojectEquirectangular(const std::vector<BgrImage> &input_images,
                                const std::vector<CameraIntrinsics> &intrinsics,
                                const std::vector<tf2::Transform> &transforms,
                                const std::vector<bool> *camera_mask,
                                sensor_msgs::msg::Image &output_image) const;
  static std::array<float, 3> bilinearSample(const BgrImage &image, double u, double v);
  void updatePlanarWarpCache(size_t index);
  void updateEquirectWarpCache(size_t index);
  void preparePlanarScratchBuffers(size_t camera_count) const;
  void prepareEquirectScratchBuffers(size_t camera_count) const;

  std::vector<InputCameraConfig> input_configs_{};

  AggregationMode aggregation_mode_{AggregationMode::WaitForAll};

  bool enable_planar_{true};
  bool enable_equirectangular_{false};

  std::string planar_image_topic_{};
  std::string planar_info_topic_{};
  int planar_width_{0};
  int planar_height_{0};
  double planar_fx_{0.0};
  double planar_fy_{0.0};
  double planar_cx_{0.0};
  double planar_cy_{0.0};
  double planar_depth_{1.0};
  double planar_blend_factor_{1.0};
  std::string planar_frame_id_{};

  std::string equirect_image_topic_{};
  std::string equirect_info_topic_{};
  int equirect_width_{0};
  int equirect_height_{0};
  double equirect_hfov_rad_{0.0};
  double equirect_vfov_rad_{0.0};
  double equirect_blend_factor_{1.0};
  std::string equirect_frame_id_{};
  double equirect_radius_{1.0};
  double transform_timeout_sec_{0.05};
  double accumulator_timeout_sec_{1.0};
  double frame_time_tolerance_sec_{0.005};
  bool recompute_every_frame_{false};

  std::vector<BgrImage> latest_images_;
  std::vector<bool> latest_image_ready_;
  std::vector<rclcpp::Time> latest_image_stamps_;
  std::vector<rclcpp::Time> latest_image_arrivals_;

  // Cached per-projection direction tables (target frame)
  std::vector<double> planar_x_norm_;
  std::vector<double> planar_y_norm_;
  std::vector<double> equirect_sin_lat_;
  std::vector<double> equirect_cos_lat_;
  std::vector<double> equirect_sin_lon_;
  std::vector<double> equirect_cos_lon_;

  // Camera static info and transforms
  std::vector<CameraIntrinsics> static_intrinsics_;
  std::vector<std::string> camera_frame_ids_;
  std::vector<bool> intrinsics_ready_;
  std::vector<tf2::Transform> cached_planar_transforms_;
  std::vector<tf2::Transform> cached_equirect_transforms_;
  std::vector<bool> planar_tf_ready_;
  std::vector<bool> equirect_tf_ready_;

  std::string gst_config_export_path_{};
  bool gst_config_dirty_{false};

  mutable std::vector<std::vector<float>> planar_accumulators_;
  mutable std::vector<std::vector<float>> planar_weights_;
  mutable std::vector<std::vector<float>> equirect_accumulators_;
  mutable std::vector<std::vector<float>> equirect_weights_;

  std::vector<std::vector<PixelMapping>> planar_warp_maps_;
  std::vector<std::vector<PixelMapping>> equirect_warp_maps_;
  std::vector<bool> planar_warp_ready_;
  std::vector<bool> equirect_warp_ready_;

  sensor_msgs::msg::CameraInfo planar_camera_info_{};
  sensor_msgs::msg::CameraInfo equirect_camera_info_{};

  // Subscriptions
  rclcpp::TimerBase::SharedPtr setup_timer_;
  std::vector<image_transport::Subscriber> image_subs_{};
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> info_subs_{};
  std::map<int64_t, FrameAccumulator> frame_accumulators_{};

  rclcpp::Publisher<Image>::SharedPtr planar_image_publisher_{};
  rclcpp::Publisher<CameraInfo>::SharedPtr planar_info_publisher_{};
  rclcpp::Publisher<Image>::SharedPtr equirect_image_publisher_{};
  rclcpp::Publisher<CameraInfo>::SharedPtr equirect_info_publisher_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
};

}  // namespace image_reprojection

#pragma once

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
  };

  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;

  struct CameraBundle {
    std::shared_ptr<message_filters::Subscriber<Image>> image_subscriber;
    std::shared_ptr<message_filters::Subscriber<CameraInfo>> info_subscriber;
    std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>> synchronizer;
  };

  void loadParameters();
  void setupSubscriptions();
  void configurePlanarCameraInfo();
  void configureEquirectCameraInfo();
  void handleCameraUpdate(size_t index,
                          const Image::ConstSharedPtr &image,
                          const CameraInfo::ConstSharedPtr &info);
  void cleanupAccumulators(const rclcpp::Time &current_stamp);

  bool lookupCameraTransforms(const rclcpp::Time &stamp,
                              const std::vector<std::string> &camera_frames,
                              const std::string &target_frame,
                              std::vector<tf2::Transform> &transforms);

  static bool toBgrImage(const Image::ConstSharedPtr &msg, BgrImage &output, const rclcpp::Logger &logger);
  static bool extractIntrinsics(const CameraInfo::ConstSharedPtr &info, CameraIntrinsics &intrinsics, const rclcpp::Logger &logger);
  bool reprojectPlanar(const std::vector<BgrImage> &input_images,
                       const std::vector<CameraIntrinsics> &intrinsics,
                       const std::vector<tf2::Transform> &transforms,
                       sensor_msgs::msg::Image &output_image) const;
  bool reprojectEquirectangular(const std::vector<BgrImage> &input_images,
                                const std::vector<CameraIntrinsics> &intrinsics,
                                const std::vector<tf2::Transform> &transforms,
                                sensor_msgs::msg::Image &output_image) const;
  static std::array<float, 3> bilinearSample(const BgrImage &image, double u, double v);

  std::vector<InputCameraConfig> input_configs_{};

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
  int sync_queue_size_{10};
  double transform_timeout_sec_{0.05};
  double accumulator_timeout_sec_{1.0};
  double frame_time_tolerance_sec_{0.005};

  sensor_msgs::msg::CameraInfo planar_camera_info_{};
  sensor_msgs::msg::CameraInfo equirect_camera_info_{};

  std::vector<CameraBundle> camera_bundles_{};
  std::map<int64_t, FrameAccumulator> frame_accumulators_{};

  rclcpp::Publisher<Image>::SharedPtr planar_image_publisher_{};
  rclcpp::Publisher<CameraInfo>::SharedPtr planar_info_publisher_{};
  rclcpp::Publisher<Image>::SharedPtr equirect_image_publisher_{};
  rclcpp::Publisher<CameraInfo>::SharedPtr equirect_info_publisher_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
};

}  // namespace image_reprojection

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2/LinearMath/Matrix3x3.h>

namespace image_reprojection {

/**
 * @brief Node that reprojects multiple camera images into the image plane of a virtual camera.
 */
class ImageReprojection : public rclcpp::Node {
 public:
  explicit ImageReprojection(const rclcpp::NodeOptions &options);

 private:
  struct InputCameraConfig {
    std::string image_topic;
    std::string camera_info_topic;
    tf2::Matrix3x3 rotation_virtual_to_input;
    std::string name;
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

  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo>;

  void loadParameters();
  void setupSubscriptions();
  void configureOutputCameraInfo();
  tf2::Matrix3x3 quaternionToRotation(const std::vector<double> &quaternion) const;
  void synchronizedCallback(const Image::ConstSharedPtr &image0,
                            const CameraInfo::ConstSharedPtr &info0,
                            const Image::ConstSharedPtr &image1,
                            const CameraInfo::ConstSharedPtr &info1);

  static bool toBgrImage(const Image::ConstSharedPtr &msg, BgrImage &output, const rclcpp::Logger &logger);
  static bool extractIntrinsics(const CameraInfo::ConstSharedPtr &info, CameraIntrinsics &intrinsics, const rclcpp::Logger &logger);
  bool reprojectImages(const std::array<BgrImage, 2> &input_images,
                       const std::array<CameraIntrinsics, 2> &intrinsics,
                       sensor_msgs::msg::Image &output_image) const;
  static std::array<float, 3> bilinearSample(const BgrImage &image, double u, double v);

  std::array<InputCameraConfig, 2> input_configs_{};
  std::string output_image_topic_{};
  std::string output_info_topic_{};
  std::string output_frame_id_{};
  int output_width_{0};
  int output_height_{0};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  int sync_queue_size_{10};

  sensor_msgs::msg::CameraInfo output_camera_info_{};

  std::array<std::shared_ptr<message_filters::Subscriber<Image>>, 2> image_subscribers_{};
  std::array<std::shared_ptr<message_filters::Subscriber<CameraInfo>>, 2> camera_info_subscribers_{};
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_{};

  rclcpp::Publisher<Image>::SharedPtr output_image_publisher_{};
  rclcpp::Publisher<CameraInfo>::SharedPtr output_info_publisher_{};
};

}  // namespace image_reprojection

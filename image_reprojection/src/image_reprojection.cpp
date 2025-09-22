#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <image_reprojection/image_reprojection.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <rmw/qos_profiles.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace image_reprojection {

namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

ImageReprojection::ImageReprojection(const rclcpp::NodeOptions &options)
    : rclcpp::Node("image_reprojection", options) {
  loadParameters();
  configureOutputCameraInfo();

  output_image_publisher_ = this->create_publisher<Image>(output_image_topic_, rclcpp::SensorDataQoS());

  auto info_qos = rclcpp::QoS(10);
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).transient_local();
  output_info_publisher_ = this->create_publisher<CameraInfo>(output_info_topic_, info_qos);

  setupSubscriptions();

  RCLCPP_INFO(get_logger(),
              "Image reprojection node initialised. Publishing reprojected images on '%s' with frame_id '%s'",
              output_image_topic_.c_str(), output_frame_id_.c_str());
}

void ImageReprojection::loadParameters() {
  input_configs_[0].name = "input0";
  input_configs_[1].name = "input1";

  input_configs_[0].image_topic = this->declare_parameter<std::string>("input0.image_topic", "~/input0/image");
  input_configs_[0].camera_info_topic = this->declare_parameter<std::string>("input0.camera_info_topic", "~/input0/camera_info");
  auto quat0 = this->declare_parameter<std::vector<double>>("input0.rotation_quaternion", {0.0, 0.0, 0.0, 1.0});
  input_configs_[0].rotation_virtual_to_input = quaternionToRotation(quat0);

  input_configs_[1].image_topic = this->declare_parameter<std::string>("input1.image_topic", "~/input1/image");
  input_configs_[1].camera_info_topic = this->declare_parameter<std::string>("input1.camera_info_topic", "~/input1/camera_info");
  auto quat1 = this->declare_parameter<std::vector<double>>("input1.rotation_quaternion", {0.0, 0.0, 0.0, 1.0});
  input_configs_[1].rotation_virtual_to_input = quaternionToRotation(quat1);

  sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 10);
  if (sync_queue_size_ < 2) {
    RCLCPP_WARN(get_logger(), "sync_queue_size must be >= 2. Using 2 instead of %d.", sync_queue_size_);
    sync_queue_size_ = 2;
  }

  output_image_topic_ = this->declare_parameter<std::string>("output.image_topic", "~/output/image");
  output_info_topic_ = this->declare_parameter<std::string>("output.camera_info_topic", "~/output/camera_info");

  output_frame_id_ = this->declare_parameter<std::string>("virtual_camera.frame_id", "virtual_camera");
  output_width_ = this->declare_parameter<int>("virtual_camera.width", 1280);
  output_height_ = this->declare_parameter<int>("virtual_camera.height", 720);
  fx_ = this->declare_parameter<double>("virtual_camera.fx", 800.0);
  fy_ = this->declare_parameter<double>("virtual_camera.fy", 800.0);

  const double cx_default = output_width_ > 0 ? static_cast<double>(output_width_) * 0.5 : 0.0;
  const double cy_default = output_height_ > 0 ? static_cast<double>(output_height_) * 0.5 : 0.0;
  cx_ = this->declare_parameter<double>("virtual_camera.cx", cx_default);
  cy_ = this->declare_parameter<double>("virtual_camera.cy", cy_default);

  if (output_width_ <= 0 || output_height_ <= 0) {
    throw std::runtime_error("virtual_camera width and height must be positive");
  }
  if (fx_ <= kEpsilon || fy_ <= kEpsilon) {
    throw std::runtime_error("virtual_camera focal lengths must be positive");
  }
}

void ImageReprojection::setupSubscriptions() {
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto info_qos = rclcpp::QoS(10);
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  for (size_t i = 0; i < input_configs_.size(); ++i) {
    image_subscribers_[i] = std::make_shared<message_filters::Subscriber<Image>>(this, input_configs_[i].image_topic, sensor_qos.get_rmw_qos_profile());
    camera_info_subscribers_[i] = std::make_shared<message_filters::Subscriber<CameraInfo>>(this, input_configs_[i].camera_info_topic, info_qos.get_rmw_qos_profile());

    RCLCPP_INFO(get_logger(), "Subscribed to image topic '%s' and camera info topic '%s'",
                input_configs_[i].image_topic.c_str(), input_configs_[i].camera_info_topic.c_str());
  }

  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), *image_subscribers_[0], *camera_info_subscribers_[0], *image_subscribers_[1], *camera_info_subscribers_[1]);

  synchronizer_->registerCallback(std::bind(&ImageReprojection::synchronizedCallback,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2,
                                            std::placeholders::_3,
                                            std::placeholders::_4));
}

void ImageReprojection::configureOutputCameraInfo() {
  output_camera_info_.height = static_cast<uint32_t>(output_height_);
  output_camera_info_.width = static_cast<uint32_t>(output_width_);
  output_camera_info_.distortion_model = "plumb_bob";
  output_camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  output_camera_info_.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
  output_camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  output_camera_info_.p = {fx_, 0.0, cx_, 0.0,
                           0.0, fy_, cy_, 0.0,
                           0.0, 0.0, 1.0, 0.0};
  output_camera_info_.binning_x = 1;
  output_camera_info_.binning_y = 1;
  output_camera_info_.roi.x_offset = 0;
  output_camera_info_.roi.y_offset = 0;
  output_camera_info_.roi.height = 0;
  output_camera_info_.roi.width = 0;
  output_camera_info_.roi.do_rectify = false;
}

tf2::Matrix3x3 ImageReprojection::quaternionToRotation(const std::vector<double> &quaternion) const {
  if (quaternion.size() != 4) {
    RCLCPP_WARN(get_logger(), "Quaternion parameter must contain exactly 4 elements (x, y, z, w). Using identity.");
    return tf2::Matrix3x3(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  }

  tf2::Quaternion q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  if (q.length2() < kEpsilon) {
    RCLCPP_WARN(get_logger(), "Quaternion parameter has near-zero length. Using identity.");
    q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }
  return tf2::Matrix3x3(q);
}

void ImageReprojection::synchronizedCallback(const Image::ConstSharedPtr &image0,
                                              const CameraInfo::ConstSharedPtr &info0,
                                              const Image::ConstSharedPtr &image1,
                                              const CameraInfo::ConstSharedPtr &info1) {
  std::array<BgrImage, 2> input_images;
  if (!toBgrImage(image0, input_images[0], get_logger()) || !toBgrImage(image1, input_images[1], get_logger())) {
    return;
  }

  std::array<CameraIntrinsics, 2> intrinsics{};
  if (!extractIntrinsics(info0, intrinsics[0], get_logger()) || !extractIntrinsics(info1, intrinsics[1], get_logger())) {
    return;
  }

  sensor_msgs::msg::Image output_image;
  if (!reprojectImages(input_images, intrinsics, output_image)) {
    return;
  }

  output_image.header.stamp = image0->header.stamp;
  output_image.header.frame_id = output_frame_id_;

  output_camera_info_.header.stamp = output_image.header.stamp;
  output_camera_info_.header.frame_id = output_frame_id_;

  output_image_publisher_->publish(output_image);
  output_info_publisher_->publish(output_camera_info_);
}

bool ImageReprojection::toBgrImage(const Image::ConstSharedPtr &msg, BgrImage &output, const rclcpp::Logger &logger) {
  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  if (width <= 0 || height <= 0) {
    RCLCPP_ERROR(logger, "Received image with non-positive dimensions (%d x %d).", width, height);
    return false;
  }

  const auto encoding = msg->encoding;
  const int src_channels =
      (encoding == sensor_msgs::image_encodings::BGR8 || encoding == sensor_msgs::image_encodings::RGB8) ? 3 :
      (encoding == sensor_msgs::image_encodings::MONO8 ? 1 :
       (encoding == sensor_msgs::image_encodings::BGRA8 || encoding == sensor_msgs::image_encodings::RGBA8 ? 4 : 0));

  if (src_channels == 0) {
    RCLCPP_ERROR(logger,
                 "Unsupported image encoding '%s'. Supported encodings: BGR8, RGB8, BGRA8, RGBA8, MONO8.",
                 encoding.c_str());
    return false;
  }

  const size_t src_stride = msg->step;
  const size_t required_bytes_per_row = static_cast<size_t>(width) * src_channels;
  if (src_stride < required_bytes_per_row) {
    RCLCPP_ERROR(logger, "Image stride (%zu) is smaller than expected row size (%zu).", src_stride, required_bytes_per_row);
    return false;
  }

  output.width = width;
  output.height = height;
  output.data.assign(static_cast<size_t>(width) * height * 3, 0);

  const uint8_t *src_data = msg->data.data();
  uint8_t *dst_data = output.data.data();
  const size_t dst_stride = static_cast<size_t>(width) * 3;

  if (encoding == sensor_msgs::image_encodings::BGR8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t *src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t *dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
      std::copy(src_row, src_row + dst_stride, dst_row);
    }
    return true;
  }

  if (encoding == sensor_msgs::image_encodings::RGB8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t *src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t *dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
      for (int col = 0; col < width; ++col) {
        const size_t src_index = static_cast<size_t>(col) * 3;
        dst_row[src_index + 0] = src_row[src_index + 2];
        dst_row[src_index + 1] = src_row[src_index + 1];
        dst_row[src_index + 2] = src_row[src_index + 0];
      }
    }
    return true;
  }

  if (encoding == sensor_msgs::image_encodings::BGRA8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t *src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t *dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
      for (int col = 0; col < width; ++col) {
        const size_t src_index = static_cast<size_t>(col) * 4;
        const size_t dst_index = static_cast<size_t>(col) * 3;
        dst_row[dst_index + 0] = src_row[src_index + 0];
        dst_row[dst_index + 1] = src_row[src_index + 1];
        dst_row[dst_index + 2] = src_row[src_index + 2];
      }
    }
    return true;
  }

  if (encoding == sensor_msgs::image_encodings::RGBA8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t *src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t *dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
      for (int col = 0; col < width; ++col) {
        const size_t src_index = static_cast<size_t>(col) * 4;
        const size_t dst_index = static_cast<size_t>(col) * 3;
        dst_row[dst_index + 0] = src_row[src_index + 2];
        dst_row[dst_index + 1] = src_row[src_index + 1];
        dst_row[dst_index + 2] = src_row[src_index + 0];
      }
    }
    return true;
  }

  if (encoding == sensor_msgs::image_encodings::MONO8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t *src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t *dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
      for (int col = 0; col < width; ++col) {
        const uint8_t value = src_row[col];
        const size_t dst_index = static_cast<size_t>(col) * 3;
        dst_row[dst_index + 0] = value;
        dst_row[dst_index + 1] = value;
        dst_row[dst_index + 2] = value;
      }
    }
    return true;
  }

  RCLCPP_ERROR(logger,
               "Unsupported image encoding '%s'. Supported encodings: BGR8, RGB8, BGRA8, RGBA8, MONO8.",
               encoding.c_str());
  return false;
}

bool ImageReprojection::extractIntrinsics(const CameraInfo::ConstSharedPtr &info,
                                          CameraIntrinsics &intrinsics,
                                          const rclcpp::Logger &logger) {
  if (info->k.size() != 9) {
    RCLCPP_ERROR(logger, "CameraInfo K matrix must have 9 elements.");
    return false;
  }

  intrinsics.fx = info->k[0];
  intrinsics.fy = info->k[4];
  intrinsics.cx = info->k[2];
  intrinsics.cy = info->k[5];
  intrinsics.width = static_cast<int>(info->width);
  intrinsics.height = static_cast<int>(info->height);

  if (intrinsics.fx <= kEpsilon || intrinsics.fy <= kEpsilon) {
    RCLCPP_ERROR(logger, "CameraInfo focal lengths must be positive.");
    return false;
  }

  if (intrinsics.width <= 0 || intrinsics.height <= 0) {
    RCLCPP_ERROR(logger, "CameraInfo width and height must be positive.");
    return false;
  }

  return true;
}

bool ImageReprojection::reprojectImages(const std::array<BgrImage, 2> &input_images,
                                        const std::array<CameraIntrinsics, 2> &intrinsics,
                                        sensor_msgs::msg::Image &output_image) const {
  const int width = output_width_;
  const int height = output_height_;

  std::vector<float> accumulator(static_cast<size_t>(height) * width * 3, 0.0f);
  std::vector<float> weights(static_cast<size_t>(height) * width, 0.0f);

  std::vector<double> x_norm(width);
  std::vector<double> y_norm(height);
  const double inv_fx = 1.0 / fx_;
  const double inv_fy = 1.0 / fy_;
  for (int u = 0; u < width; ++u) {
    x_norm[u] = (static_cast<double>(u) - cx_) * inv_fx;
  }
  for (int v = 0; v < height; ++v) {
    y_norm[v] = (static_cast<double>(v) - cy_) * inv_fy;
  }

  for (size_t i = 0; i < input_configs_.size(); ++i) {
    const auto &rotation = input_configs_[i].rotation_virtual_to_input;
    const auto &intr = intrinsics[i];
    const BgrImage &image = input_images[i];

    const double fx_in = intr.fx;
    const double fy_in = intr.fy;
    const double cx_in = intr.cx;
    const double cy_in = intr.cy;
    const int width_in = image.width;
    const int height_in = image.height;

    if (intr.width != width_in || intr.height != height_in) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "CameraInfo resolution (%dx%d) differs from image resolution (%dx%d). Using image resolution for bounds.",
                       intr.width,
                       intr.height,
                       width_in,
                       height_in);
    }

    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        tf2::Vector3 dir_virtual(x_norm[u], y_norm[v], 1.0);
        tf2::Vector3 dir_input = rotation * dir_virtual;

        const double z = dir_input.z();
        if (z <= kEpsilon) {
          continue;
        }

        const double inv_z = 1.0 / z;
        const double u_in = fx_in * (dir_input.x() * inv_z) + cx_in;
        const double v_in = fy_in * (dir_input.y() * inv_z) + cy_in;

        if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) ||
            v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, u_in, v_in);
        const size_t base_index = (static_cast<size_t>(v) * width + u) * 3;
        accumulator[base_index + 0] += colour[0];
        accumulator[base_index + 1] += colour[1];
        accumulator[base_index + 2] += colour[2];
        weights[static_cast<size_t>(v) * width + u] += 1.0f;
      }
    }
  }

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.resize(output_image.step * output_image.height);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const float weight = weights[static_cast<size_t>(v) * width + u];
      uint8_t *pixel = &output_image.data[(static_cast<size_t>(v) * width + u) * 3];
      if (weight > 0.0f) {
        const size_t base_index = (static_cast<size_t>(v) * width + u) * 3;
        const float b = accumulator[base_index + 0] / weight;
        const float g = accumulator[base_index + 1] / weight;
        const float r = accumulator[base_index + 2] / weight;
        pixel[0] = static_cast<uint8_t>(std::clamp(b, 0.0f, 255.0f));
        pixel[1] = static_cast<uint8_t>(std::clamp(g, 0.0f, 255.0f));
        pixel[2] = static_cast<uint8_t>(std::clamp(r, 0.0f, 255.0f));
      } else {
        pixel[0] = pixel[1] = pixel[2] = 0;
      }
    }
  }

  return true;
}

std::array<float, 3> ImageReprojection::bilinearSample(const BgrImage &image, double u, double v) {
  const int width = image.width;
  const int height = image.height;

  const auto clamp_coord = [](int value, int max_value) {
    return std::max(0, std::min(value, max_value));
  };

  const int x0 = clamp_coord(static_cast<int>(std::floor(u)), width - 1);
  const int y0 = clamp_coord(static_cast<int>(std::floor(v)), height - 1);
  const int x1 = clamp_coord(x0 + 1, width - 1);
  const int y1 = clamp_coord(y0 + 1, height - 1);

  const double dx = u - static_cast<double>(x0);
  const double dy = v - static_cast<double>(y0);

  const auto get_pixel = [&](int x, int y) {
    const size_t index = (static_cast<size_t>(y) * width + x) * 3;
    return std::array<float, 3>{
        static_cast<float>(image.data[index + 0]),
        static_cast<float>(image.data[index + 1]),
        static_cast<float>(image.data[index + 2])};
  };

  const auto c00 = get_pixel(x0, y0);
  const auto c10 = get_pixel(x1, y0);
  const auto c01 = get_pixel(x0, y1);
  const auto c11 = get_pixel(x1, y1);

  std::array<float, 3> result{};
  for (size_t channel = 0; channel < 3; ++channel) {
    const float interp_x0 = static_cast<float>((1.0 - dx) * c00[channel] + dx * c10[channel]);
    const float interp_x1 = static_cast<float>((1.0 - dx) * c01[channel] + dx * c11[channel]);
    result[channel] = static_cast<float>((1.0 - dy) * interp_x0 + dy * interp_x1);
  }

  return result;
}

}  // namespace image_reprojection

RCLCPP_COMPONENTS_REGISTER_NODE(image_reprojection::ImageReprojection)

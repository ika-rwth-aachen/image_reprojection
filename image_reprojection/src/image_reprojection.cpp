#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <image_reprojection/image_reprojection.hpp>
#include <image_transport/image_transport.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <rmw/qos_profiles.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace image_reprojection {

namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kDefaultAccumulatorTimeoutSec = 1.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi * 0.5;
constexpr double kTwoPi = kPi * 2.0;

}  // namespace

ImageReprojection::ImageReprojection(const rclcpp::NodeOptions &options)
    : rclcpp::Node("image_reprojection", options) {
  loadParameters();
  if (enable_planar_) {
    configurePlanarCameraInfo();
  }
  if (enable_equirectangular_) {
    configureEquirectCameraInfo();
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto info_qos = rclcpp::QoS(10);
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).transient_local();

  if (enable_planar_) {
    planar_image_publisher_ = this->create_publisher<Image>(planar_image_topic_, rclcpp::SensorDataQoS());
    planar_info_publisher_ = this->create_publisher<CameraInfo>(planar_info_topic_, info_qos);
  }

  if (enable_equirectangular_) {
    equirect_image_publisher_ = this->create_publisher<Image>(equirect_image_topic_, rclcpp::SensorDataQoS());
    equirect_info_publisher_ = this->create_publisher<CameraInfo>(equirect_info_topic_, info_qos);
  }

  std::vector<std::string> enabled_projections;
  if (enable_planar_) enabled_projections.emplace_back("planar");
  if (enable_equirectangular_) enabled_projections.emplace_back("equirectangular");

  std::string projection_list;
  for (size_t i = 0; i < enabled_projections.size(); ++i) {
    projection_list += enabled_projections[i];
    if (i + 1 < enabled_projections.size()) {
      projection_list += ", ";
    }
  }

  std::string frame_info;
  if (enable_planar_) {
    frame_info += "planar:" + (planar_frame_id_.empty() ? std::string("<dynamic>") : planar_frame_id_);
  }
  if (enable_equirectangular_) {
    if (!frame_info.empty()) frame_info += ", ";
    frame_info += "equirect:" + (equirect_frame_id_.empty() ? std::string("<dynamic>") : equirect_frame_id_);
  }

  RCLCPP_INFO(get_logger(),
              "Image reprojection node initialised with %zu inputs. Projections: %s (%s).",
              input_configs_.size(),
              projection_list.c_str(),
              frame_info.empty() ? "n/a" : frame_info.c_str());

  // run setup after constructor has finished to enable shared_from_this()
  setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
    setupSubscriptions();
    setup_timer_->cancel();
  });
}

void ImageReprojection::loadParameters() {
  sync_queue_size_ = this->declare_parameter<int>("params.sync_queue_size", 10);
  if (sync_queue_size_ < 2) {
    RCLCPP_WARN(get_logger(), "sync_queue_size must be >= 2. Using 2 instead of %d.", sync_queue_size_);
    sync_queue_size_ = 2;
  }

  accumulator_timeout_sec_ = this->declare_parameter<double>("params.frame_timeout", kDefaultAccumulatorTimeoutSec);
  if (accumulator_timeout_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "frame_timeout must be non-negative. Using %.2f instead of %.2f.",
                kDefaultAccumulatorTimeoutSec,
                accumulator_timeout_sec_);
    accumulator_timeout_sec_ = kDefaultAccumulatorTimeoutSec;
  }

  transform_timeout_sec_ = this->declare_parameter<double>("params.transform_timeout", 0.05);
  if (transform_timeout_sec_ < 0.0) {
    throw std::runtime_error("transform_timeout must be non-negative");
  }

  frame_time_tolerance_sec_ = this->declare_parameter<double>("params.frame_time_tolerance", 0.005);
  if (frame_time_tolerance_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "frame_time_tolerance must be non-negative. Using 0.0 instead of %.6f.", frame_time_tolerance_sec_);
    frame_time_tolerance_sec_ = 0.0;
  }

  enable_planar_ = this->declare_parameter<bool>("output.projection.planar.enabled", true);
  enable_equirectangular_ = this->declare_parameter<bool>("output.projection.equirectangular.enabled", false);

  if (!enable_planar_ && !enable_equirectangular_) {
    throw std::runtime_error("At least one projection (planar or equirectangular) must be enabled");
  }

  if (enable_planar_) {
    planar_image_topic_ = this->declare_parameter<std::string>("output.projection.planar.image_topic", "~/output/planar/image");
    planar_info_topic_ = this->declare_parameter<std::string>("output.projection.planar.camera_info_topic", "~/output/planar/camera_info");
    planar_frame_id_ = this->declare_parameter<std::string>("output.projection.planar.optical_frame_id", "");
    planar_width_ = this->declare_parameter<int>("output.projection.planar.width", 1280);
    planar_height_ = this->declare_parameter<int>("output.projection.planar.height", 720);
    planar_depth_ = this->declare_parameter<double>("output.projection.planar.depth", 1.0);
    planar_blend_factor_ = this->declare_parameter<double>("output.projection.planar.blend_factor", 1.0);
    const double planar_fov_x_deg = this->declare_parameter<double>("output.projection.planar.fov_x", 90.0);

    if (planar_width_ <= 0 || planar_height_ <= 0) {
      throw std::runtime_error("output.projection.planar width and height must be positive");
    }
    if (planar_depth_ <= kEpsilon) {
      throw std::runtime_error("output.projection.planar.depth must be positive");
    }

    const double planar_fov_x_rad = std::clamp(planar_fov_x_deg, 1.0, 179.0) * kPi / 180.0;
    const double half_width = static_cast<double>(planar_width_) * 0.5;
    planar_fx_ = half_width / std::tan(planar_fov_x_rad * 0.5);
    planar_fy_ = planar_fx_;  // square pixels assumption
    planar_cx_ = half_width;
    planar_cy_ = static_cast<double>(planar_height_) * 0.5;
    if (!std::isfinite(planar_blend_factor_)) {
      planar_blend_factor_ = 1.0;
    }
    planar_blend_factor_ = std::clamp(planar_blend_factor_, 0.0, 1.0);
  }

  if (enable_equirectangular_) {
    equirect_image_topic_ = this->declare_parameter<std::string>("output.projection.equirectangular.image_topic", "~/output/equirectangular/image");
    equirect_info_topic_ = this->declare_parameter<std::string>("output.projection.equirectangular.camera_info_topic", "~/output/equirectangular/camera_info");
    equirect_frame_id_ = this->declare_parameter<std::string>("output.projection.equirectangular.optical_frame_id", "");
    equirect_width_ = this->declare_parameter<int>("output.projection.equirectangular.width", 2048);
    equirect_height_ = this->declare_parameter<int>("output.projection.equirectangular.height", 1024);
    equirect_radius_ = this->declare_parameter<double>("output.projection.equirectangular.radius", enable_planar_ ? planar_depth_ : 1.0);
    equirect_blend_factor_ = this->declare_parameter<double>("output.projection.equirectangular.blend_factor", 1.0);
    const double equirect_fov_x_deg = this->declare_parameter<double>("output.projection.equirectangular.fov_x", 360.0);

    if (equirect_width_ <= 0 || equirect_height_ <= 0) {
      throw std::runtime_error("output.projection.equirectangular width and height must be positive");
    }
    if (equirect_radius_ <= kEpsilon) {
      throw std::runtime_error("output.projection.equirectangular.radius must be positive");
    }

    const double hfov_clamped_deg = std::clamp(equirect_fov_x_deg, 1.0, 360.0);
    equirect_hfov_rad_ = hfov_clamped_deg * kPi / 180.0;
    const double aspect = static_cast<double>(equirect_height_) / static_cast<double>(equirect_width_);
    equirect_vfov_rad_ = equirect_hfov_rad_ * aspect;  // linear degrees-per-pixel mapping
    // clamp vertical FoV to sensible range (0, pi]
    if (equirect_vfov_rad_ > kPi) equirect_vfov_rad_ = kPi;
    if (equirect_vfov_rad_ < kEpsilon) equirect_vfov_rad_ = kEpsilon;
    if (!std::isfinite(equirect_blend_factor_)) {
      equirect_blend_factor_ = 1.0;
    }
    equirect_blend_factor_ = std::clamp(equirect_blend_factor_, 0.0, 1.0);
  }

  recompute_every_frame_ = this->declare_parameter<bool>("params.recompute_every_frame", false);

  const auto image_topics = this->declare_parameter<std::vector<std::string>>("input.image_topics", std::vector<std::string>{});
  if (image_topics.empty()) {
    throw std::runtime_error("input.image_topics must contain at least one topic");
  }

  input_configs_.clear();
  input_configs_.reserve(image_topics.size());

  for (size_t i = 0; i < image_topics.size(); ++i) {
    const auto &topic = image_topics[i];
    InputCameraConfig config;
    config.name = "input_" + std::to_string(i);
    config.image_topic = topic;

    const std::string camera_info_param = "input." + topic + ".camera_info_topic";
    if (this->has_parameter(camera_info_param)) {
      config.camera_info_topic = this->get_parameter(camera_info_param).as_string();
    } else {
      config.camera_info_topic = this->declare_parameter<std::string>(camera_info_param, "");
    }

    if (config.camera_info_topic.empty()) {
      throw std::runtime_error("Missing camera_info_topic parameter for input image topic '" + topic + "'");
    }

    input_configs_.push_back(std::move(config));
  }

  if (enable_planar_) {
    if (planar_width_ <= 0 || planar_height_ <= 0) {
      throw std::runtime_error("virtual_camera width and height must be positive");
    }
    if (planar_fx_ <= kEpsilon || planar_fy_ <= kEpsilon) {
      throw std::runtime_error("virtual_camera focal lengths must be positive");
    }
  }
}

void ImageReprojection::setupSubscriptions() {
  if (input_configs_.empty()) {
    throw std::runtime_error("No input cameras configured");
  }

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto info_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  const size_t n = input_configs_.size();
  image_subs_.resize(n);
  info_subs_.resize(n);
  static_intrinsics_.assign(n, CameraIntrinsics{});
  camera_frame_ids_.assign(n, std::string{});
  intrinsics_ready_.assign(n, false);
  cached_planar_transforms_.assign(n, tf2::Transform::getIdentity());
  cached_equirect_transforms_.assign(n, tf2::Transform::getIdentity());
  planar_tf_ready_.assign(n, false);
  equirect_tf_ready_.assign(n, false);

  image_transport::ImageTransport it(this->shared_from_this());

  for (size_t i = 0; i < n; ++i) {
    const auto idx = i;

    std::string image_transport_param_name = "input." + input_configs_[i].image_topic + ".image_transport";
    this->declare_parameter<std::string>(image_transport_param_name, "raw"); // TransportHints does not automatically declare the parameter
    image_transport::TransportHints hints{this, "raw", image_transport_param_name};
    image_subs_[i] = it.subscribe(
        input_configs_[i].image_topic,
        sensor_qos.get_rmw_qos_profile(),
        [this, idx](const Image::ConstSharedPtr &msg) { this->imageCallback(idx, msg); },
        std::shared_ptr<void>(),
        &hints,
        rclcpp::SubscriptionOptions());

    info_subs_[i] = this->create_subscription<CameraInfo>(
        input_configs_[i].camera_info_topic, info_qos,
        [this, idx](const CameraInfo::ConstSharedPtr &msg) { this->cameraInfoCallback(idx, msg); });

    RCLCPP_INFO(get_logger(),
                "Subscribed to image '%s' and camera_info '%s'",
                input_configs_[i].image_topic.c_str(),
                input_configs_[i].camera_info_topic.c_str());
  }
}

void ImageReprojection::configurePlanarCameraInfo() {
  planar_camera_info_.height = static_cast<uint32_t>(planar_height_);
  planar_camera_info_.width = static_cast<uint32_t>(planar_width_);
  planar_camera_info_.distortion_model = "plumb_bob";
  planar_camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  planar_camera_info_.k = {planar_fx_, 0.0, planar_cx_, 0.0, planar_fy_, planar_cy_, 0.0, 0.0, 1.0};
  planar_camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  planar_camera_info_.p = {planar_fx_, 0.0, planar_cx_, 0.0,
                           0.0, planar_fy_, planar_cy_, 0.0,
                           0.0, 0.0, 1.0, 0.0};
  planar_camera_info_.binning_x = 1;
  planar_camera_info_.binning_y = 1;
  planar_camera_info_.roi.x_offset = 0;
  planar_camera_info_.roi.y_offset = 0;
  planar_camera_info_.roi.height = 0;
  planar_camera_info_.roi.width = 0;
  planar_camera_info_.roi.do_rectify = false;

  // Precompute target-frame normalized directions once
  planar_x_norm_.resize(planar_width_);
  planar_y_norm_.resize(planar_height_);
  const double inv_fx = 1.0 / planar_fx_;
  const double inv_fy = 1.0 / planar_fy_;
  for (int u = 0; u < planar_width_; ++u) planar_x_norm_[u] = (static_cast<double>(u) - planar_cx_) * inv_fx;
  for (int v = 0; v < planar_height_; ++v) planar_y_norm_[v] = (static_cast<double>(v) - planar_cy_) * inv_fy;
}

void ImageReprojection::configureEquirectCameraInfo() {
  equirect_camera_info_.height = static_cast<uint32_t>(equirect_height_);
  equirect_camera_info_.width = static_cast<uint32_t>(equirect_width_);
  equirect_camera_info_.distortion_model = "equirectangular";
  equirect_camera_info_.d.clear();
  equirect_camera_info_.k = {equirect_hfov_rad_, 0.0, 0.0,
                             0.0, equirect_vfov_rad_, 0.0,
                             0.0, 0.0, 1.0};
  equirect_camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  equirect_camera_info_.p = {equirect_hfov_rad_, 0.0, 0.0, 0.0,
                             0.0, equirect_vfov_rad_, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0};
  equirect_camera_info_.binning_x = 1;
  equirect_camera_info_.binning_y = 1;
  equirect_camera_info_.roi.x_offset = 0;
  equirect_camera_info_.roi.y_offset = 0;
  equirect_camera_info_.roi.height = 0;
  equirect_camera_info_.roi.width = 0;
  equirect_camera_info_.roi.do_rectify = false;

  // Precompute spherical directions tables for performance
  equirect_sin_lat_.resize(equirect_height_);
  equirect_cos_lat_.resize(equirect_height_);
  equirect_sin_lon_.resize(equirect_width_);
  equirect_cos_lon_.resize(equirect_width_);
  for (int v = 0; v < equirect_height_; ++v) {
    const double v_norm = (static_cast<double>(v) + 0.5) / static_cast<double>(equirect_height_);
    const double lat = (0.5 - v_norm) * equirect_vfov_rad_;
    equirect_sin_lat_[v] = std::sin(lat);
    equirect_cos_lat_[v] = std::cos(lat);
  }
  for (int u = 0; u < equirect_width_; ++u) {
    const double u_norm = (static_cast<double>(u) + 0.5) / static_cast<double>(equirect_width_);
    const double lon = (u_norm - 0.5) * equirect_hfov_rad_;
    equirect_sin_lon_[u] = std::sin(lon);
    equirect_cos_lon_[u] = std::cos(lon);
  }
}

void ImageReprojection::imageCallback(size_t index, const Image::ConstSharedPtr &image) {
  const auto start_time = this->now();

  if (index >= input_configs_.size()) {
    RCLCPP_WARN(get_logger(), "Received data for out-of-range camera index %zu", index);
    return;
  }

  BgrImage converted_image;
  if (!toBgrImage(image, converted_image, get_logger())) {
    return;
  }

  if (!intrinsics_ready_[index]) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring image for cam %zu until CameraInfo is received", index);
    return;
  }

  const rclcpp::Time stamp = image->header.stamp;
  const int64_t stamp_ns = stamp.nanoseconds();
  const rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(frame_time_tolerance_sec_);

  auto within_tolerance = [&](const FrameAccumulator &candidate) {
    const rclcpp::Duration diff = (stamp >= candidate.stamp) ? (stamp - candidate.stamp) : (candidate.stamp - stamp);
    return diff <= tolerance;
  };

  auto selected_it = frame_accumulators_.end();
  if (!frame_accumulators_.empty()) {
    auto lower = frame_accumulators_.lower_bound(stamp_ns);
    if (lower != frame_accumulators_.end() && within_tolerance(lower->second)) {
      selected_it = lower;
    } else if (lower != frame_accumulators_.begin()) {
      auto prev = std::prev(lower);
      if (within_tolerance(prev->second)) {
        selected_it = prev;
      }
    }
  }

  if (selected_it == frame_accumulators_.end()) {
    auto insert_result = frame_accumulators_.emplace(stamp_ns, FrameAccumulator{});
    selected_it = insert_result.first;
    auto &new_frame = selected_it->second;
    new_frame.stamp = stamp;
    new_frame.images.resize(input_configs_.size());
    new_frame.intrinsics.resize(input_configs_.size());
    new_frame.frame_ids.resize(input_configs_.size());
    new_frame.ready.assign(input_configs_.size(), false);
    RCLCPP_DEBUG(get_logger(), "Created new frame bucket %ld for camera %zu", selected_it->first, index);
  }

  auto &frame = selected_it->second;
  const int64_t frame_key = selected_it->first;

  frame.images[index] = std::move(converted_image);
  frame.intrinsics[index] = static_intrinsics_[index];
  frame.frame_ids[index] = camera_frame_ids_[index];
  frame.ready[index] = true;

  const size_t ready_count = static_cast<size_t>(std::count(frame.ready.begin(), frame.ready.end(), true));
  const bool frame_ready = ready_count == frame.ready.size();
  if (!frame_ready) {
    RCLCPP_DEBUG(get_logger(),
                 "Frame %ld: %zu/%zu cameras ready (latest camera %zu, stamp %.3f s)",
                 frame_key,
                 ready_count,
                 frame.ready.size(),
                 index,
                 stamp.seconds());
    cleanupAccumulators(stamp);
    return;
  }

  RCLCPP_DEBUG(get_logger(),
               "Frame %ld: all %zu cameras ready (processing)",
               frame_key,
               frame.images.size());

  const std::string fallback_frame = frame.frame_ids.front();

  if (enable_planar_ && planar_frame_id_.empty()) {
    planar_frame_id_ = fallback_frame;
  }
  if (enable_equirectangular_ && equirect_frame_id_.empty()) {
    equirect_frame_id_ = fallback_frame;
  }

  std::vector<tf2::Transform> planar_transforms;
  bool planar_transforms_valid = false;
  if (enable_planar_) {
    planar_transforms_valid = lookupCameraTransforms(frame.stamp, frame.frame_ids, planar_frame_id_, planar_transforms);
    if (!planar_transforms_valid) {
      RCLCPP_WARN(get_logger(), "Frame %ld dropped: missing transforms for planar target '%s'",
                  frame_key,
                  planar_frame_id_.c_str());
    }
  }

  std::vector<tf2::Transform> equirect_transforms;
  bool equirect_transforms_valid = false;
  if (enable_equirectangular_) {
    if (enable_planar_ && planar_transforms_valid && planar_frame_id_ == equirect_frame_id_) {
      equirect_transforms = planar_transforms;
      equirect_transforms_valid = true;
    } else {
      equirect_transforms_valid = lookupCameraTransforms(frame.stamp, frame.frame_ids, equirect_frame_id_, equirect_transforms);
      if (!equirect_transforms_valid) {
        RCLCPP_WARN(get_logger(), "Frame %ld dropped: missing transforms for equirectangular target '%s'",
                    frame_key,
                    equirect_frame_id_.c_str());
      }
    }
  }

  if ((enable_planar_ && !planar_transforms_valid) && (enable_equirectangular_ && !equirect_transforms_valid)) {
    frame_accumulators_.erase(frame_key);
    cleanupAccumulators(stamp);
    return;
  }

  std::vector<BgrImage> images;
  images.reserve(frame.images.size());
  for (auto &stored_image : frame.images) {
    images.emplace_back(std::move(stored_image));
  }

  bool planar_success = false;
  bool equirect_success = false;

  if (enable_planar_ && planar_transforms_valid) {
    sensor_msgs::msg::Image planar_output;
    const auto planar_t0 = this->now();
    const bool planar_ok = reprojectPlanar(images, frame.intrinsics, planar_transforms, planar_output);
    const double planar_ms = static_cast<double>((this->now() - planar_t0).nanoseconds()) / 1e6;
    if (planar_ok) {
      planar_output.header.stamp = frame.stamp;
      planar_output.header.frame_id = planar_frame_id_;
      auto planar_info = planar_camera_info_;
      planar_info.header.stamp = frame.stamp;
      planar_info.header.frame_id = planar_frame_id_;

      planar_image_publisher_->publish(planar_output);
      planar_info_publisher_->publish(planar_info);
      planar_success = true;
      RCLCPP_INFO(get_logger(), "Frame %ld planar projection in %.2f ms", frame_key, planar_ms);
    } else {
      RCLCPP_WARN(get_logger(), "Frame %ld planar reprojection failed (%.2f ms)", frame_key, planar_ms);
    }
  }

  if (enable_equirectangular_ && equirect_transforms_valid) {
    sensor_msgs::msg::Image equirect_output;
    const auto eq_t0 = this->now();
    const bool eq_ok = reprojectEquirectangular(images, frame.intrinsics, equirect_transforms, equirect_output);
    const double eq_ms = static_cast<double>((this->now() - eq_t0).nanoseconds()) / 1e6;
    if (eq_ok) {
      equirect_output.header.stamp = frame.stamp;
      equirect_output.header.frame_id = equirect_frame_id_;
      auto equirect_info = equirect_camera_info_;
      equirect_info.header.stamp = frame.stamp;
      equirect_info.header.frame_id = equirect_frame_id_;

      equirect_image_publisher_->publish(equirect_output);
      equirect_info_publisher_->publish(equirect_info);
      equirect_success = true;
      RCLCPP_INFO(get_logger(), "Frame %ld equirectangular projection in %.2f ms", frame_key, eq_ms);
    } else {
      RCLCPP_WARN(get_logger(), "Frame %ld equirectangular reprojection failed (%.2f ms)", frame_key, eq_ms);
    }
  }

  const double elapsed_ms = static_cast<double>((this->now() - start_time).nanoseconds()) / 1e6;
  if (planar_success || equirect_success) {
    std::string success_list;
    if (planar_success) {
      success_list += "planar";
    }
    if (equirect_success) {
      if (!success_list.empty()) {
        success_list += ", ";
      }
      success_list += "equirect";
    }
    RCLCPP_INFO(get_logger(),
                "Frame %ld published in %.2f ms (%s)",
                frame_key,
                elapsed_ms,
                success_list.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Frame %ld dropped: no projection succeeded", frame_key);
  }

  frame_accumulators_.erase(frame_key);
  cleanupAccumulators(stamp);
}

void ImageReprojection::cleanupAccumulators(const rclcpp::Time &current_stamp) {
  if (accumulator_timeout_sec_ <= 0.0) {
    return;
  }

  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(accumulator_timeout_sec_);
  for (auto it = frame_accumulators_.begin(); it != frame_accumulators_.end();) {
    if (current_stamp - it->second.stamp > timeout) {
      it = frame_accumulators_.erase(it);
    } else {
      ++it;
    }
  }
}

bool ImageReprojection::lookupCameraTransforms(const rclcpp::Time &stamp,
                                               const std::vector<std::string> &camera_frames,
                                               const std::string &target_frame,
                                               std::vector<tf2::Transform> &transforms) {
  if (!tf_buffer_) {
    RCLCPP_ERROR(get_logger(), "TF buffer is not initialised.");
    return false;
  }

  if (target_frame.empty()) {
    RCLCPP_ERROR(get_logger(), "Target frame for reprojection is empty");
    return false;
  }

  transforms.resize(camera_frames.size());
  const tf2::Duration timeout = tf2::durationFromSec(transform_timeout_sec_);

  for (size_t i = 0; i < camera_frames.size(); ++i) {
    if (camera_frames[i].empty()) {
      RCLCPP_ERROR(get_logger(), "Camera %zu provided an empty frame_id in CameraInfo.", i);
      return false;
    }

    if (camera_frames[i] == target_frame) {
      transforms[i].setIdentity();
      continue;
    }

    try {
      const auto transform_msg = tf_buffer_->lookupTransform(camera_frames[i], target_frame, stamp, timeout);
      tf2::fromMsg(transform_msg.transform, transforms[i]);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to lookup transform from '%s' to '%s': %s",
                           target_frame.c_str(),
                           camera_frames[i].c_str(),
                           ex.what());
      return false;
    }
  }

  return true;
}

void ImageReprojection::cameraInfoCallback(size_t index, const CameraInfo::ConstSharedPtr &info) {
  if (index >= input_configs_.size()) return;
  CameraIntrinsics intr;
  if (!extractIntrinsics(info, intr, get_logger())) return;
  static_intrinsics_[index] = intr;
  camera_frame_ids_[index] = info->header.frame_id;
  intrinsics_ready_[index] = true;

  // If target frames are not set, pick this one as default
  if (enable_planar_ && planar_frame_id_.empty()) planar_frame_id_ = camera_frame_ids_[index];
  if (enable_equirectangular_ && equirect_frame_id_.empty()) equirect_frame_id_ = camera_frame_ids_[index];

  // Cache static transforms if possible (time 0)
  if (enable_planar_ && !planar_frame_id_.empty()) {
    try {
      const auto tf_msg = tf_buffer_->lookupTransform(camera_frame_ids_[index], planar_frame_id_, rclcpp::Time(0), tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, cached_planar_transforms_[index]);
      planar_tf_ready_[index] = true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(get_logger(), "Planar transform not yet available for cam %zu: %s", index, ex.what());
    }
  }
  if (enable_equirectangular_ && !equirect_frame_id_.empty()) {
    try {
      const auto tf_msg = tf_buffer_->lookupTransform(camera_frame_ids_[index], equirect_frame_id_, rclcpp::Time(0), tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, cached_equirect_transforms_[index]);
      equirect_tf_ready_[index] = true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_DEBUG(get_logger(), "Equirect transform not yet available for cam %zu: %s", index, ex.what());
    }
  }
}

bool ImageReprojection::reprojectEquirectangular(const std::vector<BgrImage> &input_images,
                                                 const std::vector<CameraIntrinsics> &intrinsics,
                                                 const std::vector<tf2::Transform> &transforms,
                                                 sensor_msgs::msg::Image &output_image) const {
  if (input_images.empty()) {
    RCLCPP_WARN(get_logger(), "No input images available for equirectangular reprojection.");
    return false;
  }

  if (intrinsics.size() != input_images.size() || transforms.size() != input_images.size()) {
    RCLCPP_ERROR(get_logger(),
                 "Inconsistent input sizes: %zu images, %zu intrinsics, %zu transforms.",
                 input_images.size(), intrinsics.size(), transforms.size());
    return false;
  }

  const int width = equirect_width_;
  const int height = equirect_height_;
  const size_t pixel_count = static_cast<size_t>(height) * width;

  std::vector<std::vector<float>> accumulators;
  accumulators.reserve(input_images.size());
  std::vector<std::vector<float>> weights;
  weights.reserve(input_images.size());

  for (size_t i = 0; i < input_images.size(); ++i) {
    accumulators.emplace_back(pixel_count * 3, 0.0f);
    weights.emplace_back(pixel_count, 0.0f);
  }

  for (size_t i = 0; i < input_images.size(); ++i) {
    const auto &intr = intrinsics[i];
    const BgrImage &image = input_images[i];
    const tf2::Transform &transform = transforms[i];

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

    auto &acc = accumulators[i];
    auto &weight_buffer = weights[i];

    for (int v = 0; v < height; ++v) {
      const double sin_lat = equirect_sin_lat_[v];
      const double cos_lat = equirect_cos_lat_[v];

      for (int u = 0; u < width; ++u) {
        const double sin_lon = equirect_sin_lon_[u];
        const double cos_lon = equirect_cos_lon_[u];

        tf2::Vector3 direction(cos_lat * sin_lon,
                               -sin_lat,
                               cos_lat * cos_lon);
        const tf2::Vector3 point_virtual = direction * equirect_radius_;
        const tf2::Vector3 point_input = transform * point_virtual;

        const double z = point_input.z();
        if (z <= kEpsilon) {
          continue;
        }

        const double inv_z = 1.0 / z;
        const double u_in = fx_in * (point_input.x() * inv_z) + cx_in;
        const double v_in = fy_in * (point_input.y() * inv_z) + cy_in;

        if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) ||
            v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, u_in, v_in);
        const size_t pixel_index = static_cast<size_t>(v) * width + u;
        const size_t base_index = pixel_index * 3;
        acc[base_index + 0] += colour[0];
        acc[base_index + 1] += colour[1];
        acc[base_index + 2] += colour[2];
        weight_buffer[pixel_index] += 1.0f;
      }
    }
  }

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.resize(output_image.step * output_image.height);

  const auto colour_from_accumulator = [](const std::vector<float> &acc, float weight, size_t base_index) {
    std::array<float, 3> colour{0.0f, 0.0f, 0.0f};
    if (weight > 0.0f) {
      const float inv_weight = 1.0f / weight;
      colour[0] = acc[base_index + 0] * inv_weight;
      colour[1] = acc[base_index + 1] * inv_weight;
      colour[2] = acc[base_index + 2] * inv_weight;
    }
    return colour;
  };

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t pixel_index = static_cast<size_t>(v) * width + u;
      const size_t base_index = pixel_index * 3;
      uint8_t *pixel = &output_image.data[base_index];

      float total_weight = 0.0f;
      float max_weight = -1.0f;
      size_t dominant_index = 0;

      for (size_t i = 0; i < weights.size(); ++i) {
        const float w = weights[i][pixel_index];
        if (w <= 0.0f) {
          continue;
        }
        total_weight += w;
        if (w > max_weight) {
          max_weight = w;
          dominant_index = i;
        }
      }

      if (total_weight <= 0.0f) {
        pixel[0] = pixel[1] = pixel[2] = 0;
        continue;
      }

      std::array<float, 3> dominant_colour{0.0f, 0.0f, 0.0f};
      std::array<float, 3> average_colour{0.0f, 0.0f, 0.0f};

      for (size_t i = 0; i < weights.size(); ++i) {
        const float w = weights[i][pixel_index];
        if (w <= 0.0f) {
          continue;
        }
        const auto colour = colour_from_accumulator(accumulators[i], w, base_index);
        const float normalized_w = w / total_weight;
        average_colour[0] += normalized_w * colour[0];
        average_colour[1] += normalized_w * colour[1];
        average_colour[2] += normalized_w * colour[2];

        if (i == dominant_index) {
          dominant_colour = colour;
        }
      }

      for (size_t channel = 0; channel < 3; ++channel) {
        const float blended_value = static_cast<float>((1.0 - planar_blend_factor_) * dominant_colour[channel] +
                                                      planar_blend_factor_ * average_colour[channel]);
        pixel[channel] = static_cast<uint8_t>(std::clamp(blended_value, 0.0f, 255.0f));
      }
    }
  }

  return true;
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

bool ImageReprojection::reprojectPlanar(const std::vector<BgrImage> &input_images,
                                        const std::vector<CameraIntrinsics> &intrinsics,
                                        const std::vector<tf2::Transform> &transforms,
                                        sensor_msgs::msg::Image &output_image) const {
  if (input_images.empty()) {
    RCLCPP_WARN(get_logger(), "No input images available for reprojection.");
    return false;
  }

  if (intrinsics.size() != input_images.size() || transforms.size() != input_images.size()) {
    RCLCPP_ERROR(get_logger(),
                 "Inconsistent input sizes: %zu images, %zu intrinsics, %zu transforms.",
                 input_images.size(), intrinsics.size(), transforms.size());
    return false;
  }

  const int width = planar_width_;
  const int height = planar_height_;
  const size_t pixel_count = static_cast<size_t>(height) * width;

  std::vector<std::vector<float>> accumulators;
  accumulators.reserve(input_images.size());
  std::vector<std::vector<float>> weights;
  weights.reserve(input_images.size());

  for (size_t i = 0; i < input_images.size(); ++i) {
    accumulators.emplace_back(pixel_count * 3, 0.0f);
    weights.emplace_back(pixel_count, 0.0f);
  }

  const auto &x_norm = planar_x_norm_;
  const auto &y_norm = planar_y_norm_;

  for (size_t i = 0; i < input_images.size(); ++i) {
    const auto &intr = intrinsics[i];
    const BgrImage &image = input_images[i];
    const tf2::Transform &transform = transforms[i];

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

    auto &acc = accumulators[i];
    auto &weight_buffer = weights[i];

    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        const tf2::Vector3 point_virtual(x_norm[u] * planar_depth_,
                                         y_norm[v] * planar_depth_,
                                         planar_depth_);
        const tf2::Vector3 point_input = transform * point_virtual;

        const double z = point_input.z();
        if (z <= kEpsilon) {
          continue;
        }

        const double inv_z = 1.0 / z;
        const double u_in = fx_in * (point_input.x() * inv_z) + cx_in;
        const double v_in = fy_in * (point_input.y() * inv_z) + cy_in;

        if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) ||
            v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, u_in, v_in);
        const size_t pixel_index = static_cast<size_t>(v) * width + u;
        const size_t base_index = pixel_index * 3;
        acc[base_index + 0] += colour[0];
        acc[base_index + 1] += colour[1];
        acc[base_index + 2] += colour[2];
        weight_buffer[pixel_index] += 1.0f;
      }
    }
  }

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.resize(output_image.step * output_image.height);

  const auto colour_from_accumulator = [](const std::vector<float> &acc, float weight, size_t base_index) {
    std::array<float, 3> colour{0.0f, 0.0f, 0.0f};
    if (weight > 0.0f) {
      const float inv_weight = 1.0f / weight;
      colour[0] = acc[base_index + 0] * inv_weight;
      colour[1] = acc[base_index + 1] * inv_weight;
      colour[2] = acc[base_index + 2] * inv_weight;
    }
    return colour;
  };

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t pixel_index = static_cast<size_t>(v) * width + u;
      const size_t base_index = pixel_index * 3;
      uint8_t *pixel = &output_image.data[base_index];

      float total_weight = 0.0f;
      float max_weight = -1.0f;
      size_t dominant_index = 0;

      for (size_t i = 0; i < weights.size(); ++i) {
        const float w = weights[i][pixel_index];
        if (w <= 0.0f) {
          continue;
        }
        total_weight += w;
        if (w > max_weight) {
          max_weight = w;
          dominant_index = i;
        }
      }

      if (total_weight <= 0.0f) {
        pixel[0] = pixel[1] = pixel[2] = 0;
        continue;
      }

      std::array<float, 3> dominant_colour{0.0f, 0.0f, 0.0f};
      std::array<float, 3> average_colour{0.0f, 0.0f, 0.0f};

      for (size_t i = 0; i < weights.size(); ++i) {
        const float w = weights[i][pixel_index];
        if (w <= 0.0f) {
          continue;
        }
        const auto colour = colour_from_accumulator(accumulators[i], w, base_index);
        const float normalized_w = w / total_weight;
        average_colour[0] += normalized_w * colour[0];
        average_colour[1] += normalized_w * colour[1];
        average_colour[2] += normalized_w * colour[2];

        if (i == dominant_index) {
          dominant_colour = colour;
        }
      }

      for (size_t channel = 0; channel < 3; ++channel) {
        const float blended_value = static_cast<float>((1.0 - equirect_blend_factor_) * dominant_colour[channel] +
                                                      equirect_blend_factor_ * average_colour[channel]);
        pixel[channel] = static_cast<uint8_t>(std::clamp(blended_value, 0.0f, 255.0f));
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

// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0
 
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <image_reprojection/image_reprojection.hpp>
#ifdef HAVE_CUDA
#include <reprojection_cuda.h>
#endif
#include <image_transport/image_transport.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

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

std::string escapeJson(const std::string& value) {
  std::ostringstream oss;
  for (const char ch : value) {
    switch (ch) {
      case '"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          char buffer[7];
          std::snprintf(buffer, sizeof(buffer), "\\u%04X", static_cast<unsigned int>(static_cast<unsigned char>(ch)));
          oss << buffer;
        } else {
          oss << ch;
        }
    }
  }
  return oss.str();
}

std::array<double, 16> transformToMatrix(const tf2::Transform& transform) {
  std::array<double, 16> matrix{};
  const tf2::Matrix3x3 basis = transform.getBasis();
  const tf2::Vector3 origin = transform.getOrigin();

  matrix[0] = basis[0][0];
  matrix[1] = basis[0][1];
  matrix[2] = basis[0][2];
  matrix[3] = origin.x();

  matrix[4] = basis[1][0];
  matrix[5] = basis[1][1];
  matrix[6] = basis[1][2];
  matrix[7] = origin.y();

  matrix[8] = basis[2][0];
  matrix[9] = basis[2][1];
  matrix[10] = basis[2][2];
  matrix[11] = origin.z();

  matrix[12] = 0.0;
  matrix[13] = 0.0;
  matrix[14] = 0.0;
  matrix[15] = 1.0;

  return matrix;
}

template <typename Container>
void writeJsonArray(std::ostringstream& oss, const Container& values) {
  oss << '[';
  const size_t count = values.size();
  for (size_t i = 0; i < count; ++i) {
    oss << values[i];
    if (i + 1 < count) {
      oss << ", ";
    }
  }
  oss << ']';
}

}  // namespace

struct ImageReprojection::CudaState {
#ifdef HAVE_CUDA
  struct Projection {
    ReprojectionCuda* dominant{nullptr};
    ReprojectionCuda* multi{nullptr};
    uint64_t version{std::numeric_limits<uint64_t>::max()};
    bool dominant_failed{false};
    bool multi_failed{false};

    void reset(uint64_t new_version) {
      reprojection_cuda_destroy(dominant);
      reprojection_cuda_destroy(multi);
      dominant = nullptr;
      multi = nullptr;
      version = new_version;
      dominant_failed = false;
      multi_failed = false;
    }
    ~Projection() { reset(version); }
  };
  Projection planar;
  Projection equirect;
#endif
};

ImageReprojection::~ImageReprojection() = default;

ImageReprojection::ImageReprojection(const rclcpp::NodeOptions& options) : rclcpp::Node("image_reprojection", options) {
  loadParameters();
  if (enable_planar_) {
    configurePlanarCameraInfo();
  }
  if (enable_equirectangular_) {
    configureEquirectCameraInfo();
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  const char* sync_mode_str = aggregation_mode_ == AggregationMode::WaitForAll ? "wait_all" : "lead_latest";

  RCLCPP_INFO(get_logger(),
              "Image reprojection node initialised with %zu inputs. "
              "Projections: %s (%s). Sync mode: %s. GPU: %s",
              input_configs_.size(), projection_list.c_str(), frame_info.empty() ? "n/a" : frame_info.c_str(),
              sync_mode_str, use_gpu_ ? "enabled" : "disabled");
#ifndef HAVE_CUDA
  if (use_gpu_) RCLCPP_WARN(get_logger(), "params.use_gpu is true, but this package was built without CUDA; using CPU");
#endif

  // run setup after constructor has finished to enable shared_from_this()
  setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
    setupTopics();
    setupParameterCallback();
    setup_timer_->cancel();
  });
  frame_timeout_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
    std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    if (aggregation_mode_ == AggregationMode::WaitForAll) expireFrameAccumulators();
  });
}

void ImageReprojection::loadParameters() {
  auto fixed_descriptor = [](const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
  };
  auto dynamic_descriptor = [](const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = false;
    return descriptor;
  };

  const auto fixed = fixed_descriptor("Configured at startup. Restart the node to change this parameter.");
  const auto dynamic = dynamic_descriptor("May be changed at runtime with ros2 param set.");

  accumulator_timeout_sec_ = this->declare_parameter<double>("params.frame_timeout", kDefaultAccumulatorTimeoutSec, dynamic);
  if (accumulator_timeout_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "frame_timeout must be non-negative. Using %.2f instead of %.2f.", kDefaultAccumulatorTimeoutSec,
                accumulator_timeout_sec_);
    accumulator_timeout_sec_ = kDefaultAccumulatorTimeoutSec;
  }

  transform_timeout_sec_ = this->declare_parameter<double>("params.transform_timeout", 0.05, dynamic);
  if (transform_timeout_sec_ < 0.0) {
    throw std::runtime_error("transform_timeout must be non-negative");
  }

  frame_time_tolerance_sec_ = this->declare_parameter<double>("params.frame_time_tolerance", 0.005, dynamic);
  if (frame_time_tolerance_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(), "frame_time_tolerance must be non-negative. Using 0.0 instead of %.6f.", frame_time_tolerance_sec_);
    frame_time_tolerance_sec_ = 0.0;
  }
  wait_all_publish_partial_ = this->declare_parameter<bool>("params.wait_all_publish_partial", true, dynamic);

  gst_config_export_path_ = this->declare_parameter<std::string>("output.gstreamer.config_export_path", "", fixed);
  if (!gst_config_export_path_.empty()) {
    gst_config_dirty_ = true;
  }

  const std::string sync_mode = this->declare_parameter<std::string>("params.sync_mode", "wait_all", dynamic);
  if (sync_mode == "wait_all" || sync_mode == "all" || sync_mode == "sync") {
    aggregation_mode_ = AggregationMode::WaitForAll;
  } else if (sync_mode == "lead_latest" || sync_mode == "lead" || sync_mode == "lead_image") {
    aggregation_mode_ = AggregationMode::LeadWithLatest;
  } else {
    throw std::runtime_error("Unsupported params.sync_mode value: '" + sync_mode + "'");
  }

  enable_planar_ = this->declare_parameter<bool>("output.projection.planar.enabled", true, fixed);
  enable_equirectangular_ = this->declare_parameter<bool>("output.projection.equirectangular.enabled", false, fixed);

  if (!enable_planar_ && !enable_equirectangular_) {
    throw std::runtime_error("At least one projection (planar or equirectangular) must be enabled");
  }

  planar_image_topic_ =
      this->declare_parameter<std::string>("output.projection.planar.image_topic", "~/output/planar/image", fixed);
  planar_info_topic_ =
      this->declare_parameter<std::string>("output.projection.planar.camera_info_topic", "~/output/planar/camera_info", fixed);
  planar_frame_id_ = this->declare_parameter<std::string>("output.projection.planar.optical_frame_id", "", fixed);
  const int planar_width = this->declare_parameter<int>("output.projection.planar.width", 1280, dynamic);
  const int planar_height = this->declare_parameter<int>("output.projection.planar.height", 720, dynamic);
  const double planar_depth = this->declare_parameter<double>("output.projection.planar.depth", 1.0, dynamic);
  const double planar_blend_factor = this->declare_parameter<double>("output.projection.planar.blend_factor", 1.0, dynamic);
  const double planar_fov_x_deg = this->declare_parameter<double>("output.projection.planar.fov_x", 90.0, dynamic);

  if (enable_planar_) {
    if (planar_width <= 0 || planar_height <= 0) {
      throw std::runtime_error("output.projection.planar width and height must be positive");
    }
    if (planar_depth <= kEpsilon) {
      throw std::runtime_error("output.projection.planar.depth must be positive");
    }
  }
  applyPlanarProjectionConfig(planar_width, planar_height, planar_depth, planar_fov_x_deg, planar_blend_factor);

  equirect_image_topic_ = this->declare_parameter<std::string>("output.projection.equirectangular.image_topic",
                                                               "~/output/equirectangular/image", fixed);
  equirect_info_topic_ = this->declare_parameter<std::string>("output.projection.equirectangular.camera_info_topic",
                                                              "~/output/equirectangular/camera_info", fixed);
  equirect_frame_id_ = this->declare_parameter<std::string>("output.projection.equirectangular.optical_frame_id", "", fixed);
  const int equirect_width = this->declare_parameter<int>("output.projection.equirectangular.width", 2048, dynamic);
  const int equirect_height = this->declare_parameter<int>("output.projection.equirectangular.height", 1024, dynamic);
  const double equirect_radius =
      this->declare_parameter<double>("output.projection.equirectangular.radius", enable_planar_ ? planar_depth_ : 1.0, dynamic);
  const double equirect_blend_factor =
      this->declare_parameter<double>("output.projection.equirectangular.blend_factor", 1.0, dynamic);
  const double equirect_fov_x_deg = this->declare_parameter<double>("output.projection.equirectangular.fov_x", 360.0, dynamic);

  if (enable_equirectangular_) {
    if (equirect_width <= 0 || equirect_height <= 0) {
      throw std::runtime_error(
          "output.projection.equirectangular width and "
          "height must be positive");
    }
    if (equirect_radius <= kEpsilon) {
      throw std::runtime_error("output.projection.equirectangular.radius must be positive");
    }
  }
  applyEquirectProjectionConfig(equirect_width, equirect_height, equirect_radius, equirect_fov_x_deg, equirect_blend_factor);

  recompute_every_frame_ = this->declare_parameter<bool>("params.recompute_every_frame", false, dynamic);
  use_gpu_ = this->declare_parameter<bool>("params.use_gpu", false, fixed);

  const auto image_topics =
      this->declare_parameter<std::vector<std::string>>("input.image_topics", std::vector<std::string>{}, fixed);
  if (image_topics.empty()) {
    throw std::runtime_error("input.image_topics must contain at least one topic");
  }

  input_configs_.clear();
  input_configs_.reserve(image_topics.size());

  for (size_t i = 0; i < image_topics.size(); ++i) {
    const auto& topic = image_topics[i];
    InputCameraConfig config;
    config.name = "input_" + std::to_string(i);
    config.image_topic = topic;

    const std::string camera_info_param = "input." + topic + ".camera_info_topic";
    if (this->has_parameter(camera_info_param)) {
      config.camera_info_topic = this->get_parameter(camera_info_param).as_string();
    } else {
      config.camera_info_topic = this->declare_parameter<std::string>(camera_info_param, "", fixed);
    }

    if (config.camera_info_topic.empty()) {
      throw std::runtime_error("Missing camera_info_topic parameter for input image topic '" + topic + "'");
    }

    input_configs_.push_back(std::move(config));
  }

  if (enable_planar_) {
    if (planar_fx_ <= kEpsilon || planar_fy_ <= kEpsilon) {
      throw std::runtime_error("virtual_camera focal lengths must be positive");
    }
  }
}

void ImageReprojection::setupParameterCallback() {
  auto fail = [](const std::string& reason) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
  };

  auto ok = []() {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  };

  auto parse_sync_mode = [](const std::string& value, AggregationMode& mode) {
    if (value == "wait_all" || value == "all" || value == "sync") {
      mode = AggregationMode::WaitForAll;
      return true;
    }
    if (value == "lead_latest" || value == "lead" || value == "lead_image") {
      mode = AggregationMode::LeadWithLatest;
      return true;
    }
    return false;
  };

  parameter_callback_handle_ =
      this->add_on_set_parameters_callback([this, fail, ok, parse_sync_mode](const std::vector<rclcpp::Parameter>& parameters) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        int planar_width = planar_width_;
        int planar_height = planar_height_;
        double planar_depth = planar_depth_;
        double planar_fov_x_deg = planar_fov_x_deg_;
        double planar_blend_factor = planar_blend_factor_;

        int equirect_width = equirect_width_;
        int equirect_height = equirect_height_;
        double equirect_radius = equirect_radius_;
        double equirect_fov_x_deg = equirect_fov_x_deg_;
        double equirect_blend_factor = equirect_blend_factor_;

        bool recompute_every_frame = recompute_every_frame_;
        double transform_timeout_sec = transform_timeout_sec_;
        double accumulator_timeout_sec = accumulator_timeout_sec_;
        double frame_time_tolerance_sec = frame_time_tolerance_sec_;
        bool wait_all_publish_partial = wait_all_publish_partial_;
        AggregationMode aggregation_mode = aggregation_mode_;

        bool planar_changed = false;
        bool equirect_changed = false;
        bool recompute_changed = false;
        bool sync_changed = false;

        for (const auto& parameter : parameters) {
          const auto& name = parameter.get_name();
          const auto type = parameter.get_type();

          if (name == "output.projection.planar.width") {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER) return fail(name + " must be an integer");
            planar_width = static_cast<int>(parameter.as_int());
            planar_changed = true;
          } else if (name == "output.projection.planar.height") {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER) return fail(name + " must be an integer");
            planar_height = static_cast<int>(parameter.as_int());
            planar_changed = true;
          } else if (name == "output.projection.planar.depth") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            planar_depth = parameter.as_double();
            planar_changed = true;
          } else if (name == "output.projection.planar.fov_x") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            planar_fov_x_deg = parameter.as_double();
            planar_changed = true;
          } else if (name == "output.projection.planar.blend_factor") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            planar_blend_factor = parameter.as_double();
            planar_changed = true;
          } else if (name == "output.projection.equirectangular.width") {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER) return fail(name + " must be an integer");
            equirect_width = static_cast<int>(parameter.as_int());
            equirect_changed = true;
          } else if (name == "output.projection.equirectangular.height") {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER) return fail(name + " must be an integer");
            equirect_height = static_cast<int>(parameter.as_int());
            equirect_changed = true;
          } else if (name == "output.projection.equirectangular.radius") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            equirect_radius = parameter.as_double();
            equirect_changed = true;
          } else if (name == "output.projection.equirectangular.fov_x") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            equirect_fov_x_deg = parameter.as_double();
            equirect_changed = true;
          } else if (name == "output.projection.equirectangular.blend_factor") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            equirect_blend_factor = parameter.as_double();
            equirect_changed = true;
          } else if (name == "params.recompute_every_frame") {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL) return fail(name + " must be a bool");
            recompute_every_frame = parameter.as_bool();
            recompute_changed = recompute_every_frame != recompute_every_frame_;
          } else if (name == "params.transform_timeout") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            transform_timeout_sec = parameter.as_double();
          } else if (name == "params.frame_timeout") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            accumulator_timeout_sec = parameter.as_double();
          } else if (name == "params.frame_time_tolerance") {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE) return fail(name + " must be a double");
            frame_time_tolerance_sec = parameter.as_double();
          } else if (name == "params.wait_all_publish_partial") {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL) return fail(name + " must be a bool");
            wait_all_publish_partial = parameter.as_bool();
          } else if (name == "params.sync_mode") {
            if (type != rclcpp::ParameterType::PARAMETER_STRING) return fail(name + " must be a string");
            if (!parse_sync_mode(parameter.as_string(), aggregation_mode)) {
              return fail("Unsupported params.sync_mode value: '" + parameter.as_string() + "'");
            }
            sync_changed = aggregation_mode != aggregation_mode_;
          } else {
            return fail("Parameter '" + name + "' is read-only and requires a node restart");
          }
        }

        if (enable_planar_ || planar_changed) {
          if (planar_width <= 0 || planar_height <= 0) {
            return fail("output.projection.planar width and height must be positive");
          }
          if (!std::isfinite(planar_depth) || planar_depth <= kEpsilon) {
            return fail("output.projection.planar.depth must be positive and finite");
          }
          if (!std::isfinite(planar_fov_x_deg)) {
            return fail("output.projection.planar.fov_x must be finite");
          }
          if (!std::isfinite(planar_blend_factor)) {
            return fail("output.projection.planar.blend_factor must be finite");
          }
        }

        if (enable_equirectangular_ || equirect_changed) {
          if (equirect_width <= 0 || equirect_height <= 0) {
            return fail(
                "output.projection.equirectangular width and height "
                "must be positive");
          }
          if (!std::isfinite(equirect_radius) || equirect_radius <= kEpsilon) {
            return fail(
                "output.projection.equirectangular.radius must be "
                "positive and finite");
          }
          if (!std::isfinite(equirect_fov_x_deg)) {
            return fail("output.projection.equirectangular.fov_x must be finite");
          }
          if (!std::isfinite(equirect_blend_factor)) {
            return fail(
                "output.projection.equirectangular.blend_factor must "
                "be finite");
          }
        }
        if (!std::isfinite(transform_timeout_sec) || transform_timeout_sec < 0.0) {
          return fail("params.transform_timeout must be non-negative and finite");
        }
        if (!std::isfinite(accumulator_timeout_sec) || accumulator_timeout_sec < 0.0) {
          return fail("params.frame_timeout must be non-negative and finite");
        }
        if (!std::isfinite(frame_time_tolerance_sec) || frame_time_tolerance_sec < 0.0) {
          return fail("params.frame_time_tolerance must be non-negative and finite");
        }

        if (planar_changed) {
          applyPlanarProjectionConfig(planar_width, planar_height, planar_depth, planar_fov_x_deg, planar_blend_factor);
          configurePlanarCameraInfo();
          rebuildPlanarWarpCaches();
          markGstConfigDirty();
        }
        if (equirect_changed) {
          applyEquirectProjectionConfig(equirect_width, equirect_height, equirect_radius, equirect_fov_x_deg,
                                        equirect_blend_factor);
          configureEquirectCameraInfo();
          rebuildEquirectWarpCaches();
          markGstConfigDirty();
        }

        const bool sync_export_changed = accumulator_timeout_sec != accumulator_timeout_sec_ ||
                                         frame_time_tolerance_sec != frame_time_tolerance_sec_ ||
                                         wait_all_publish_partial != wait_all_publish_partial_ ||
                                         aggregation_mode != aggregation_mode_;
        recompute_every_frame_ = recompute_every_frame;
        transform_timeout_sec_ = transform_timeout_sec;
        accumulator_timeout_sec_ = accumulator_timeout_sec;
        frame_time_tolerance_sec_ = frame_time_tolerance_sec;
        wait_all_publish_partial_ = wait_all_publish_partial;
        aggregation_mode_ = aggregation_mode;
        if (sync_export_changed) markGstConfigDirty();

        if (recompute_changed) {
          std::fill(planar_warp_ready_.begin(), planar_warp_ready_.end(), false);
          std::fill(equirect_warp_ready_.begin(), equirect_warp_ready_.end(), false);
          planar_dominant_map_ready_ = false;
          equirect_dominant_map_ready_ = false;
          if (!recompute_every_frame_) {
            rebuildPlanarWarpCaches();
            rebuildEquirectWarpCaches();
          }
        }
        if (sync_changed) {
          frame_accumulators_.clear();
          for (auto& pending : pending_images_) pending.clear();
          std::fill(latest_image_ready_.begin(), latest_image_ready_.end(), false);
        }

        if (planar_changed || equirect_changed || sync_export_changed) {
          exportGstConfigIfReady();
        }

        return ok();
      });
}

void ImageReprojection::applyPlanarProjectionConfig(int width, int height, double depth, double fov_x_deg, double blend_factor) {
  planar_width_ = width;
  planar_height_ = height;
  planar_depth_ = depth;
  planar_fov_x_deg_ = fov_x_deg;

  const double planar_fov_x_rad = std::clamp(fov_x_deg, 1.0, 179.0) * kPi / 180.0;
  const double half_width = static_cast<double>(planar_width_) * 0.5;
  planar_fx_ = half_width / std::tan(planar_fov_x_rad * 0.5);
  planar_fy_ = planar_fx_;
  planar_cx_ = half_width;
  planar_cy_ = static_cast<double>(planar_height_) * 0.5;
  planar_blend_factor_ = std::clamp(blend_factor, 0.0, 1.0);
}

void ImageReprojection::applyEquirectProjectionConfig(
    int width, int height, double radius, double fov_x_deg, double blend_factor) {
  equirect_width_ = width;
  equirect_height_ = height;
  equirect_radius_ = radius;
  equirect_fov_x_deg_ = fov_x_deg;

  const double hfov_clamped_deg = std::clamp(fov_x_deg, 1.0, 360.0);
  equirect_hfov_rad_ = hfov_clamped_deg * kPi / 180.0;
  const double aspect = static_cast<double>(equirect_height_) / static_cast<double>(equirect_width_);
  equirect_vfov_rad_ = equirect_hfov_rad_ * aspect;
  if (equirect_vfov_rad_ > kPi) equirect_vfov_rad_ = kPi;
  if (equirect_vfov_rad_ < kEpsilon) equirect_vfov_rad_ = kEpsilon;
  equirect_blend_factor_ = std::clamp(blend_factor, 0.0, 1.0);
}

void ImageReprojection::rebuildPlanarWarpCaches() {
  if (!enable_planar_) {
    return;
  }
  ++planar_warp_version_;
  planar_dominant_map_ready_ = false;
  planar_warp_ready_.assign(input_configs_.size(), false);
  if (planar_warp_maps_.size() != input_configs_.size()) {
    planar_warp_maps_.resize(input_configs_.size());
  }
  for (size_t i = 0; i < input_configs_.size(); ++i) {
    updatePlanarWarpCache(i);
  }
}

void ImageReprojection::rebuildPlanarDominantWarpCache() {
  planar_dominant_map_ready_ = !recompute_every_frame_ && planar_blend_factor_ == 0.0 &&
                               buildDominantWarpCache(planar_warp_maps_, planar_warp_ready_, planar_width_, planar_height_,
                                                      planar_dominant_map_);
}

void ImageReprojection::rebuildEquirectWarpCaches() {
  if (!enable_equirectangular_) {
    return;
  }
  ++equirect_warp_version_;
  equirect_dominant_map_ready_ = false;
  equirect_warp_ready_.assign(input_configs_.size(), false);
  if (equirect_warp_maps_.size() != input_configs_.size()) {
    equirect_warp_maps_.resize(input_configs_.size());
  }
  for (size_t i = 0; i < input_configs_.size(); ++i) {
    updateEquirectWarpCache(i);
  }
}

void ImageReprojection::rebuildEquirectDominantWarpCache() {
  equirect_dominant_map_ready_ = !recompute_every_frame_ && equirect_blend_factor_ == 0.0 &&
                                 buildDominantWarpCache(equirect_warp_maps_, equirect_warp_ready_, equirect_width_,
                                                        equirect_height_, equirect_dominant_map_);
}

bool ImageReprojection::buildDominantWarpCache(const std::vector<std::vector<PixelMapping>>& warp_maps,
                                                const std::vector<bool>& warp_ready,
                                                int width,
                                                int height,
                                                std::vector<DominantPixelMapping>& destination) const {
  if (width <= 0 || height <= 0 || warp_maps.size() != input_configs_.size() || warp_ready.size() != input_configs_.size()) {
    return false;
  }

  const size_t pixel_count = static_cast<size_t>(width) * height;
  for (size_t i = 0; i < input_configs_.size(); ++i) {
    if (!warp_ready[i] || warp_maps[i].size() != pixel_count) return false;
    const auto& intr = static_intrinsics_[i];
    if (static_cast<uint64_t>(intr.width) * intr.height * 3 > std::numeric_limits<uint32_t>::max()) return false;
  }

  destination.assign(pixel_count, DominantPixelMapping{});
  for (size_t camera = 0; camera < input_configs_.size(); ++camera) {
    const auto& mapping = warp_maps[camera];
    const float max_u = static_cast<float>(static_intrinsics_[camera].width - 1);
    const float max_v = static_cast<float>(static_intrinsics_[camera].height - 1);
    for (size_t pixel = 0; pixel < pixel_count; ++pixel) {
      auto& selected = destination[pixel];
      if (selected.camera != std::numeric_limits<uint32_t>::max()) continue;
      const auto& entry = mapping[pixel];
      if (!std::isfinite(entry.u) || !std::isfinite(entry.v) || entry.u < 0.0f || entry.u > max_u || entry.v < 0.0f ||
          entry.v > max_v) {
        continue;
      }
      const int x0 = static_cast<int>(std::floor(entry.u));
      const int y0 = static_cast<int>(std::floor(entry.v));
      selected.source_offset = static_cast<uint32_t>((static_cast<uint64_t>(y0) * static_intrinsics_[camera].width + x0) * 3);
      selected.dx = entry.u - static_cast<float>(x0);
      selected.dy = entry.v - static_cast<float>(y0);
      selected.right_step = x0 + 1 < static_intrinsics_[camera].width ? 3 : 0;
      selected.down_step = y0 + 1 < static_intrinsics_[camera].height ? 1 : 0;
      selected.camera = static_cast<uint32_t>(camera);
    }
  }
  return true;
}

void ImageReprojection::setupTopics() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (input_configs_.empty()) {
    throw std::runtime_error("No input cameras configured");
  }

  image_transport::ImageTransport it(this->shared_from_this());

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto info_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  auto input_info_qos = rclcpp::SensorDataQoS();

  if (enable_planar_) {
    planar_image_publisher_ = it.advertise(planar_image_topic_, 1);
    planar_info_publisher_ = this->create_publisher<CameraInfo>(planar_info_topic_, info_qos);
  }

  if (enable_equirectangular_) {
    equirect_image_publisher_ = it.advertise(equirect_image_topic_, 1);
    equirect_info_publisher_ = this->create_publisher<CameraInfo>(equirect_info_topic_, info_qos);
  }

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
  planar_warp_maps_.assign(n, {});
  planar_dominant_map_.clear();
  planar_dominant_map_ready_ = false;
  equirect_warp_maps_.assign(n, {});
  equirect_dominant_map_.clear();
  equirect_dominant_map_ready_ = false;
  planar_warp_ready_.assign(n, false);
  equirect_warp_ready_.assign(n, false);
  planar_accumulators_.assign(n, {});
  planar_weights_.assign(n, {});
  equirect_accumulators_.assign(n, {});
  equirect_weights_.assign(n, {});
  latest_images_.assign(n, {});
  latest_image_ready_.assign(n, false);
  latest_image_stamps_.assign(n, rclcpp::Time());
  latest_image_arrivals_.assign(n, rclcpp::Time());
  pending_images_.resize(n);

  for (size_t i = 0; i < n; ++i) {
    const auto idx = i;

    std::string image_transport_param_name = "input." + input_configs_[i].image_topic + ".image_transport";
    rcl_interfaces::msg::ParameterDescriptor image_transport_descriptor;
    image_transport_descriptor.description = "Configured at startup. Restart the node to change this parameter.";
    image_transport_descriptor.read_only = true;
    this->declare_parameter<std::string>(image_transport_param_name, "raw",
                                         image_transport_descriptor);  // TransportHints does not automatically
                                                                       // declare the parameter
    image_transport::TransportHints hints{this, "raw", image_transport_param_name};
    image_subs_[i] = it.subscribe(
        input_configs_[i].image_topic, sensor_qos.get_rmw_qos_profile(),
        [this, idx](const Image::ConstSharedPtr& msg) { this->imageCallback(idx, msg); }, std::shared_ptr<void>(), &hints,
        rclcpp::SubscriptionOptions());

    info_subs_[i] = this->create_subscription<CameraInfo>(
        input_configs_[i].camera_info_topic, input_info_qos,
        [this, idx](const CameraInfo::ConstSharedPtr& msg) { this->cameraInfoCallback(idx, msg); });

    RCLCPP_INFO(get_logger(), "Subscribed to image '%s' and camera_info '%s'", input_configs_[i].image_topic.c_str(),
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
  planar_camera_info_.p = {planar_fx_, 0.0, planar_cx_, 0.0, 0.0, planar_fy_, planar_cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
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
  equirect_camera_info_.k = {equirect_hfov_rad_, 0.0, 0.0, 0.0, equirect_vfov_rad_, 0.0, 0.0, 0.0, 1.0};
  equirect_camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  equirect_camera_info_.p = {equirect_hfov_rad_, 0.0, 0.0, 0.0, 0.0, equirect_vfov_rad_, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
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

void ImageReprojection::imageCallback(size_t index, const Image::ConstSharedPtr& image) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto arrival_time = this->now();

  if (index >= input_configs_.size()) {
    RCLCPP_WARN(get_logger(), "Received data for out-of-range camera index %zu", index);
    return;
  }

  if (!intrinsics_ready_[index]) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring image for cam %zu until CameraInfo is received", index);
    return;
  }

  BgrImage converted_image;
  if (!toBgrImage(image, converted_image, get_logger())) {
    return;
  }

  const rclcpp::Time stamp = image->header.stamp;

  if (aggregation_mode_ == AggregationMode::LeadWithLatest) {
    processLeadCameraImage(index, stamp, arrival_time, std::move(converted_image));
    return;
  }

  const int64_t stamp_ns = stamp.nanoseconds();
  const int64_t tolerance_ns = rclcpp::Duration::from_seconds(frame_time_tolerance_sec_).nanoseconds();
  auto distance = [](int64_t a, int64_t b) { return a >= b ? a - b : b - a; };
  auto attach = [&](FrameAccumulator& frame, size_t camera, PendingImage&& pending) {
    frame.images[camera] = std::move(pending.image);
    frame.intrinsics[camera] = static_intrinsics_[camera];
    frame.frame_ids[camera] = camera_frame_ids_[camera];
    frame.arrival_times[camera] = pending.arrival;
    frame.header_stamps[camera] = pending.stamp;
    frame.ready[camera] = true;
  };

  auto selected_it = frame_accumulators_.end();
  if (index == 0) {
    selected_it = frame_accumulators_.find(stamp_ns);
    if (selected_it == frame_accumulators_.end()) {
      selected_it = frame_accumulators_.emplace(stamp_ns, FrameAccumulator()).first;
      auto& frame = selected_it->second;
      frame.stamp = stamp;
      frame.created_at = std::chrono::steady_clock::now();
      frame.images.resize(input_configs_.size());
      frame.intrinsics.resize(input_configs_.size());
      frame.frame_ids.resize(input_configs_.size());
      frame.arrival_times.resize(input_configs_.size());
      frame.header_stamps.resize(input_configs_.size());
      frame.ready.assign(input_configs_.size(), false);
    }
    attach(selected_it->second, 0, PendingImage{std::move(converted_image), arrival_time, stamp});
    for (size_t camera = 1; camera < pending_images_.size(); ++camera) {
      auto& pending = pending_images_[camera];
      auto best = pending.end();
      int64_t best_distance = tolerance_ns + 1;
      for (auto it = pending.begin(); it != pending.end(); ++it) {
        const int64_t gap = distance(it->first, stamp_ns);
        if (gap <= tolerance_ns && gap < best_distance) {
          best = it;
          best_distance = gap;
        }
      }
      if (best != pending.end()) {
        attach(selected_it->second, camera, std::move(best->second));
        pending.erase(best);
      }
    }
  } else {
    int64_t best_distance = tolerance_ns + 1;
    for (auto it = frame_accumulators_.begin(); it != frame_accumulators_.end(); ++it) {
      if (it->second.ready[index]) {
        continue;
      }
      const int64_t gap = distance(it->first, stamp_ns);
      if (gap <= tolerance_ns && gap < best_distance) {
        selected_it = it;
        best_distance = gap;
      }
    }
    if (selected_it == frame_accumulators_.end()) {
      pending_images_[index][stamp_ns] = PendingImage{std::move(converted_image), arrival_time, stamp};
      cleanupAccumulators(stamp);
      return;
    }
    attach(selected_it->second, index, PendingImage{std::move(converted_image), arrival_time, stamp});
  }

  auto& frame = selected_it->second;
  const int64_t frame_key = selected_it->first;

  const size_t ready_count = static_cast<size_t>(std::count(frame.ready.begin(), frame.ready.end(), true));
  const bool frame_ready = ready_count == frame.ready.size();
  if (!frame_ready) {
    RCLCPP_DEBUG(get_logger(), "Frame %ld: %zu/%zu cameras ready (latest camera %zu, stamp %.3f s)", frame_key, ready_count,
                 frame.ready.size(), index, stamp.seconds());
    cleanupAccumulators(stamp);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Frame %ld: all %zu cameras ready (processing)", frame_key, frame.images.size());

  processFrame(frame_key, frame, frame.ready);
  frame_accumulators_.erase(frame_key);
  cleanupAccumulators(stamp);
}

void ImageReprojection::processLeadCameraImage(size_t index,
                                               const rclcpp::Time& stamp,
                                               const rclcpp::Time& arrival_time,
                                               BgrImage&& image) {
  const size_t camera_count = input_configs_.size();
  if (camera_count == 0) {
    return;
  }

  latest_images_[index] = std::move(image);
  latest_image_ready_[index] = true;
  latest_image_stamps_[index] = stamp;
  latest_image_arrivals_[index] = arrival_time;

  if (index != 0) {
    cleanupAccumulators(stamp);
    return;
  }

  FrameAccumulator frame;
  frame.stamp = stamp;
  frame.images.resize(camera_count);
  frame.intrinsics.resize(camera_count);
  frame.frame_ids.resize(camera_count);
  frame.arrival_times.resize(camera_count);
  frame.header_stamps.resize(camera_count);
  frame.ready.assign(camera_count, false);

  std::vector<bool> camera_mask(camera_count, false);
  const rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(frame_time_tolerance_sec_);

  size_t active_count = 0;
  for (size_t i = 0; i < camera_count; ++i) {
    frame.frame_ids[i] = camera_frame_ids_[i];
    frame.intrinsics[i] = intrinsics_ready_[i] ? static_intrinsics_[i] : CameraIntrinsics{};

    if (!intrinsics_ready_[i]) {
      continue;
    }
    if (!latest_image_ready_[i]) {
      continue;
    }
    if (camera_frame_ids_[i].empty()) {
      continue;
    }

    if (i != index) {
      const auto& other_stamp = latest_image_stamps_[i];
      const rclcpp::Duration diff = (other_stamp >= stamp) ? (other_stamp - stamp) : (stamp - other_stamp);
      if (diff > tolerance) {
        continue;
      }
    }

    camera_mask[i] = true;
    frame.ready[i] = true;
    frame.images[i] = std::move(latest_images_[i]);
    frame.arrival_times[i] = latest_image_arrivals_[i];
    frame.header_stamps[i] = latest_image_stamps_[i];
    ++active_count;
  }

  if (!camera_mask[index]) {
    camera_mask[index] = true;
    frame.ready[index] = true;
    frame.images[index] = std::move(latest_images_[index]);
    frame.arrival_times[index] = arrival_time;
    frame.header_stamps[index] = stamp;
    ++active_count;
  }

  if (active_count == 0) {
    RCLCPP_DEBUG(get_logger(), "Lead camera frame %ld dropped: no usable inputs within tolerance", stamp.nanoseconds());
    cleanupAccumulators(stamp);
    return;
  }

  const int64_t frame_key = stamp.nanoseconds();
  processFrame(frame_key, frame, camera_mask);

  for (size_t i = 0; i < camera_count; ++i) {
    if (camera_mask[i]) {
      latest_images_[i] = std::move(frame.images[i]);
    }
  }

  cleanupAccumulators(stamp);
}

void ImageReprojection::markGstConfigDirty() {
  if (!gst_config_export_path_.empty()) {
    gst_config_dirty_ = true;
  }
}

void ImageReprojection::exportGstConfigIfReady() {
  if (gst_config_export_path_.empty() || !gst_config_dirty_) {
    return;
  }

  const size_t camera_count = input_configs_.size();
  if (camera_count == 0) {
    return;
  }

  const auto intrinsics_ready = std::all_of(intrinsics_ready_.begin(), intrinsics_ready_.end(), [](bool ready) { return ready; });
  if (!intrinsics_ready) {
    return;
  }

  if (enable_planar_) {
    if (planar_frame_id_.empty()) {
      return;
    }
    if (planar_tf_ready_.size() < camera_count) {
      return;
    }
    const bool planar_ready =
        std::all_of(planar_tf_ready_.begin(), planar_tf_ready_.begin() + camera_count, [](bool ready) { return ready; });
    if (!planar_ready) {
      return;
    }
  }

  if (enable_equirectangular_) {
    if (equirect_frame_id_.empty()) {
      return;
    }
    if (equirect_tf_ready_.size() < camera_count) {
      return;
    }
    const bool equirect_ready =
        std::all_of(equirect_tf_ready_.begin(), equirect_tf_ready_.begin() + camera_count, [](bool ready) { return ready; });
    if (!equirect_ready) {
      return;
    }
  }

  std::ostringstream json;
  json << std::setprecision(10);
  json << "{\n";
  json << "  \"generated_by\": \"image_reprojection\",\n";
  json << "  \"camera_count\": " << camera_count << ",\n";
  json << "  \"sync\": {\"mode\": \""
       << (aggregation_mode_ == AggregationMode::WaitForAll ? "wait_all" : "lead_latest")
       << "\", \"frame_timeout\": " << accumulator_timeout_sec_
       << ", \"frame_time_tolerance\": " << frame_time_tolerance_sec_
       << ", \"wait_all_publish_partial\": " << (wait_all_publish_partial_ ? "true" : "false") << "},\n";
  json << "  \"cameras\": [\n";
  for (size_t i = 0; i < camera_count; ++i) {
    const auto& config = input_configs_[i];
    const auto& intr = static_intrinsics_[i];
    json << "    {\n";
    json << "      \"index\": " << i << ",\n";
    json << "      \"name\": \"" << escapeJson(config.name) << "\",\n";
    json << "      \"image_topic\": \"" << escapeJson(config.image_topic) << "\",\n";
    json << "      \"camera_info_topic\": \"" << escapeJson(config.camera_info_topic) << "\",\n";
    json << "      \"frame_id\": \"" << escapeJson(camera_frame_ids_[i]) << "\",\n";
    json << "      \"intrinsics\": {\n";
    json << "        \"fx\": " << intr.fx << ",\n";
    json << "        \"fy\": " << intr.fy << ",\n";
    json << "        \"cx\": " << intr.cx << ",\n";
    json << "        \"cy\": " << intr.cy << ",\n";
    json << "        \"width\": " << intr.width << ",\n";
    json << "        \"height\": " << intr.height << "\n";
    json << "      },\n";
    json << "      \"planar_transform\": ";
    if (enable_planar_) {
      const auto matrix = transformToMatrix(cached_planar_transforms_[i]);
      writeJsonArray(json, matrix);
    } else {
      json << "null";
    }
    json << ",\n";
    json << "      \"equirectangular_transform\": ";
    if (enable_equirectangular_) {
      const auto matrix = transformToMatrix(cached_equirect_transforms_[i]);
      writeJsonArray(json, matrix);
    } else {
      json << "null";
    }
    json << "\n";
    json << "    }";
    if (i + 1 < camera_count) {
      json << ',';
    }
    json << "\n";
  }
  json << "  ],\n";

  json << "  \"planar\": {\n";
  json << "    \"enabled\": " << (enable_planar_ ? "true" : "false");
  if (enable_planar_) {
    json << ",\n";
    json << "    \"frame_id\": \"" << escapeJson(planar_frame_id_) << "\",\n";
    json << "    \"width\": " << planar_width_ << ",\n";
    json << "    \"height\": " << planar_height_ << ",\n";
    json << "    \"fx\": " << planar_fx_ << ",\n";
    json << "    \"fy\": " << planar_fy_ << ",\n";
    json << "    \"cx\": " << planar_cx_ << ",\n";
    json << "    \"cy\": " << planar_cy_ << ",\n";
    json << "    \"depth\": " << planar_depth_ << ",\n";
    json << "    \"blend_factor\": " << planar_blend_factor_ << "\n";
  } else {
    json << "\n";
  }
  json << "  },\n";

  json << "  \"equirectangular\": {\n";
  json << "    \"enabled\": " << (enable_equirectangular_ ? "true" : "false");
  if (enable_equirectangular_) {
    json << ",\n";
    json << "    \"frame_id\": \"" << escapeJson(equirect_frame_id_) << "\",\n";
    json << "    \"width\": " << equirect_width_ << ",\n";
    json << "    \"height\": " << equirect_height_ << ",\n";
    json << "    \"hfov_rad\": " << equirect_hfov_rad_ << ",\n";
    json << "    \"vfov_rad\": " << equirect_vfov_rad_ << ",\n";
    json << "    \"radius\": " << equirect_radius_ << ",\n";
    json << "    \"blend_factor\": " << equirect_blend_factor_ << "\n";
  } else {
    json << "\n";
  }
  json << "  }\n";
  json << "}\n";

  std::error_code ec;
  std::filesystem::path export_path(gst_config_export_path_);
  if (export_path.has_parent_path() && !export_path.parent_path().empty()) {
    std::filesystem::create_directories(export_path.parent_path(), ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Failed to create directories for GStreamer config export '%s': %s",
                  export_path.parent_path().string().c_str(), ec.message().c_str());
      return;
    }
  }

  std::ofstream output(export_path, std::ios::binary | std::ios::trunc);
  if (!output) {
    RCLCPP_WARN(get_logger(), "Failed to open GStreamer config export path '%s' for writing", export_path.string().c_str());
    return;
  }

  output << json.str();
  if (!output.good()) {
    RCLCPP_WARN(get_logger(), "Failed to write complete GStreamer config to '%s'", export_path.string().c_str());
    return;
  }

  output.close();
  if (!output) {
    RCLCPP_WARN(get_logger(), "Error finalising GStreamer config export '%s'", export_path.string().c_str());
    return;
  }

  gst_config_dirty_ = false;
  RCLCPP_INFO(get_logger(), "Exported GStreamer configuration to %s", export_path.string().c_str());
}

void ImageReprojection::processFrame(int64_t frame_key, FrameAccumulator& frame, const std::vector<bool>& camera_mask) {
  const size_t camera_count = frame.images.size();
  auto is_active = [&](size_t idx) {
    if (camera_mask.empty()) {
      return true;
    }
    if (idx >= camera_mask.size()) {
      return false;
    }
    return camera_mask[idx];
  };

  size_t active_count = 0;
  for (size_t i = 0; i < camera_count; ++i) {
    if (is_active(i)) {
      ++active_count;
    }
  }

  if (active_count == 0) {
    RCLCPP_WARN(get_logger(), "Frame %ld dropped: no active cameras", frame_key);
    return;
  }

  std::string fallback_frame;
  for (size_t i = 0; i < camera_count; ++i) {
    if (!is_active(i)) {
      continue;
    }
    if (!frame.frame_ids[i].empty()) {
      fallback_frame = frame.frame_ids[i];
      break;
    }
  }
  if (fallback_frame.empty() && camera_count > 0) {
    fallback_frame = frame.frame_ids.front();
  }

  if (enable_planar_ && planar_frame_id_.empty()) {
    planar_frame_id_ = fallback_frame;
  }
  if (enable_equirectangular_ && equirect_frame_id_.empty()) {
    equirect_frame_id_ = fallback_frame;
  }

  const auto processing_start = this->now();
  const std::vector<bool>* mask_ptr = camera_mask.empty() ? nullptr : &camera_mask;

  std::vector<tf2::Transform> planar_transforms;
  bool planar_transforms_valid = false;
  double planar_lookup_ms = 0.0;
  bool planar_lookup_attempted = false;
  if (enable_planar_) {
    const auto lookup_t0 = this->now();
    planar_lookup_attempted = true;
    planar_transforms_valid =
        lookupCameraTransforms(frame.stamp, frame.frame_ids, planar_frame_id_, planar_transforms, true, mask_ptr);
    planar_lookup_ms = static_cast<double>((this->now() - lookup_t0).nanoseconds()) / 1e6;
    if (!planar_transforms_valid) {
      RCLCPP_WARN(get_logger(), "Frame %ld dropped: missing transforms for planar target '%s'", frame_key,
                  planar_frame_id_.c_str());
    }
  }

  std::vector<tf2::Transform> equirect_transforms;
  bool equirect_transforms_valid = false;
  double equirect_lookup_ms = 0.0;
  bool equirect_lookup_attempted = false;
  bool equirect_lookup_reused = false;
  if (enable_equirectangular_) {
    if (enable_planar_ && planar_transforms_valid && planar_frame_id_ == equirect_frame_id_) {
      equirect_transforms = planar_transforms;
      equirect_transforms_valid = true;
      equirect_lookup_reused = true;
    } else {
      const auto lookup_t0 = this->now();
      equirect_lookup_attempted = true;
      equirect_transforms_valid =
          lookupCameraTransforms(frame.stamp, frame.frame_ids, equirect_frame_id_, equirect_transforms, false, mask_ptr);
      equirect_lookup_ms = static_cast<double>((this->now() - lookup_t0).nanoseconds()) / 1e6;
      if (!equirect_transforms_valid) {
        RCLCPP_WARN(get_logger(),
                    "Frame %ld dropped: missing transforms for equirectangular "
                    "target '%s'",
                    frame_key, equirect_frame_id_.c_str());
      }
    }
  }

  if ((enable_planar_ && !planar_transforms_valid) && (enable_equirectangular_ && !equirect_transforms_valid)) {
    return;
  }

  std::vector<BgrImage> images;
  images.reserve(frame.images.size());
  for (auto& stored_image : frame.images) {
    images.emplace_back(std::move(stored_image));
  }

  auto restore_images = [&]() {
    for (size_t i = 0; i < images.size(); ++i) {
      frame.images[i] = std::move(images[i]);
    }
  };

  bool planar_success = false;
  bool equirect_success = false;
  bool planar_attempted = false;
  bool equirect_attempted = false;
  double planar_ms = 0.0;
  double equirect_ms = 0.0;

  if (enable_planar_ && planar_transforms_valid) {
    sensor_msgs::msg::Image planar_output;
    if (!planar_dominant_map_ready_ && !recompute_every_frame_ && planar_blend_factor_ == 0.0) {
      rebuildPlanarDominantWarpCache();
    }
    const auto planar_t0 = this->now();
    planar_attempted = true;
    const bool planar_ok = reprojectPlanar(images, frame.intrinsics, planar_transforms, mask_ptr, planar_output);
    planar_ms = static_cast<double>((this->now() - planar_t0).nanoseconds()) / 1e6;
    if (planar_ok) {
      planar_output.header.stamp = frame.stamp;
      planar_output.header.frame_id = planar_frame_id_;
      auto planar_info = planar_camera_info_;
      planar_info.header.stamp = frame.stamp;
      planar_info.header.frame_id = planar_frame_id_;

      planar_image_publisher_.publish(planar_output);
      planar_info_publisher_->publish(planar_info);
      planar_success = true;
    } else {
      RCLCPP_WARN(get_logger(), "Frame %ld planar reprojection failed (%.2f ms)", frame_key, planar_ms);
    }
  }

  if (enable_equirectangular_ && equirect_transforms_valid) {
    sensor_msgs::msg::Image equirect_output;
    if (!equirect_dominant_map_ready_ && !recompute_every_frame_ && equirect_blend_factor_ == 0.0) {
      rebuildEquirectDominantWarpCache();
    }
    const auto eq_t0 = this->now();
    equirect_attempted = true;
    const bool eq_ok = reprojectEquirectangular(images, frame.intrinsics, equirect_transforms, mask_ptr, equirect_output);
    equirect_ms = static_cast<double>((this->now() - eq_t0).nanoseconds()) / 1e6;
    if (eq_ok) {
      equirect_output.header.stamp = frame.stamp;
      equirect_output.header.frame_id = equirect_frame_id_;
      auto equirect_info = equirect_camera_info_;
      equirect_info.header.stamp = frame.stamp;
      equirect_info.header.frame_id = equirect_frame_id_;

      equirect_image_publisher_.publish(equirect_output);
      equirect_info_publisher_->publish(equirect_info);
      equirect_success = true;
    } else {
      RCLCPP_WARN(get_logger(), "Frame %ld equirectangular reprojection failed (%.2f ms)", frame_key, equirect_ms);
    }
  }

  if (!planar_success && !equirect_success) {
    restore_images();
    RCLCPP_WARN(get_logger(), "Frame %ld dropped: no projection succeeded", frame_key);
    return;
  }

  rclcpp::Time oldest_stamp;
  rclcpp::Time oldest_arrival;
  bool stamp_available = false;
  bool arrival_available = false;
  for (size_t i = 0; i < frame.header_stamps.size(); ++i) {
    if (!is_active(i)) {
      continue;
    }
    const auto& candidate_stamp = frame.header_stamps[i];
    if (candidate_stamp.nanoseconds() == 0) {
      continue;
    }
    if (!stamp_available || candidate_stamp < oldest_stamp) {
      oldest_stamp = candidate_stamp;
      stamp_available = true;
    }
    const auto& candidate_arrival = frame.arrival_times[i];
    if (candidate_arrival.nanoseconds() == 0) {
      continue;
    }
    if (!arrival_available || candidate_arrival < oldest_arrival) {
      oldest_arrival = candidate_arrival;
      arrival_available = true;
    }
  }

  const auto publish_time = this->now();
  double stamp_delay_ms = 0.0;
  double arrival_delay_ms = 0.0;
  if (stamp_available) {
    stamp_delay_ms = static_cast<double>((publish_time - oldest_stamp).nanoseconds()) / 1e6;
  }
  if (arrival_available) {
    arrival_delay_ms = static_cast<double>((publish_time - oldest_arrival).nanoseconds()) / 1e6;
  }

  auto format_ms = [](double value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2fms", value);
    return std::string(buffer);
  };

  rclcpp::Time latest_arrival;
  bool latest_arrival_available = false;
  for (size_t i = 0; i < frame.arrival_times.size(); ++i) {
    if (!is_active(i)) {
      continue;
    }
    const auto& candidate_arrival = frame.arrival_times[i];
    if (candidate_arrival.nanoseconds() == 0) {
      continue;
    }
    if (!latest_arrival_available || candidate_arrival > latest_arrival) {
      latest_arrival = candidate_arrival;
      latest_arrival_available = true;
    }
  }

  double sync_wait_ms = 0.0;
  double arrival_window_ms = 0.0;
  double ready_to_process_gap_ms = 0.0;
  if (arrival_available) {
    sync_wait_ms = static_cast<double>((processing_start - oldest_arrival).nanoseconds()) / 1e6;
  }
  if (arrival_available && latest_arrival_available) {
    arrival_window_ms = static_cast<double>((latest_arrival - oldest_arrival).nanoseconds()) / 1e6;
    ready_to_process_gap_ms = static_cast<double>((processing_start - latest_arrival).nanoseconds()) / 1e6;
  }
  if (sync_wait_ms < 0.0) {
    sync_wait_ms = 0.0;
  }
  if (arrival_window_ms < 0.0) {
    arrival_window_ms = 0.0;
  }
  if (ready_to_process_gap_ms < 0.0) {
    ready_to_process_gap_ms = 0.0;
  }

  double processing_latency_ms = static_cast<double>((publish_time - processing_start).nanoseconds()) / 1e6;

  auto summarise_lookup = [&](bool enabled, bool attempted, bool reused, double duration_ms) {
    if (!enabled) {
      return std::string{};
    }
    if (!attempted) {
      return reused ? std::string("reused") : std::string("skipped");
    }
    return format_ms(duration_ms);
  };

  auto summarise_projection = [&](bool enabled, bool attempted, bool succeeded, double duration_ms) {
    if (!enabled) {
      return std::string{};
    }
    if (!attempted) {
      return std::string("skipped");
    }
    std::ostringstream oss;
    oss << (succeeded ? "ok" : "fail") << " (" << format_ms(duration_ms) << ")";
    return oss.str();
  };

  std::string planar_lookup_summary = summarise_lookup(enable_planar_, planar_lookup_attempted, false, planar_lookup_ms);
  std::string equirect_lookup_summary =
      summarise_lookup(enable_equirectangular_, equirect_lookup_attempted, equirect_lookup_reused, equirect_lookup_ms);

  std::string planar_projection_summary = summarise_projection(enable_planar_, planar_attempted, planar_success, planar_ms);
  std::string equirect_projection_summary =
      summarise_projection(enable_equirectangular_, equirect_attempted, equirect_success, equirect_ms);

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

  std::ostringstream line;
  line << "Frame " << frame_key << " published [" << (success_list.empty() ? "?" : success_list) << "]";
  line << "\n  oldest stamp to publication:          " << format_ms(stamp_delay_ms)
       << "\n  oldest arrival to publication:        " << format_ms(arrival_delay_ms)
       << "\n    oldest arrival to processing start:   " << format_ms(sync_wait_ms);
  if (arrival_available && latest_arrival_available) {
    line << "\n      oldest arrival to latest arrival:     " << format_ms(arrival_window_ms)
         << "\n      latest arrival to processing start:   " << format_ms(ready_to_process_gap_ms);
  }
  line << "\n    processing start to publication:      " << format_ms(processing_latency_ms);
  line << "\n      transform lookups: ";
  if (enable_planar_) {
    line << "planar=" << planar_lookup_summary;
    if (enable_equirectangular_) {
      line << ", ";
    }
  }
  if (enable_equirectangular_) {
    line << "equirect=" << equirect_lookup_summary;
  }
  line << "\n      projections: ";
  if (enable_planar_) {
    line << "planar=" << planar_projection_summary;
    if (enable_equirectangular_) {
      line << ", ";
    }
  }
  if (enable_equirectangular_) {
    line << "equirect=" << equirect_projection_summary;
  }
  line << "\n      cameras used: " << active_count << "/" << camera_count;

  RCLCPP_INFO(get_logger(), "%s", line.str().c_str());

  restore_images();
}

void ImageReprojection::expireFrameAccumulators() {
  if (accumulator_timeout_sec_ <= 0.0) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(accumulator_timeout_sec_);
  for (auto it = frame_accumulators_.begin(); it != frame_accumulators_.end();) {
    if (now - it->second.created_at > timeout) {
      if (wait_all_publish_partial_) {
        const size_t ready_count = static_cast<size_t>(std::count(it->second.ready.begin(), it->second.ready.end(), true));
        RCLCPP_DEBUG(get_logger(), "Frame %ld timed out: publishing %zu/%zu cameras", it->first, ready_count,
                     it->second.ready.size());
        if (ready_count > 0) processFrame(it->first, it->second, it->second.ready);
      }
      it = frame_accumulators_.erase(it);
    } else {
      ++it;
    }
  }
}

void ImageReprojection::cleanupAccumulators(const rclcpp::Time& current_stamp) {
  if (accumulator_timeout_sec_ <= 0.0) return;
  if (aggregation_mode_ == AggregationMode::WaitForAll) expireFrameAccumulators();

  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(accumulator_timeout_sec_);
  for (auto& pending : pending_images_) {
    for (auto it = pending.begin(); it != pending.end();) {
      if (current_stamp - it->second.stamp > timeout) {
        it = pending.erase(it);
      } else {
        ++it;
      }
    }
  }
}

bool ImageReprojection::lookupCameraTransforms(const rclcpp::Time& stamp,
                                               const std::vector<std::string>& camera_frames,
                                               const std::string& target_frame,
                                               std::vector<tf2::Transform>& transforms,
                                               bool planar_projection,
                                               const std::vector<bool>* camera_mask) {
  if (!tf_buffer_) {
    RCLCPP_ERROR(get_logger(), "TF buffer is not initialised.");
    return false;
  }

  if (target_frame.empty()) {
    RCLCPP_ERROR(get_logger(), "Target frame for reprojection is empty");
    return false;
  }

  const size_t camera_count = camera_frames.size();
  transforms.resize(camera_count);

  const bool use_mask = camera_mask && camera_mask->size() == camera_count;
  auto is_active = [&](size_t idx) {
    if (!use_mask) {
      return true;
    }
    return (*camera_mask)[idx];
  };

  auto& cached_transforms = planar_projection ? cached_planar_transforms_ : cached_equirect_transforms_;
  auto& cache_ready = planar_projection ? planar_tf_ready_ : equirect_tf_ready_;
  const bool use_cache = !recompute_every_frame_;
  const tf2::Duration timeout = tf2::durationFromSec(transform_timeout_sec_);

  if (cached_transforms.size() < camera_count) {
    cached_transforms.resize(camera_count, tf2::Transform::getIdentity());
  }
  if (cache_ready.size() < camera_count) {
    cache_ready.resize(camera_count, false);
  }

  bool cache_updated = false;

  if (use_cache) {
    bool fully_cached = true;
    for (size_t i = 0; i < camera_count; ++i) {
      if (!is_active(i)) {
        continue;
      }
      if (!cache_ready[i]) {
        fully_cached = false;
        break;
      }
    }
    if (fully_cached) {
      std::copy(cached_transforms.begin(), cached_transforms.begin() + camera_count, transforms.begin());
      return true;
    }
  }

  for (size_t i = 0; i < camera_count; ++i) {
    if (!is_active(i)) {
      transforms[i].setIdentity();
      continue;
    }

    if (camera_frames[i].empty()) {
      RCLCPP_ERROR(get_logger(), "Camera %zu provided an empty frame_id in CameraInfo.", i);
      return false;
    }

    if (camera_frames[i] == target_frame) {
      transforms[i].setIdentity();
      if (use_cache) {
        const bool was_ready = cache_ready[i];
        cached_transforms[i] = transforms[i];
        cache_ready[i] = true;
        if (planar_projection) {
          updatePlanarWarpCache(i);
        } else {
          updateEquirectWarpCache(i);
        }
        if (!was_ready) {
          cache_updated = true;
        }
      }
      continue;
    }

    try {
      const geometry_msgs::msg::TransformStamped tf_msg =
          tf_buffer_->lookupTransform(camera_frames[i], target_frame, recompute_every_frame_ ? stamp : rclcpp::Time(0), timeout);
      tf2::fromMsg(tf_msg.transform, transforms[i]);
      if (use_cache) {
        const bool was_ready = cache_ready[i];
        cached_transforms[i] = transforms[i];
        cache_ready[i] = true;
        if (planar_projection) {
          updatePlanarWarpCache(i);
        } else {
          updateEquirectWarpCache(i);
        }
        if (!was_ready) {
          cache_updated = true;
        }
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to lookup transform from '%s' to '%s': %s",
                           target_frame.c_str(), camera_frames[i].c_str(), ex.what());
      return false;
    }
  }

  if (cache_updated) {
    if (use_gpu_ && !recompute_every_frame_) {
      if (planar_projection && !planar_dominant_map_ready_ && planar_blend_factor_ == 0.0)
        rebuildPlanarDominantWarpCache();
      if (!planar_projection && !equirect_dominant_map_ready_ && equirect_blend_factor_ == 0.0)
        rebuildEquirectDominantWarpCache();
      prepareCudaProjection(planar_projection);
    }
    markGstConfigDirty();
    exportGstConfigIfReady();
  }

  return true;
}

void ImageReprojection::cameraInfoCallback(size_t index, const CameraInfo::ConstSharedPtr& info) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (index >= input_configs_.size()) return;
  CameraIntrinsics intr;
  if (!extractIntrinsics(info, intr, get_logger())) return;
  if (intrinsics_ready_[index]) {
    const auto& previous = static_intrinsics_[index];
    if (previous.fx == intr.fx && previous.fy == intr.fy && previous.cx == intr.cx && previous.cy == intr.cy &&
        previous.width == intr.width && previous.height == intr.height && camera_frame_ids_[index] == info->header.frame_id) {
      return;
    }
  }
  const bool intrinsics_were_ready = intrinsics_ready_[index];
  const std::string previous_frame_id = camera_frame_ids_[index];

  static_intrinsics_[index] = intr;
  camera_frame_ids_[index] = info->header.frame_id;
  intrinsics_ready_[index] = true;

  bool require_export = !intrinsics_were_ready || previous_frame_id != camera_frame_ids_[index];

  // If target frames are not set, pick this one as default
  if (enable_planar_ && planar_frame_id_.empty()) {
    planar_frame_id_ = camera_frame_ids_[index];
    require_export = true;
  }
  if (enable_equirectangular_ && equirect_frame_id_.empty()) {
    equirect_frame_id_ = camera_frame_ids_[index];
    require_export = true;
  }

  // Cache static transforms if possible (time 0)
  if (enable_planar_ && !planar_frame_id_.empty()) {
    try {
      const bool was_ready = planar_tf_ready_[index];
      const auto tf_msg = tf_buffer_->lookupTransform(camera_frame_ids_[index], planar_frame_id_, rclcpp::Time(0),
                                                      tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, cached_planar_transforms_[index]);
      planar_tf_ready_[index] = true;
      updatePlanarWarpCache(index);
      if (!was_ready) {
        require_export = true;
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(get_logger(), "Planar transform not yet available for cam %zu: %s", index, ex.what());
    }
  }
  if (enable_equirectangular_ && !equirect_frame_id_.empty()) {
    try {
      const bool was_ready = equirect_tf_ready_[index];
      const auto tf_msg = tf_buffer_->lookupTransform(camera_frame_ids_[index], equirect_frame_id_, rclcpp::Time(0),
                                                      tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, cached_equirect_transforms_[index]);
      equirect_tf_ready_[index] = true;
      updateEquirectWarpCache(index);
      if (!was_ready) {
        require_export = true;
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(get_logger(), "Equirect transform not yet available for cam %zu: %s", index, ex.what());
    }
  }

  if (use_gpu_ && !recompute_every_frame_) {
    if (enable_planar_) {
      if (!planar_dominant_map_ready_ && planar_blend_factor_ == 0.0) rebuildPlanarDominantWarpCache();
      prepareCudaProjection(true);
    }
    if (enable_equirectangular_) {
      if (!equirect_dominant_map_ready_ && equirect_blend_factor_ == 0.0) rebuildEquirectDominantWarpCache();
      prepareCudaProjection(false);
    }
  }

  if (require_export) {
    markGstConfigDirty();
    exportGstConfigIfReady();
  }
}

void ImageReprojection::updatePlanarWarpCache(size_t index) {
  ++planar_warp_version_;
  if (!enable_planar_ || recompute_every_frame_) {
    return;
  }
  if (index >= input_configs_.size()) {
    return;
  }
  planar_dominant_map_ready_ = false;
  planar_warp_ready_[index] = false;
  if (!intrinsics_ready_[index] || !planar_tf_ready_[index]) {
    return;
  }

  const int width = planar_width_;
  const int height = planar_height_;
  if (width <= 0 || height <= 0) {
    return;
  }

  const size_t pixel_count = static_cast<size_t>(width) * height;
  auto& mapping = planar_warp_maps_[index];
  mapping.resize(pixel_count);

  const auto& intr = static_intrinsics_[index];
  const tf2::Transform& transform = cached_planar_transforms_[index];
  const double fx_in = intr.fx;
  const double fy_in = intr.fy;
  const double cx_in = intr.cx;
  const double cy_in = intr.cy;
  const int width_in = intr.width;
  const int height_in = intr.height;

  const auto& x_norm = planar_x_norm_;
  const auto& y_norm = planar_y_norm_;

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t pixel_index = static_cast<size_t>(v) * width + u;
      auto& entry = mapping[pixel_index];

      const tf2::Vector3 point_virtual(x_norm[u] * planar_depth_, y_norm[v] * planar_depth_, planar_depth_);
      const tf2::Vector3 point_input = transform * point_virtual;

      const double z = point_input.z();
      if (z <= kEpsilon) {
        entry = PixelMapping{};
        continue;
      }

      const double inv_z = 1.0 / z;
      const double u_in = fx_in * (point_input.x() * inv_z) + cx_in;
      const double v_in = fy_in * (point_input.y() * inv_z) + cy_in;

      if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) || v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
        entry = PixelMapping{};
        continue;
      }

      entry.u = static_cast<float>(u_in);
      entry.v = static_cast<float>(v_in);
    }
  }

  planar_warp_ready_[index] = true;
}

void ImageReprojection::updateEquirectWarpCache(size_t index) {
  ++equirect_warp_version_;
  if (!enable_equirectangular_ || recompute_every_frame_) {
    return;
  }
  if (index >= input_configs_.size()) {
    return;
  }
  equirect_dominant_map_ready_ = false;
  equirect_warp_ready_[index] = false;
  if (!intrinsics_ready_[index] || !equirect_tf_ready_[index]) {
    return;
  }

  const int width = equirect_width_;
  const int height = equirect_height_;
  if (width <= 0 || height <= 0) {
    return;
  }

  const size_t pixel_count = static_cast<size_t>(width) * height;
  auto& mapping = equirect_warp_maps_[index];
  mapping.resize(pixel_count);

  const auto& intr = static_intrinsics_[index];
  const tf2::Transform& transform = cached_equirect_transforms_[index];
  const double fx_in = intr.fx;
  const double fy_in = intr.fy;
  const double cx_in = intr.cx;
  const double cy_in = intr.cy;
  const int width_in = intr.width;
  const int height_in = intr.height;

  for (int v = 0; v < height; ++v) {
    const double sin_lat = equirect_sin_lat_[v];
    const double cos_lat = equirect_cos_lat_[v];

    for (int u = 0; u < width; ++u) {
      const size_t pixel_index = static_cast<size_t>(v) * width + u;
      auto& entry = mapping[pixel_index];
      const double sin_lon = equirect_sin_lon_[u];
      const double cos_lon = equirect_cos_lon_[u];

      tf2::Vector3 direction(cos_lat * sin_lon, -sin_lat, cos_lat * cos_lon);
      const tf2::Vector3 point_virtual = direction * equirect_radius_;
      const tf2::Vector3 point_input = transform * point_virtual;

      const double z = point_input.z();
      if (z <= kEpsilon) {
        entry = PixelMapping{};
        continue;
      }

      const double inv_z = 1.0 / z;
      const double u_in = fx_in * (point_input.x() * inv_z) + cx_in;
      const double v_in = fy_in * (point_input.y() * inv_z) + cy_in;

      if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) || v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
        entry = PixelMapping{};
        continue;
      }

      entry.u = static_cast<float>(u_in);
      entry.v = static_cast<float>(v_in);
    }
  }

  equirect_warp_ready_[index] = true;
}

void ImageReprojection::preparePlanarScratchBuffers(size_t camera_count) const {
  if (!enable_planar_) {
    return;
  }
  const int width = planar_width_;
  const int height = planar_height_;
  if (width <= 0 || height <= 0) {
    return;
  }

  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (planar_accumulators_.size() != camera_count) {
    planar_accumulators_.resize(camera_count);
  }
  if (planar_weights_.size() != camera_count) {
    planar_weights_.resize(camera_count);
  }

  for (size_t i = 0; i < camera_count; ++i) {
    auto& acc = planar_accumulators_[i];
    auto& weights = planar_weights_[i];
    const size_t acc_size = pixel_count * 3;
    if (acc.size() != acc_size) {
      acc.assign(acc_size, 0.0f);
    } else {
      std::fill(acc.begin(), acc.end(), 0.0f);
    }
    if (weights.size() != pixel_count) {
      weights.assign(pixel_count, 0.0f);
    } else {
      std::fill(weights.begin(), weights.end(), 0.0f);
    }
  }
}

void ImageReprojection::prepareEquirectScratchBuffers(size_t camera_count) const {
  if (!enable_equirectangular_) {
    return;
  }
  const int width = equirect_width_;
  const int height = equirect_height_;
  if (width <= 0 || height <= 0) {
    return;
  }

  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (equirect_accumulators_.size() != camera_count) {
    equirect_accumulators_.resize(camera_count);
  }
  if (equirect_weights_.size() != camera_count) {
    equirect_weights_.resize(camera_count);
  }

  for (size_t i = 0; i < camera_count; ++i) {
    auto& acc = equirect_accumulators_[i];
    auto& weights = equirect_weights_[i];
    const size_t acc_size = pixel_count * 3;
    if (acc.size() != acc_size) {
      acc.assign(acc_size, 0.0f);
    } else {
      std::fill(acc.begin(), acc.end(), 0.0f);
    }
    if (weights.size() != pixel_count) {
      weights.assign(pixel_count, 0.0f);
    } else {
      std::fill(weights.begin(), weights.end(), 0.0f);
    }
  }
}

bool ImageReprojection::renderDominantWarpCache(const std::vector<BgrImage>& input_images,
                                                const std::vector<CameraIntrinsics>& intrinsics,
                                                size_t active_count,
                                                int width,
                                                int height,
                                                const std::vector<DominantPixelMapping>& mapping,
                                                bool ready,
                                                sensor_msgs::msg::Image& output_image) const {
  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (!ready || mapping.size() != pixel_count || active_count != input_images.size()) return false;
  for (size_t i = 0; i < input_images.size(); ++i) {
    if (input_images[i].width != intrinsics[i].width || input_images[i].height != intrinsics[i].height) return false;
  }

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.assign(pixel_count * 3, 0);
  for (size_t pixel = 0; pixel < pixel_count; ++pixel) {
    const auto& entry = mapping[pixel];
    if (entry.camera == std::numeric_limits<uint32_t>::max()) continue;
    const auto& image = input_images[entry.camera];
    const uint8_t* top = image.data.data() + entry.source_offset;
    const uint8_t* bottom = top + (entry.down_step ? static_cast<size_t>(image.width) * 3 : 0);
    const double dx = entry.dx;
    const double dy = entry.dy;
    const size_t base = pixel * 3;
    for (size_t channel = 0; channel < 3; ++channel) {
      const float upper = static_cast<float>((1.0 - dx) * top[channel] + dx * top[entry.right_step + channel]);
      const float lower = static_cast<float>((1.0 - dx) * bottom[channel] + dx * bottom[entry.right_step + channel]);
      const float value = static_cast<float>((1.0 - dy) * upper + dy * lower);
      output_image.data[base + channel] = static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
    }
  }
  return true;
}

void ImageReprojection::prepareCudaProjection(bool planar_projection) const {
#ifdef HAVE_CUDA
  if (!use_gpu_ || recompute_every_frame_) return;
  const int width = planar_projection ? planar_width_ : equirect_width_;
  const int height = planar_projection ? planar_height_ : equirect_height_;
  const auto& warp_maps = planar_projection ? planar_warp_maps_ : equirect_warp_maps_;
  const auto& warp_ready = planar_projection ? planar_warp_ready_ : equirect_warp_ready_;
  const auto& dominant_map = planar_projection ? planar_dominant_map_ : equirect_dominant_map_;
  const bool dominant_ready = planar_projection ? planar_dominant_map_ready_ : equirect_dominant_map_ready_;
  const uint64_t version = planar_projection ? planar_warp_version_ : equirect_warp_version_;
  const size_t camera_count = input_configs_.size();
  if (width <= 0 || height <= 0 || camera_count == 0 || warp_maps.size() != camera_count ||
      warp_ready.size() != camera_count) return;
  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (!std::all_of(warp_ready.begin(), warp_ready.end(), [](bool ready) { return ready; })) return;

  if (!cuda_state_) cuda_state_ = std::make_unique<CudaState>();
  auto& state = planar_projection ? cuda_state_->planar : cuda_state_->equirect;
  if (state.version != version) state.reset(version);
  const bool need_multi = !state.multi && !state.multi_failed;
  const bool need_dominant = dominant_ready && dominant_map.size() == pixel_count &&
                             !state.dominant && !state.dominant_failed;
  if (!need_multi && !need_dominant) return;
  std::vector<size_t> camera_bytes(camera_count);
  std::vector<unsigned> widths(camera_count), heights(camera_count), strides(camera_count);
  for (size_t i = 0; i < camera_count; ++i) {
    const auto& intr = static_intrinsics_[i];
    if (intr.width <= 0 || intr.height <= 0 || warp_maps[i].size() != pixel_count) return;
    widths[i] = static_cast<unsigned>(intr.width);
    heights[i] = static_cast<unsigned>(intr.height);
    strides[i] = widths[i] * 3;
    camera_bytes[i] = static_cast<size_t>(strides[i]) * heights[i];
  }

  if (need_multi) {
    std::vector<ReprojectionCudaPixelMap> maps(camera_count * pixel_count);
    for (size_t i = 0; i < camera_count; ++i) {
      for (size_t p = 0; p < pixel_count; ++p) {
        maps[i * pixel_count + p] = {warp_maps[i][p].u, warp_maps[i][p].v};
      }
    }
    state.multi = reprojection_cuda_create_multi(
        maps.data(), pixel_count, static_cast<unsigned>(width), static_cast<unsigned>(height),
        static_cast<unsigned>(width * 3), camera_bytes.data(), widths.data(), heights.data(),
        strides.data(), static_cast<unsigned>(camera_count));
    if (!state.multi) {
      state.multi_failed = true;
      RCLCPP_WARN(get_logger(), "CUDA multi-camera setup failed; using CPU for partial or blended frames");
    }
  }
  if (need_dominant) {
    std::vector<ReprojectionCudaMap> maps(pixel_count);
    for (size_t p = 0; p < pixel_count; ++p) {
      const auto& source = dominant_map[p];
      auto& destination = maps[p];
      destination.camera = source.camera;
      destination.source_offset = source.source_offset;
      destination.right_step = source.right_step;
      destination.down_step = source.camera == std::numeric_limits<uint32_t>::max()
                                  ? 0 : source.down_step * strides[source.camera];
      destination.dx = source.dx;
      destination.dy = source.dy;
    }
    state.dominant = reprojection_cuda_create(
        maps.data(), pixel_count, static_cast<unsigned>(width), static_cast<unsigned>(height),
        static_cast<unsigned>(width * 3), camera_bytes.data(), static_cast<unsigned>(camera_count));
    if (!state.dominant) {
      state.dominant_failed = true;
      RCLCPP_WARN(get_logger(), "CUDA dominant-map setup failed; using CPU");
    }
  }
#else
  (void)planar_projection;
#endif
}

bool ImageReprojection::renderCuda(const std::vector<BgrImage>& input_images,
                                   const std::vector<CameraIntrinsics>& intrinsics,
                                   const std::vector<bool>* camera_mask,
                                   bool planar_projection,
                                   sensor_msgs::msg::Image& output_image) const {
#ifndef HAVE_CUDA
  (void)input_images;
  (void)intrinsics;
  (void)camera_mask;
  (void)planar_projection;
  (void)output_image;
  return false;
#else
  if (!use_gpu_ || recompute_every_frame_ || input_images.size() != input_configs_.size()) return false;
  const int width = planar_projection ? planar_width_ : equirect_width_;
  const int height = planar_projection ? planar_height_ : equirect_height_;
  const float blend = static_cast<float>(planar_projection ? planar_blend_factor_ : equirect_blend_factor_);
  const auto& warp_maps = planar_projection ? planar_warp_maps_ : equirect_warp_maps_;
  const auto& warp_ready = planar_projection ? planar_warp_ready_ : equirect_warp_ready_;
  const auto& dominant_map = planar_projection ? planar_dominant_map_ : equirect_dominant_map_;
  const bool dominant_ready = planar_projection ? planar_dominant_map_ready_ : equirect_dominant_map_ready_;
  const size_t camera_count = input_images.size();
  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (warp_maps.size() != camera_count || warp_ready.size() != camera_count ||
      intrinsics.size() != camera_count || width <= 0 || height <= 0) return false;

  std::vector<const uint8_t*> inputs(camera_count, nullptr);
  std::vector<size_t> camera_bytes(camera_count);
  size_t active_count = 0;
  for (size_t i = 0; i < camera_count; ++i) {
    const auto& cached = static_intrinsics_[i];
    if (cached.width < 0 || cached.height < 0) return false;
    camera_bytes[i] = static_cast<size_t>(cached.width) * cached.height * 3;
    const bool active = !camera_mask || camera_mask->size() != camera_count || (*camera_mask)[i];
    if (!active) continue;
    const auto& image = input_images[i];
    const auto& frame_intr = intrinsics[i];
    if (!warp_ready[i] || warp_maps[i].size() != pixel_count ||
        image.width != cached.width || image.height != cached.height ||
        image.data.size() < camera_bytes[i] ||
        frame_intr.fx != cached.fx || frame_intr.fy != cached.fy ||
        frame_intr.cx != cached.cx || frame_intr.cy != cached.cy ||
        frame_intr.width != cached.width || frame_intr.height != cached.height) return false;
    inputs[i] = image.data.data();
    ++active_count;
  }
  if (active_count == 0) return false;

  const bool use_dominant = blend == 0.0f && active_count == camera_count &&
                            dominant_ready && dominant_map.size() == pixel_count;
  prepareCudaProjection(planar_projection);
  if (!cuda_state_) return false;
  auto& state = planar_projection ? cuda_state_->planar : cuda_state_->equirect;
  ReprojectionCuda* context = use_dominant ? state.dominant : state.multi;
  if (!context) return false;

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.resize(pixel_count * 3);
  const bool success = use_dominant
                           ? reprojection_cuda_render(context, inputs.data(), output_image.data.data())
                           : reprojection_cuda_render_multi(context, inputs.data(), output_image.data.data(), blend);
  if (!success) {
    if (use_dominant) {
      reprojection_cuda_destroy(state.dominant);
      state.dominant = nullptr;
      state.dominant_failed = true;
    } else {
      reprojection_cuda_destroy(state.multi);
      state.multi = nullptr;
      state.multi_failed = true;
    }
    RCLCPP_WARN(get_logger(), "CUDA reprojection failed; using CPU");
  }
  return success;
#endif
}

bool ImageReprojection::reprojectEquirectangular(const std::vector<BgrImage>& input_images,
                                                 const std::vector<CameraIntrinsics>& intrinsics,
                                                 const std::vector<tf2::Transform>& transforms,
                                                 const std::vector<bool>* camera_mask,
                                                 sensor_msgs::msg::Image& output_image) const {
  if (input_images.empty()) {
    RCLCPP_WARN(get_logger(), "No input images available for equirectangular reprojection.");
    return false;
  }

  if (intrinsics.size() != input_images.size() || transforms.size() != input_images.size()) {
    RCLCPP_ERROR(get_logger(), "Inconsistent input sizes: %zu images, %zu intrinsics, %zu transforms.", input_images.size(),
                 intrinsics.size(), transforms.size());
    return false;
  }

  const int width = equirect_width_;
  const int height = equirect_height_;
  if (width <= 0 || height <= 0) {
    RCLCPP_ERROR(get_logger(), "Equirectangular projection output dimensions must be positive.");
    return false;
  }

  const size_t pixel_count = static_cast<size_t>(height) * width;
  const size_t camera_count = input_images.size();

  const bool has_mask = camera_mask && camera_mask->size() == camera_count;
  auto camera_enabled = [&](size_t idx) { return !has_mask || (*camera_mask)[idx]; };

  size_t active_count = 0;
  for (size_t i = 0; i < camera_count; ++i) {
    if (camera_enabled(i)) {
      ++active_count;
    }
  }
  if (active_count == 0) {
    RCLCPP_WARN(get_logger(), "No active input cameras available for equirectangular reprojection.");
    return false;
  }

  if (renderCuda(input_images, intrinsics, camera_mask, false, output_image)) return true;

  if (equirect_blend_factor_ == 0.0 && !recompute_every_frame_ &&
      renderDominantWarpCache(input_images, intrinsics, active_count, width, height, equirect_dominant_map_,
                              equirect_dominant_map_ready_, output_image)) return true;

  const bool dominant_only = equirect_blend_factor_ == 0.0;
  if (!dominant_only) {
    prepareEquirectScratchBuffers(camera_count);
  }
  auto& accumulators = equirect_accumulators_;
  auto& weights = equirect_weights_;

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.resize(output_image.step * output_image.height);
  std::vector<uint8_t> occupied;
  if (dominant_only) {
    std::fill(output_image.data.begin(), output_image.data.end(), 0);
    occupied.assign(pixel_count, 0);
  }
  auto write_sample = [&](size_t pixel_index, const std::array<float, 3>& colour,
                          std::vector<float>& acc, std::vector<float>& weight_buffer) {
    const size_t base_index = pixel_index * 3;
    if (dominant_only) {
      if (occupied[pixel_index]) return;
      occupied[pixel_index] = 1;
      output_image.data[base_index] = static_cast<uint8_t>(std::clamp(colour[0], 0.0f, 255.0f));
      output_image.data[base_index + 1] = static_cast<uint8_t>(std::clamp(colour[1], 0.0f, 255.0f));
      output_image.data[base_index + 2] = static_cast<uint8_t>(std::clamp(colour[2], 0.0f, 255.0f));
    } else {
      acc[base_index] += colour[0];
      acc[base_index + 1] += colour[1];
      acc[base_index + 2] += colour[2];
      weight_buffer[pixel_index] += 1.0f;
    }
  };

  bool use_precomputed = !recompute_every_frame_;
  if (use_precomputed) {
    if (equirect_warp_maps_.size() < camera_count) {
      use_precomputed = false;
    }
    for (size_t i = 0; i < camera_count && use_precomputed; ++i) {
      if (!camera_enabled(i)) {
        continue;
      }
      if (i >= equirect_warp_ready_.size() || !equirect_warp_ready_[i]) {
        use_precomputed = false;
        break;
      }
      if (equirect_warp_maps_[i].size() != pixel_count) {
        use_precomputed = false;
        break;
      }
      if (intrinsics[i].width != input_images[i].width || intrinsics[i].height != input_images[i].height) {
        use_precomputed = false;
        break;
      }
    }
  }

  for (size_t i = 0; i < camera_count; ++i) {
    if (!camera_enabled(i)) {
      continue;
    }

    const auto& intr = intrinsics[i];
    const BgrImage& image = input_images[i];
    if (image.width <= 0 || image.height <= 0) {
      continue;
    }

    auto& acc = accumulators[i];
    auto& weight_buffer = weights[i];

    if (use_precomputed) {
      const auto& mapping = equirect_warp_maps_[i];
      const int width_in = image.width;
      const int height_in = image.height;
      const double max_u = static_cast<double>(width_in - 1);
      const double max_v = static_cast<double>(height_in - 1);

      for (size_t pixel_index = 0; pixel_index < pixel_count; ++pixel_index) {
        if (dominant_only && occupied[pixel_index]) continue;
        const auto& entry = mapping[pixel_index];
        if (!std::isfinite(entry.u) || !std::isfinite(entry.v)) {
          continue;
        }
        if (entry.u < 0.0f || entry.u > max_u || entry.v < 0.0f || entry.v > max_v) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, static_cast<double>(entry.u), static_cast<double>(entry.v));
        write_sample(pixel_index, colour, acc, weight_buffer);
      }
      continue;
    }

    const tf2::Transform& transform = transforms[i];
    const double fx_in = intr.fx;
    const double fy_in = intr.fy;
    const double cx_in = intr.cx;
    const double cy_in = intr.cy;
    const int width_in = image.width;
    const int height_in = image.height;

    if (intr.width != width_in || intr.height != height_in) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "CameraInfo resolution (%dx%d) differs from image "
                       "resolution (%dx%d). Using image resolution for bounds.",
                       intr.width, intr.height, width_in, height_in);
    }

    for (int v = 0; v < height; ++v) {
      const double sin_lat = equirect_sin_lat_[v];
      const double cos_lat = equirect_cos_lat_[v];

      for (int u = 0; u < width; ++u) {
        const size_t pixel_index = static_cast<size_t>(v) * width + u;
        if (dominant_only && occupied[pixel_index]) continue;
        const double sin_lon = equirect_sin_lon_[u];
        const double cos_lon = equirect_cos_lon_[u];

        tf2::Vector3 direction(cos_lat * sin_lon, -sin_lat, cos_lat * cos_lon);
        const tf2::Vector3 point_virtual = direction * equirect_radius_;
        const tf2::Vector3 point_input = transform * point_virtual;

        const double z = point_input.z();
        if (z <= kEpsilon) {
          continue;
        }

        const double inv_z = 1.0 / z;
        const double u_in = fx_in * (point_input.x() * inv_z) + cx_in;
        const double v_in = fy_in * (point_input.y() * inv_z) + cy_in;

        if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) || v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, u_in, v_in);
        write_sample(pixel_index, colour, acc, weight_buffer);
      }
    }
  }

  if (dominant_only) return true;

  const auto colour_from_accumulator = [](const std::vector<float>& acc, float weight, size_t base_index) {
    std::array<float, 3> colour{0.0f, 0.0f, 0.0f};
    if (weight > 0.0f) {
      const float inv_weight = 1.0f / weight;
      colour[0] = acc[base_index + 0] * inv_weight;
      colour[1] = acc[base_index + 1] * inv_weight;
      colour[2] = acc[base_index + 2] * inv_weight;
    }
    return colour;
  };

  const float blend_factor = static_cast<float>(equirect_blend_factor_);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t pixel_index = static_cast<size_t>(v) * width + u;
      const size_t base_index = pixel_index * 3;
      uint8_t* pixel = &output_image.data[base_index];

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
        const float blended_value =
            static_cast<float>((1.0f - blend_factor) * dominant_colour[channel] + blend_factor * average_colour[channel]);
        pixel[channel] = static_cast<uint8_t>(std::clamp(blended_value, 0.0f, 255.0f));
      }
    }
  }

  return true;
}

bool ImageReprojection::toBgrImage(const Image::ConstSharedPtr& msg, BgrImage& output, const rclcpp::Logger& logger) {
  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  if (width <= 0 || height <= 0) {
    RCLCPP_ERROR(logger, "Received image with non-positive dimensions (%d x %d).", width, height);
    return false;
  }

  const auto encoding = msg->encoding;
  const int src_channels =
      (encoding == sensor_msgs::image_encodings::BGR8 || encoding == sensor_msgs::image_encodings::RGB8)
          ? 3
          : (encoding == sensor_msgs::image_encodings::MONO8
                 ? 1
                 : (encoding == sensor_msgs::image_encodings::BGRA8 || encoding == sensor_msgs::image_encodings::RGBA8 ? 4 : 0));

  if (src_channels == 0) {
    RCLCPP_ERROR(logger,
                 "Unsupported image encoding '%s'. Supported encodings: BGR8, "
                 "RGB8, BGRA8, RGBA8, MONO8.",
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

  const uint8_t* src_data = msg->data.data();
  uint8_t* dst_data = output.data.data();
  const size_t dst_stride = static_cast<size_t>(width) * 3;

  if (encoding == sensor_msgs::image_encodings::BGR8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t* src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t* dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
      std::copy(src_row, src_row + dst_stride, dst_row);
    }
    return true;
  }

  if (encoding == sensor_msgs::image_encodings::RGB8) {
    for (int row = 0; row < height; ++row) {
      const uint8_t* src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t* dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
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
      const uint8_t* src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t* dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
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
      const uint8_t* src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t* dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
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
      const uint8_t* src_row = src_data + static_cast<size_t>(row) * src_stride;
      uint8_t* dst_row = dst_data + static_cast<size_t>(row) * dst_stride;
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
               "Unsupported image encoding '%s'. Supported encodings: BGR8, "
               "RGB8, BGRA8, RGBA8, MONO8.",
               encoding.c_str());
  return false;
}

bool ImageReprojection::extractIntrinsics(const CameraInfo::ConstSharedPtr& info,
                                          CameraIntrinsics& intrinsics,
                                          const rclcpp::Logger& logger) {
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

bool ImageReprojection::reprojectPlanar(const std::vector<BgrImage>& input_images,
                                        const std::vector<CameraIntrinsics>& intrinsics,
                                        const std::vector<tf2::Transform>& transforms,
                                        const std::vector<bool>* camera_mask,
                                        sensor_msgs::msg::Image& output_image) const {
  if (input_images.empty()) {
    RCLCPP_WARN(get_logger(), "No input images available for reprojection.");
    return false;
  }

  if (intrinsics.size() != input_images.size() || transforms.size() != input_images.size()) {
    RCLCPP_ERROR(get_logger(), "Inconsistent input sizes: %zu images, %zu intrinsics, %zu transforms.", input_images.size(),
                 intrinsics.size(), transforms.size());
    return false;
  }

  const int width = planar_width_;
  const int height = planar_height_;
  if (width <= 0 || height <= 0) {
    RCLCPP_ERROR(get_logger(), "Planar projection output dimensions must be positive.");
    return false;
  }

  const size_t pixel_count = static_cast<size_t>(height) * width;
  const size_t camera_count = input_images.size();

  const bool has_mask = camera_mask && camera_mask->size() == camera_count;
  auto camera_enabled = [&](size_t idx) { return !has_mask || (*camera_mask)[idx]; };

  size_t active_count = 0;
  for (size_t i = 0; i < camera_count; ++i) {
    if (camera_enabled(i)) {
      ++active_count;
    }
  }
  if (active_count == 0) {
    RCLCPP_WARN(get_logger(), "No active input cameras available for planar reprojection.");
    return false;
  }

  if (renderCuda(input_images, intrinsics, camera_mask, true, output_image)) return true;

  if (planar_blend_factor_ == 0.0 && !recompute_every_frame_ &&
      renderDominantWarpCache(input_images, intrinsics, active_count, width, height, planar_dominant_map_,
                              planar_dominant_map_ready_, output_image)) return true;

  const bool dominant_only = planar_blend_factor_ == 0.0;
  if (!dominant_only) {
    preparePlanarScratchBuffers(camera_count);
  }
  auto& accumulators = planar_accumulators_;
  auto& weights = planar_weights_;

  output_image.height = static_cast<uint32_t>(height);
  output_image.width = static_cast<uint32_t>(width);
  output_image.encoding = sensor_msgs::image_encodings::BGR8;
  output_image.is_bigendian = false;
  output_image.step = static_cast<uint32_t>(width * 3);
  output_image.data.resize(output_image.step * output_image.height);
  std::vector<uint8_t> occupied;
  if (dominant_only) {
    std::fill(output_image.data.begin(), output_image.data.end(), 0);
    occupied.assign(pixel_count, 0);
  }
  auto write_sample = [&](size_t pixel_index, const std::array<float, 3>& colour,
                          std::vector<float>& acc, std::vector<float>& weight_buffer) {
    const size_t base_index = pixel_index * 3;
    if (dominant_only) {
      if (occupied[pixel_index]) return;
      occupied[pixel_index] = 1;
      output_image.data[base_index] = static_cast<uint8_t>(std::clamp(colour[0], 0.0f, 255.0f));
      output_image.data[base_index + 1] = static_cast<uint8_t>(std::clamp(colour[1], 0.0f, 255.0f));
      output_image.data[base_index + 2] = static_cast<uint8_t>(std::clamp(colour[2], 0.0f, 255.0f));
    } else {
      acc[base_index] += colour[0];
      acc[base_index + 1] += colour[1];
      acc[base_index + 2] += colour[2];
      weight_buffer[pixel_index] += 1.0f;
    }
  };

  bool use_precomputed = !recompute_every_frame_;
  if (use_precomputed) {
    if (planar_warp_maps_.size() < camera_count) {
      use_precomputed = false;
    }
    for (size_t i = 0; i < camera_count && use_precomputed; ++i) {
      if (!camera_enabled(i)) {
        continue;
      }
      if (i >= planar_warp_ready_.size() || !planar_warp_ready_[i]) {
        use_precomputed = false;
        break;
      }
      if (planar_warp_maps_[i].size() != pixel_count) {
        use_precomputed = false;
        break;
      }
      if (intrinsics[i].width != input_images[i].width || intrinsics[i].height != input_images[i].height) {
        use_precomputed = false;
        break;
      }
    }
  }

  const auto& x_norm = planar_x_norm_;
  const auto& y_norm = planar_y_norm_;

  for (size_t i = 0; i < camera_count; ++i) {
    if (!camera_enabled(i)) {
      continue;
    }

    const auto& intr = intrinsics[i];
    const BgrImage& image = input_images[i];
    if (image.width <= 0 || image.height <= 0) {
      continue;
    }

    auto& acc = accumulators[i];
    auto& weight_buffer = weights[i];

    if (use_precomputed) {
      const auto& mapping = planar_warp_maps_[i];
      const int width_in = image.width;
      const int height_in = image.height;
      const double max_u = static_cast<double>(width_in - 1);
      const double max_v = static_cast<double>(height_in - 1);

      for (size_t pixel_index = 0; pixel_index < pixel_count; ++pixel_index) {
        if (dominant_only && occupied[pixel_index]) continue;
        const auto& entry = mapping[pixel_index];
        if (!std::isfinite(entry.u) || !std::isfinite(entry.v)) {
          continue;
        }

        if (entry.u < 0.0f || entry.u > max_u || entry.v < 0.0f || entry.v > max_v) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, static_cast<double>(entry.u), static_cast<double>(entry.v));
        write_sample(pixel_index, colour, acc, weight_buffer);
      }
      continue;
    }

    const tf2::Transform& transform = transforms[i];
    const double fx_in = intr.fx;
    const double fy_in = intr.fy;
    const double cx_in = intr.cx;
    const double cy_in = intr.cy;
    const int width_in = image.width;
    const int height_in = image.height;

    if (intr.width != width_in || intr.height != height_in) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "CameraInfo resolution (%dx%d) differs from image "
                       "resolution (%dx%d). Using image resolution for bounds.",
                       intr.width, intr.height, width_in, height_in);
    }

    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        const size_t pixel_index = static_cast<size_t>(v) * width + u;
        if (dominant_only && occupied[pixel_index]) continue;
        const tf2::Vector3 point_virtual(x_norm[u] * planar_depth_, y_norm[v] * planar_depth_, planar_depth_);
        const tf2::Vector3 point_input = transform * point_virtual;

        const double z = point_input.z();
        if (z <= kEpsilon) {
          continue;
        }

        const double inv_z = 1.0 / z;
        const double u_in = fx_in * (point_input.x() * inv_z) + cx_in;
        const double v_in = fy_in * (point_input.y() * inv_z) + cy_in;

        if (u_in < 0.0 || u_in > static_cast<double>(width_in - 1) || v_in < 0.0 || v_in > static_cast<double>(height_in - 1)) {
          continue;
        }

        const std::array<float, 3> colour = bilinearSample(image, u_in, v_in);
        write_sample(pixel_index, colour, acc, weight_buffer);
      }
    }
  }

  if (dominant_only) return true;

  const auto colour_from_accumulator = [](const std::vector<float>& acc, float weight, size_t base_index) {
    std::array<float, 3> colour{0.0f, 0.0f, 0.0f};
    if (weight > 0.0f) {
      const float inv_weight = 1.0f / weight;
      colour[0] = acc[base_index + 0] * inv_weight;
      colour[1] = acc[base_index + 1] * inv_weight;
      colour[2] = acc[base_index + 2] * inv_weight;
    }
    return colour;
  };

  const float blend_factor = static_cast<float>(planar_blend_factor_);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const size_t pixel_index = static_cast<size_t>(v) * width + u;
      const size_t base_index = pixel_index * 3;
      uint8_t* pixel = &output_image.data[base_index];

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
        const float blended_value =
            static_cast<float>((1.0f - blend_factor) * dominant_colour[channel] + blend_factor * average_colour[channel]);
        pixel[channel] = static_cast<uint8_t>(std::clamp(blended_value, 0.0f, 255.0f));
      }
    }
  }

  return true;
}

std::array<float, 3> ImageReprojection::bilinearSample(const BgrImage& image, double u, double v) {
  const int width = image.width;
  const int height = image.height;

  const auto clamp_coord = [](int value, int max_value) { return std::max(0, std::min(value, max_value)); };

  const int x0 = clamp_coord(static_cast<int>(std::floor(u)), width - 1);
  const int y0 = clamp_coord(static_cast<int>(std::floor(v)), height - 1);
  const int x1 = clamp_coord(x0 + 1, width - 1);
  const int y1 = clamp_coord(y0 + 1, height - 1);

  const double dx = u - static_cast<double>(x0);
  const double dy = v - static_cast<double>(y0);

  const auto get_pixel = [&](int x, int y) {
    const size_t index = (static_cast<size_t>(y) * width + x) * 3;
    return std::array<float, 3>{static_cast<float>(image.data[index + 0]), static_cast<float>(image.data[index + 1]),
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

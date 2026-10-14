// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0
 
#include <gst/base/gstaggregator.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <json-glib/json-glib.h>
#include <math.h>
#include <string.h>
#ifdef HAVE_CUDA
#include "reprojection_cuda.h"
#endif

#ifndef PACKAGE
#define PACKAGE "gst_image_reprojection"
#endif

#define GST_TYPE_IMAGE_REPROJECTION (gst_image_reprojection_get_type())
#define GST_IMAGE_REPROJECTION(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_IMAGE_REPROJECTION, GstImageReprojection))
#define GST_IMAGE_REPROJECTION_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_IMAGE_REPROJECTION, GstImageReprojectionClass))
#define GST_IS_IMAGE_REPROJECTION(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_IMAGE_REPROJECTION))
#define GST_IS_IMAGE_REPROJECTION_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_IMAGE_REPROJECTION))

#define GST_TYPE_IMAGE_REPROJECTION_PAD (gst_image_reprojection_pad_get_type())
#define GST_IMAGE_REPROJECTION_PAD(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_IMAGE_REPROJECTION_PAD, GstImageReprojectionPad))

#define GST_IMAGE_REPROJECTION_DEFAULT_BLEND 1.0

typedef enum {
  GST_IMAGE_REPROJECTION_MODE_AUTO = 0,
  GST_IMAGE_REPROJECTION_MODE_PLANAR,
  GST_IMAGE_REPROJECTION_MODE_EQUIRECT
} GstImageReprojectionMode;

typedef enum {
  GST_IMAGE_REPROJECTION_SYNC_WAIT_ALL,
  GST_IMAGE_REPROJECTION_SYNC_LEAD_LATEST
} GstImageReprojectionSyncMode;

static GType gst_image_reprojection_mode_get_type(void);
#define GST_TYPE_IMAGE_REPROJECTION_MODE (gst_image_reprojection_mode_get_type())

static gboolean json_object_get_boolean_member_or(JsonObject* object, const gchar* member, gboolean fallback) {
  if (!json_object_has_member(object, member)) {
    return fallback;
  }
  return json_object_get_boolean_member(object, member);
}

static const gchar* json_object_get_string_member_or(JsonObject* object, const gchar* member, const gchar* fallback) {
  if (!json_object_has_member(object, member)) {
    return fallback;
  }
  return json_object_get_string_member(object, member);
}

typedef struct {
  double fx;
  double fy;
  double cx;
  double cy;
  int width;
  int height;
  int map_width;
  int map_height;
  int stride;
  gsize plane_offset;
  double planar_matrix[16];
  double equirect_matrix[16];
  gboolean has_planar_matrix;
  gboolean has_equirect_matrix;
} GstImageReprojectionCamera;

typedef struct {
  gboolean enabled;
  gchar* frame_id;
  gint width;
  gint height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth;
  double blend_factor;
  gsize pixel_count;
  double* x_norm; /* width */
  double* y_norm; /* height */
} GstImageReprojectionPlanar;

typedef struct {
  gboolean enabled;
  gchar* frame_id;
  gint width;
  gint height;
  double hfov_rad;
  double vfov_rad;
  double radius;
  double blend_factor;
  gsize pixel_count;
  double* sin_lat; /* height */
  double* cos_lat; /* height */
  double* sin_lon; /* width */
  double* cos_lon; /* width */
} GstImageReprojectionEquirect;

typedef struct {
  float u;
  float v;
} GstImageReprojectionPixelMap;

typedef struct {
  guint camera;
  gsize source_offset;
  float dx;
  float dy;
  guint8 right_step;
  guint down_step;
} GstImageReprojectionDominantMap;

typedef struct _GstImageReprojectionPad {
  GstAggregatorPad parent;
  guint index;
  GstVideoInfo info;
} GstImageReprojectionPad;

typedef struct _GstImageReprojectionPadClass {
  GstAggregatorPadClass parent_class;
} GstImageReprojectionPadClass;

typedef struct _GstImageReprojection {
  GstAggregator parent;

  gchar* config_path;
  GstImageReprojectionMode mode_property;
  GstImageReprojectionMode active_mode;
  gboolean config_loaded;
  gboolean warned_pad_count;
  gboolean initial_events_pushed;
  GstImageReprojectionSyncMode sync_mode;
  double frame_timeout_sec;
  GstClockTime frame_time_tolerance;
  gboolean wait_all_publish_partial;

  GMutex lock;

  guint camera_count;
  GstImageReprojectionCamera* cameras;
  GstBuffer** latest_buffers;

  GstImageReprojectionPlanar planar;
  GstImageReprojectionEquirect equirect;

  GstImageReprojectionPixelMap** planar_maps; /* [camera][pixel] */
  GstImageReprojectionPixelMap** equirect_maps;
  GstImageReprojectionDominantMap* planar_dominant_map;
  GstImageReprojectionDominantMap* equirect_dominant_map;

  gboolean use_gpu;
#ifdef HAVE_CUDA
  gboolean gpu_dominant_disabled;
  gboolean gpu_multi_disabled;
  const GstImageReprojectionDominantMap* gpu_map;
  ReprojectionCuda* gpu_context;
  ReprojectionCuda* gpu_multi_context;
#endif

  float** planar_accumulators;
  float** planar_weights;
  float** equirect_accumulators;
  float** equirect_weights;

  GstCaps* src_caps;
  GstVideoInfo output_info;
  GstSegment segment;

  guint next_pad_index;
} GstImageReprojection;

typedef struct _GstImageReprojectionClass {
  GstAggregatorClass parent_class;
} GstImageReprojectionClass;

G_DEFINE_TYPE(GstImageReprojection, gst_image_reprojection, GST_TYPE_AGGREGATOR)
G_DEFINE_TYPE(GstImageReprojectionPad, gst_image_reprojection_pad, GST_TYPE_AGGREGATOR_PAD)

enum {
  PROP_0,
  PROP_CONFIG_PATH,
  PROP_PROJECTION_MODE,
  PROP_USE_GPU,
};

static GstStaticPadTemplate gst_image_reprojection_sink_template =
    GST_STATIC_PAD_TEMPLATE("sink_%u", GST_PAD_SINK, GST_PAD_REQUEST, GST_STATIC_CAPS("video/x-raw, format=(string)BGR"));

static GstStaticPadTemplate gst_image_reprojection_src_template =
    GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS("video/x-raw, format=(string)BGR"));

static const double kEpsilon = 1e-9;

/* Forward declarations */
static void gst_image_reprojection_set_property(GObject* object, guint prop_id, const GValue* value, GParamSpec* pspec);
static void gst_image_reprojection_get_property(GObject* object, guint prop_id, GValue* value, GParamSpec* pspec);
static void gst_image_reprojection_dispose(GObject* object);
static gboolean gst_image_reprojection_start(GstAggregator* aggregator);
static gboolean gst_image_reprojection_stop(GstAggregator* aggregator);
static GstFlowReturn gst_image_reprojection_flush(GstAggregator* aggregator);
static GstFlowReturn gst_image_reprojection_aggregate(GstAggregator* aggregator, gboolean timeout);
static GstClockTime gst_image_reprojection_get_next_time(GstAggregator* aggregator);
static GstAggregatorPad* gst_image_reprojection_create_new_pad(GstAggregator* aggregator,
                                                               GstPadTemplate* templ,
                                                               const gchar* name,
                                                               const GstCaps* caps);
static gboolean gst_image_reprojection_sink_event(GstAggregator* aggregator, GstAggregatorPad* pad, GstEvent* event);

static gboolean gst_image_reprojection_load_config(GstImageReprojection* self, GError** error);
static void gst_image_reprojection_clear_config(GstImageReprojection* self);
static gboolean gst_image_reprojection_update_planar_maps(GstImageReprojection* self, GError** error);
static gboolean gst_image_reprojection_update_equirect_maps(GstImageReprojection* self, GError** error);
#ifdef HAVE_CUDA
static void gst_image_reprojection_prepare_cuda(GstImageReprojection* self,
                                                const GstImageReprojectionDominantMap* dominant,
                                                guint width, guint height);
static void gst_image_reprojection_prepare_cuda_multi(GstImageReprojection* self,
                                                      GstImageReprojectionPixelMap** pixel_maps,
                                                      guint width, guint height);
static gboolean gst_image_reprojection_try_cuda_multi(GstImageReprojection* self,
                                                       GstBuffer** buffers, GstMapInfo* maps,
                                                       GstImageReprojectionPixelMap** pixel_maps,
                                                       guint width, guint height, float blend,
                                                       GstBuffer** out_buffer_ptr);
#endif

static gboolean gst_image_reprojection_ensure_output_caps(GstImageReprojection* self, GError** error);
static gboolean gst_image_reprojection_push_initial_events(GstImageReprojection* self, GstAggregator* aggregator);

static void gst_image_reprojection_reset_scratch_planar(GstImageReprojection* self);
static void gst_image_reprojection_reset_scratch_equirect(GstImageReprojection* self);

static void gst_image_reprojection_class_init(GstImageReprojectionClass* klass) {
  GObjectClass* gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass* element_class = GST_ELEMENT_CLASS(klass);
  GstAggregatorClass* aggregator_class = GST_AGGREGATOR_CLASS(klass);

  gobject_class->set_property = gst_image_reprojection_set_property;
  gobject_class->get_property = gst_image_reprojection_get_property;
  gobject_class->dispose = gst_image_reprojection_dispose;

  g_object_class_install_property(
      gobject_class, PROP_CONFIG_PATH,
      g_param_spec_string("config-path", "Configuration Path", "Path to JSON configuration exported by image_reprojection node",
                          NULL, G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY));

  g_object_class_install_property(
      gobject_class, PROP_PROJECTION_MODE,
      g_param_spec_enum("projection-mode", "Projection Mode",
                        "Projection to generate (auto/planar/equirectangular). In auto mode, planar is preferred if enabled",
                        GST_TYPE_IMAGE_REPROJECTION_MODE, GST_IMAGE_REPROJECTION_MODE_AUTO,
                        G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY));

  g_object_class_install_property(
      gobject_class, PROP_USE_GPU,
      g_param_spec_boolean("use-gpu", "Use GPU",
                           "Use CUDA for reprojection when available and input dimensions match the configuration",
                           FALSE, G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY));

  gst_element_class_add_pad_template(element_class, gst_static_pad_template_get(&gst_image_reprojection_sink_template));
  gst_element_class_add_pad_template(element_class, gst_static_pad_template_get(&gst_image_reprojection_src_template));

  gst_element_class_set_static_metadata(element_class, "Image Reprojection", "Filter/Effect/Video",
                                        "Reprojects multiple input video streams using various projection methods",
                                        "Lennart Reiher <lennart.reiher@ika.rwth-aachen.de>");

  aggregator_class->start = gst_image_reprojection_start;
  aggregator_class->stop = gst_image_reprojection_stop;
  aggregator_class->flush = gst_image_reprojection_flush;
  aggregator_class->aggregate = gst_image_reprojection_aggregate;
  aggregator_class->get_next_time = gst_image_reprojection_get_next_time;
  aggregator_class->create_new_pad = gst_image_reprojection_create_new_pad;
  aggregator_class->sink_event = gst_image_reprojection_sink_event;
}

static void gst_image_reprojection_pad_class_init(GstImageReprojectionPadClass* klass) { (void)klass; }

static void gst_image_reprojection_pad_init(GstImageReprojectionPad* pad) {
  pad->index = G_MAXUINT;
  gst_video_info_init(&pad->info);
}

static void gst_image_reprojection_init(GstImageReprojection* self) {
  self->config_path = NULL;
  self->mode_property = GST_IMAGE_REPROJECTION_MODE_AUTO;
  self->active_mode = GST_IMAGE_REPROJECTION_MODE_AUTO;
  self->config_loaded = FALSE;
  self->warned_pad_count = FALSE;
  self->initial_events_pushed = FALSE;
  self->sync_mode = GST_IMAGE_REPROJECTION_SYNC_WAIT_ALL;
  self->frame_timeout_sec = 1.0;
  self->frame_time_tolerance = 5 * GST_MSECOND;
  self->wait_all_publish_partial = TRUE;
  gst_aggregator_set_force_live(GST_AGGREGATOR(self), TRUE);
  gst_aggregator_set_ignore_inactive_pads(GST_AGGREGATOR(self), TRUE);
  g_mutex_init(&self->lock);

  self->camera_count = 0;
  self->cameras = NULL;
  self->latest_buffers = NULL;

  memset(&self->planar, 0, sizeof(self->planar));
  memset(&self->equirect, 0, sizeof(self->equirect));

  self->planar_maps = NULL;
  self->equirect_maps = NULL;
  self->planar_dominant_map = NULL;
  self->equirect_dominant_map = NULL;
  self->use_gpu = FALSE;
#ifdef HAVE_CUDA
  self->gpu_dominant_disabled = FALSE;
  self->gpu_multi_disabled = FALSE;
  self->gpu_map = NULL;
  self->gpu_context = NULL;
  self->gpu_multi_context = NULL;
#endif

  self->planar_accumulators = NULL;
  self->planar_weights = NULL;
  self->equirect_accumulators = NULL;
  self->equirect_weights = NULL;

  self->src_caps = NULL;
  gst_video_info_init(&self->output_info);
  gst_segment_init(&self->segment, GST_FORMAT_TIME);

  self->next_pad_index = 0;
}

static void gst_image_reprojection_dispose(GObject* object) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(object);

  gst_image_reprojection_clear_config(self);
  g_clear_pointer(&self->config_path, g_free);
  g_mutex_clear(&self->lock);

  G_OBJECT_CLASS(gst_image_reprojection_parent_class)->dispose(object);
}

static void gst_image_reprojection_set_property(GObject* object, guint prop_id, const GValue* value, GParamSpec* pspec) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(object);

  switch (prop_id) {
    case PROP_CONFIG_PATH: {
      const gchar* path = g_value_get_string(value);
      g_mutex_lock(&self->lock);
      g_free(self->config_path);
      self->config_path = path ? g_strdup(path) : NULL;
      self->config_loaded = FALSE;
      g_mutex_unlock(&self->lock);
      break;
    }
    case PROP_PROJECTION_MODE:
      g_mutex_lock(&self->lock);
      self->mode_property = (GstImageReprojectionMode)g_value_get_enum(value);
      self->config_loaded = FALSE;
      g_mutex_unlock(&self->lock);
      break;
    case PROP_USE_GPU:
      g_mutex_lock(&self->lock);
      self->use_gpu = g_value_get_boolean(value);
      g_mutex_unlock(&self->lock);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void gst_image_reprojection_get_property(GObject* object, guint prop_id, GValue* value, GParamSpec* pspec) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(object);

  switch (prop_id) {
    case PROP_CONFIG_PATH:
      g_value_set_string(value, self->config_path);
      break;
    case PROP_PROJECTION_MODE:
      g_value_set_enum(value, self->mode_property);
      break;
    case PROP_USE_GPU:
      g_value_set_boolean(value, self->use_gpu);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static GstAggregatorPad* gst_image_reprojection_create_new_pad(GstAggregator* aggregator,
                                                               GstPadTemplate* templ,
                                                               const gchar* name,
                                                               const GstCaps* caps) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(aggregator);
  GstImageReprojectionPad* pad;
  const gboolean auto_name = !name || g_strcmp0(name, "sink_%u") == 0;
  guint index = self->next_pad_index;
  if (!auto_name && g_str_has_prefix(name, "sink_")) {
    gchar* end = NULL;
    const guint64 parsed = g_ascii_strtoull(name + 5, &end, 10);
    if (end != name + 5 && *end == '\0' && parsed < G_MAXUINT) index = (guint)parsed;
  }
  gchar* generated_name = auto_name ? g_strdup_printf("sink_%u", index) : NULL;
  pad = g_object_new(GST_TYPE_IMAGE_REPROJECTION_PAD, "name", auto_name ? generated_name : name, "direction", GST_PAD_SINK,
                     "template", templ, NULL);
  g_free(generated_name);
  pad->index = index;
  self->next_pad_index = MAX(self->next_pad_index, index + 1);

  GST_DEBUG_OBJECT(self, "Created new sink pad %u", pad->index);

  if (caps) {
    gst_video_info_from_caps(&pad->info, caps);
  }

  return GST_AGGREGATOR_PAD(pad);
}

static gboolean gst_image_reprojection_sink_event(GstAggregator* aggregator, GstAggregatorPad* pad, GstEvent* event) {
  GstImageReprojectionPad* ip = GST_IMAGE_REPROJECTION_PAD(pad);

  if (GST_EVENT_TYPE(event) == GST_EVENT_CAPS) {
    GstCaps* caps = NULL;
    gst_event_parse_caps(event, &caps);
    if (caps) {
      gst_video_info_from_caps(&ip->info, caps);
    }
  }

  return GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->sink_event(aggregator, pad, event);
}

static gboolean gst_image_reprojection_start(GstAggregator* aggregator) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(aggregator);
  GError* error = NULL;

  g_mutex_lock(&self->lock);
  gboolean ok = gst_image_reprojection_load_config(self, &error);
  g_mutex_unlock(&self->lock);

  if (!ok) {
    if (error) {
      GST_ERROR_OBJECT(self, "Failed to load config: %s", error->message);
      g_clear_error(&error);
    }
    return FALSE;
  }

#ifdef HAVE_CUDA
  if (self->use_gpu) {
    if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR &&
        self->planar.blend_factor == 0.0 && self->planar_dominant_map) {
      gst_image_reprojection_prepare_cuda(self, self->planar_dominant_map, self->planar.width, self->planar.height);
    } else if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_EQUIRECT &&
               self->equirect.blend_factor == 0.0 && self->equirect_dominant_map) {
      gst_image_reprojection_prepare_cuda(self, self->equirect_dominant_map,
                                          self->equirect.width, self->equirect.height);
    }
    if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR) {
      gst_image_reprojection_prepare_cuda_multi(self, self->planar_maps,
                                                self->planar.width, self->planar.height);
    } else if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_EQUIRECT) {
      gst_image_reprojection_prepare_cuda_multi(self, self->equirect_maps,
                                                self->equirect.width, self->equirect.height);
    }
  }
#endif

  if (GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->start) {
    return GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->start(aggregator);
  }

  return TRUE;
}

static gboolean gst_image_reprojection_stop(GstAggregator* aggregator) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(aggregator);

  g_mutex_lock(&self->lock);
  self->config_loaded = FALSE;
  self->initial_events_pushed = FALSE;
  g_mutex_unlock(&self->lock);

  if (GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->stop) {
    return GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->stop(aggregator);
  }
  return TRUE;
}

static GstFlowReturn gst_image_reprojection_flush(GstAggregator* aggregator) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(aggregator);
  for (guint i = 0; i < self->camera_count; ++i) {
    if (self->latest_buffers && self->latest_buffers[i]) {
      gst_buffer_unref(self->latest_buffers[i]);
      self->latest_buffers[i] = NULL;
    }
  }
  GstAggregatorClass* parent = GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class);
  return parent->flush ? parent->flush(aggregator) : GST_FLOW_OK;
}

static inline gboolean isfinitef(float v) { return isfinite(v); }

static inline float clampf(float v, float min_v, float max_v) {
  if (v < min_v) return min_v;
  if (v > max_v) return max_v;
  return v;
}

static inline void bilinear_sample(const guint8* data, guint width, guint height, guint stride, float u, float v,
                                   float out[3]) {
  if (width == 0 || height == 0) {
    out[0] = out[1] = out[2] = 0.0f;
    return;
  }

  float u_clamped = clampf(u, 0.0f, (float)(width - 1));
  float v_clamped = clampf(v, 0.0f, (float)(height - 1));

  guint x0 = (guint)floorf(u_clamped);
  guint y0 = (guint)floorf(v_clamped);
  guint x1 = MIN(x0 + 1, width - 1);
  guint y1 = MIN(y0 + 1, height - 1);

  float dx = u_clamped - (float)x0;
  float dy = v_clamped - (float)y0;

  const guint8* row0 = data + (gsize)y0 * stride;
  const guint8* row1 = data + (gsize)y1 * stride;

  const guint8* p00 = row0 + x0 * 3;
  const guint8* p10 = row0 + x1 * 3;
  const guint8* p01 = row1 + x0 * 3;
  const guint8* p11 = row1 + x1 * 3;

  for (guint c = 0; c < 3; ++c) {
    float val0 = (1.0f - dx) * (float)p00[c] + dx * (float)p10[c];
    float val1 = (1.0f - dx) * (float)p01[c] + dx * (float)p11[c];
    out[c] = (1.0f - dy) * val0 + dy * val1;
  }
}

static GstImageReprojectionDominantMap* gst_image_reprojection_build_dominant_map(
    GstImageReprojection* self, GstImageReprojectionPixelMap** maps, gsize pixel_count) {
  GstImageReprojectionDominantMap* dominant = g_new0(GstImageReprojectionDominantMap, pixel_count);
  for (gsize p = 0; p < pixel_count; ++p) dominant[p].camera = G_MAXUINT;
  for (guint i = 0; i < self->camera_count; ++i) {
    const guint width = self->cameras[i].map_width;
    const guint height = self->cameras[i].map_height;
    const guint stride = GST_ROUND_UP_4(width * 3);
    for (gsize p = 0; p < pixel_count; ++p) {
      GstImageReprojectionDominantMap* selected = &dominant[p];
      if (selected->camera != G_MAXUINT) continue;
      const float u = maps[i][p].u;
      const float v = maps[i][p].v;
      if (!isfinitef(u) || !isfinitef(v) || u < 0.0f || v < 0.0f || u > (float)(width - 1) ||
          v > (float)(height - 1)) continue;
      const guint x = (guint)floorf(u);
      const guint y = (guint)floorf(v);
      selected->camera = i;
      selected->source_offset = (gsize)y * stride + (gsize)x * 3;
      selected->dx = u - (float)x;
      selected->dy = v - (float)y;
      selected->right_step = x + 1 < width ? 3 : 0;
      selected->down_step = y + 1 < height ? stride : 0;
    }
  }
  return dominant;
}

#ifdef HAVE_CUDA
static void gst_image_reprojection_prepare_cuda(GstImageReprojection* self,
                                                const GstImageReprojectionDominantMap* dominant,
                                                guint width, guint height) {
  if (self->gpu_map == dominant || self->gpu_dominant_disabled) return;
  reprojection_cuda_destroy(self->gpu_context);
  self->gpu_context = NULL;
  self->gpu_map = dominant;
  const gsize pixel_count = (gsize)width * height;
  ReprojectionCudaMap* cuda_maps = g_new(ReprojectionCudaMap, pixel_count);
  size_t* camera_bytes = g_new(size_t, self->camera_count);
  for (guint i = 0; i < self->camera_count; ++i) {
    camera_bytes[i] = (size_t)GST_ROUND_UP_4(self->cameras[i].map_width * 3) *
                      self->cameras[i].map_height;
  }
  for (gsize p = 0; p < pixel_count; ++p) {
    cuda_maps[p].camera = dominant[p].camera;
    cuda_maps[p].source_offset = dominant[p].source_offset;
    cuda_maps[p].right_step = dominant[p].right_step;
    cuda_maps[p].down_step = dominant[p].down_step;
    cuda_maps[p].dx = dominant[p].dx;
    cuda_maps[p].dy = dominant[p].dy;
  }
  self->gpu_context = reprojection_cuda_create(
      cuda_maps, pixel_count, width, height, GST_VIDEO_INFO_COMP_STRIDE(&self->output_info, 0),
      camera_bytes, self->camera_count);
  g_free(cuda_maps);
  g_free(camera_bytes);
  if (!self->gpu_context) {
    GST_WARNING_OBJECT(self, "CUDA map setup failed; falling back to CPU");
    self->gpu_dominant_disabled = TRUE;
  }
}

static void gst_image_reprojection_prepare_cuda_multi(GstImageReprojection* self,
                                                      GstImageReprojectionPixelMap** pixel_maps,
                                                      guint width, guint height) {
  if (self->gpu_multi_context || self->gpu_multi_disabled) return;
  const gsize pixel_count = (gsize)width * height;
  _Static_assert(sizeof(GstImageReprojectionPixelMap) == sizeof(ReprojectionCudaPixelMap),
                 "CUDA pixel-map layout must match CPU pixel-map layout");
  ReprojectionCudaPixelMap* cuda_maps =
      g_new(ReprojectionCudaPixelMap, (gsize)self->camera_count * pixel_count);
  size_t* camera_bytes = g_new(size_t, self->camera_count);
  guint* widths = g_new(guint, self->camera_count);
  guint* heights = g_new(guint, self->camera_count);
  guint* strides = g_new(guint, self->camera_count);
  for (guint i = 0; i < self->camera_count; ++i) {
    widths[i] = self->cameras[i].map_width;
    heights[i] = self->cameras[i].map_height;
    strides[i] = GST_ROUND_UP_4(widths[i] * 3);
    camera_bytes[i] = (size_t)strides[i] * heights[i];
    memcpy(cuda_maps + (gsize)i * pixel_count, pixel_maps[i],
           pixel_count * sizeof(ReprojectionCudaPixelMap));
  }
  self->gpu_multi_context = reprojection_cuda_create_multi(
      cuda_maps, pixel_count, width, height, GST_VIDEO_INFO_COMP_STRIDE(&self->output_info, 0),
      camera_bytes, widths, heights, strides, self->camera_count);
  g_free(cuda_maps);
  g_free(camera_bytes);
  g_free(widths);
  g_free(heights);
  g_free(strides);
  if (!self->gpu_multi_context) {
    GST_WARNING_OBJECT(self, "CUDA multi-camera setup failed; falling back to CPU");
    self->gpu_multi_disabled = TRUE;
  }
}

static gboolean gst_image_reprojection_try_cuda_multi(GstImageReprojection* self,
                                                       GstBuffer** buffers, GstMapInfo* maps,
                                                       GstImageReprojectionPixelMap** pixel_maps,
                                                       guint width, guint height, float blend,
                                                       GstBuffer** out_buffer_ptr) {
  if (!self->use_gpu || self->gpu_multi_disabled) return FALSE;
  for (guint i = 0; i < self->camera_count; ++i) {
    if (!buffers[i]) continue;
    const GstImageReprojectionCamera* cam = &self->cameras[i];
    if (cam->width != cam->map_width || cam->height != cam->map_height ||
        cam->stride != GST_ROUND_UP_4(cam->map_width * 3)) return FALSE;
    const gsize bytes = (gsize)cam->stride * cam->height;
    if (cam->plane_offset > maps[i].size || bytes > maps[i].size - cam->plane_offset) return FALSE;
  }
  gst_image_reprojection_prepare_cuda_multi(self, pixel_maps, width, height);
  if (!self->gpu_multi_context) return FALSE;
  GstBuffer* out = gst_buffer_new_allocate(NULL, GST_VIDEO_INFO_SIZE(&self->output_info), NULL);
  GstMapInfo out_map;
  if (!out || !gst_buffer_map(out, &out_map, GST_MAP_WRITE)) {
    if (out) gst_buffer_unref(out);
    return FALSE;
  }
  const guint8** inputs = g_new0(const guint8*, self->camera_count);
  for (guint i = 0; i < self->camera_count; ++i) {
    if (buffers[i]) inputs[i] = maps[i].data + self->cameras[i].plane_offset;
  }
  const gboolean success = reprojection_cuda_render_multi(
      self->gpu_multi_context, inputs, out_map.data, blend);
  g_free(inputs);
  gst_buffer_unmap(out, &out_map);
  if (success) {
    *out_buffer_ptr = out;
    return TRUE;
  }
  GST_WARNING_OBJECT(self, "CUDA multi-camera reprojection failed; falling back to CPU");
  self->gpu_multi_disabled = TRUE;
  gst_buffer_unref(out);
  return FALSE;
}
#endif

static GstFlowReturn gst_image_reprojection_process_zero_blend(GstImageReprojection* self, GstBuffer** buffers,
                                                                GstMapInfo* maps, GstImageReprojectionPixelMap** pixel_maps,
                                                                const GstImageReprojectionDominantMap* dominant,
                                                                guint width, guint height, GstBuffer** out_buffer_ptr) {
  const gsize pixel_count = (gsize)width * height;
  const guint output_stride = GST_VIDEO_INFO_COMP_STRIDE(&self->output_info, 0);
  gboolean use_dominant = dominant != NULL;
  for (guint i = 0; i < self->camera_count; ++i) {
    const GstImageReprojectionCamera* cam = &self->cameras[i];
    if (!buffers[i] || cam->width != cam->map_width || cam->height != cam->map_height ||
        cam->stride != GST_ROUND_UP_4(cam->map_width * 3)) {
      use_dominant = FALSE;
      break;
    }
  }

#ifdef HAVE_CUDA
  if (!use_dominant &&
      gst_image_reprojection_try_cuda_multi(self, buffers, maps, pixel_maps,
                                            width, height, 0.0f, out_buffer_ptr)) return GST_FLOW_OK;
#endif

  GstBuffer* out = gst_buffer_new_allocate(NULL, GST_VIDEO_INFO_SIZE(&self->output_info), NULL);
  GstMapInfo out_map;
  if (!out || !gst_buffer_map(out, &out_map, GST_MAP_WRITE)) {
    if (out) gst_buffer_unref(out);
    return GST_FLOW_ERROR;
  }

  if (use_dominant) {
#ifdef HAVE_CUDA
    if (self->use_gpu && !self->gpu_dominant_disabled) {
      gst_image_reprojection_prepare_cuda(self, dominant, width, height);
      if (self->gpu_context) {
        const guint8** inputs = g_new(const guint8*, self->camera_count);
        gboolean layout_ok = TRUE;
        for (guint i = 0; i < self->camera_count; ++i) {
          const GstImageReprojectionCamera* cam = &self->cameras[i];
          const gsize bytes = (gsize)cam->stride * cam->height;
          if (cam->plane_offset > maps[i].size || bytes > maps[i].size - cam->plane_offset) {
            layout_ok = FALSE;
            break;
          }
          inputs[i] = maps[i].data + cam->plane_offset;
        }
        if (layout_ok) {
          if (reprojection_cuda_render(self->gpu_context, inputs, out_map.data)) {
            g_free(inputs);
            gst_buffer_unmap(out, &out_map);
            *out_buffer_ptr = out;
            return GST_FLOW_OK;
          }
          GST_WARNING_OBJECT(self, "CUDA reprojection failed; falling back to CPU");
          self->gpu_dominant_disabled = TRUE;
        }
        g_free(inputs);
      }
    }
#endif
    memset(out_map.data, 0, out_map.size);
    for (gsize p = 0; p < pixel_count; ++p) {
      const GstImageReprojectionDominantMap* entry = &dominant[p];
      if (entry->camera == G_MAXUINT) continue;
      const GstImageReprojectionCamera* cam = &self->cameras[entry->camera];
      const guint8* top = maps[entry->camera].data + cam->plane_offset + entry->source_offset;
      const guint8* bottom = top + entry->down_step;
      guint8* dst = out_map.data + (p / width) * output_stride + (p % width) * 3;
      for (guint c = 0; c < 3; ++c) {
        const float upper = (1.0f - entry->dx) * top[c] + entry->dx * top[entry->right_step + c];
        const float lower = (1.0f - entry->dx) * bottom[c] + entry->dx * bottom[entry->right_step + c];
        dst[c] = (guint8)clampf((1.0f - entry->dy) * upper + entry->dy * lower, 0.0f, 255.0f);
      }
    }
  } else {
    memset(out_map.data, 0, out_map.size);
    guint8* occupied = g_new0(guint8, pixel_count);
    for (guint i = 0; i < self->camera_count; ++i) {
      if (!buffers[i]) continue;
      const GstImageReprojectionCamera* cam = &self->cameras[i];
      const guint8* data = maps[i].data + cam->plane_offset;
      for (gsize p = 0; p < pixel_count; ++p) {
        if (occupied[p]) continue;
        const float u = pixel_maps[i][p].u;
        const float v = pixel_maps[i][p].v;
        if (!isfinitef(u) || !isfinitef(v) || u < 0.0f || v < 0.0f || u > (float)(cam->width - 1) ||
            v > (float)(cam->height - 1)) continue;
        float colour[3];
        bilinear_sample(data, cam->width, cam->height, cam->stride, u, v, colour);
        guint8* dst = out_map.data + (p / width) * output_stride + (p % width) * 3;
        for (guint c = 0; c < 3; ++c) dst[c] = (guint8)clampf(colour[c], 0.0f, 255.0f);
        occupied[p] = 1;
      }
    }
    g_free(occupied);
  }

  gst_buffer_unmap(out, &out_map);
  *out_buffer_ptr = out;
  return GST_FLOW_OK;
}

static GstFlowReturn gst_image_reprojection_process_planar(GstImageReprojection* self,
                                                           GstBuffer** buffers,
                                                           GstMapInfo* maps,
                                                           GstBuffer** out_buffer_ptr) {
  const guint camera_count = self->camera_count;
  const gsize pixel_count = self->planar.pixel_count;
  if (pixel_count == 0) {
    return GST_FLOW_ERROR;
  }
  if (self->planar.blend_factor == 0.0) {
    return gst_image_reprojection_process_zero_blend(self, buffers, maps, self->planar_maps,
                                                     self->planar_dominant_map, self->planar.width,
                                                     self->planar.height, out_buffer_ptr);
  }

#ifdef HAVE_CUDA
  if (gst_image_reprojection_try_cuda_multi(self, buffers, maps, self->planar_maps,
                                            self->planar.width, self->planar.height,
                                            (float)self->planar.blend_factor, out_buffer_ptr)) return GST_FLOW_OK;
#endif

  gst_image_reprojection_reset_scratch_planar(self);

  for (guint i = 0; i < camera_count; ++i) {
    if (!buffers[i]) continue;
    const guint8* data = maps[i].data + self->cameras[i].plane_offset;
    if (!data) continue;

    const GstImageReprojectionPixelMap* mapping = self->planar_maps[i];
    if (!mapping) continue;

    float* acc = self->planar_accumulators[i];
    float* weights = self->planar_weights[i];
    if (!acc || !weights) continue;

    const guint width_in = self->cameras[i].width > 0 ? (guint)self->cameras[i].width : 0;
    const guint height_in = self->cameras[i].height > 0 ? (guint)self->cameras[i].height : 0;

    if (width_in == 0 || height_in == 0) continue;

    const float max_u = (float)(width_in - 1);
    const float max_v = (float)(height_in - 1);

    for (gsize p = 0; p < pixel_count; ++p) {
      float u = mapping[p].u;
      float v = mapping[p].v;
      if (!isfinitef(u) || !isfinitef(v)) continue;
      if (u < 0.0f || u > max_u || v < 0.0f || v > max_v) continue;

      float colour[3];
      bilinear_sample(data, width_in, height_in, self->cameras[i].stride, u, v, colour);

      gsize base = p * 3;
      acc[base + 0] += colour[0];
      acc[base + 1] += colour[1];
      acc[base + 2] += colour[2];
      weights[p] += 1.0f;
    }
  }

  GstBuffer* out = gst_buffer_new_allocate(NULL, GST_VIDEO_INFO_SIZE(&self->output_info), NULL);
  if (!out) {
    return GST_FLOW_ERROR;
  }

  GstMapInfo out_map;
  if (!gst_buffer_map(out, &out_map, GST_MAP_WRITE)) {
    gst_buffer_unref(out);
    return GST_FLOW_ERROR;
  }

  guint8* out_data = out_map.data;
  memset(out_data, 0, out_map.size);
  const guint output_stride = GST_VIDEO_INFO_COMP_STRIDE(&self->output_info, 0);
  const float blend = (float)self->planar.blend_factor;

  for (gsize p = 0; p < pixel_count; ++p) {
    float total_weight = 0.0f;
    float max_weight = -1.0f;
    guint dominant_idx = 0;

    for (guint i = 0; i < camera_count; ++i) {
      float w = self->planar_weights[i] ? self->planar_weights[i][p] : 0.0f;
      if (w <= 0.0f) continue;
      total_weight += w;
      if (w > max_weight) {
        max_weight = w;
        dominant_idx = i;
      }
    }

    guint8* dst = out_data + (p / self->planar.width) * output_stride + (p % self->planar.width) * 3;
    if (total_weight <= 0.0f) {
      dst[0] = dst[1] = dst[2] = 0;
      continue;
    }

    float dominant_colour[3] = {0.0f, 0.0f, 0.0f};
    float average_colour[3] = {0.0f, 0.0f, 0.0f};

    for (guint i = 0; i < camera_count; ++i) {
      float* acc = self->planar_accumulators[i];
      float* weights = self->planar_weights[i];
      if (!acc || !weights) continue;
      float w = weights[p];
      if (w <= 0.0f) continue;

      float inv_w = 1.0f / w;
      gsize base = p * 3;
      float colour[3] = {acc[base + 0] * inv_w, acc[base + 1] * inv_w, acc[base + 2] * inv_w};

      float normalized = w / total_weight;
      average_colour[0] += normalized * colour[0];
      average_colour[1] += normalized * colour[1];
      average_colour[2] += normalized * colour[2];

      if (i == dominant_idx) {
        dominant_colour[0] = colour[0];
        dominant_colour[1] = colour[1];
        dominant_colour[2] = colour[2];
      }
    }

    for (guint c = 0; c < 3; ++c) {
      float blended = (1.0f - blend) * dominant_colour[c] + blend * average_colour[c];
      dst[c] = (guint8)clampf(blended, 0.0f, 255.0f);
    }
  }

  gst_buffer_unmap(out, &out_map);
  *out_buffer_ptr = out;
  return GST_FLOW_OK;
}

static GstFlowReturn gst_image_reprojection_process_equirect(GstImageReprojection* self,
                                                             GstBuffer** buffers,
                                                             GstMapInfo* maps,
                                                             GstBuffer** out_buffer_ptr) {
  const guint camera_count = self->camera_count;
  const gsize pixel_count = self->equirect.pixel_count;
  if (pixel_count == 0) {
    return GST_FLOW_ERROR;
  }
  if (self->equirect.blend_factor == 0.0) {
    return gst_image_reprojection_process_zero_blend(self, buffers, maps, self->equirect_maps,
                                                     self->equirect_dominant_map, self->equirect.width,
                                                     self->equirect.height, out_buffer_ptr);
  }

#ifdef HAVE_CUDA
  if (gst_image_reprojection_try_cuda_multi(self, buffers, maps, self->equirect_maps,
                                            self->equirect.width, self->equirect.height,
                                            (float)self->equirect.blend_factor, out_buffer_ptr)) return GST_FLOW_OK;
#endif

  gst_image_reprojection_reset_scratch_equirect(self);

  for (guint i = 0; i < camera_count; ++i) {
    if (!buffers[i]) continue;
    const guint8* data = maps[i].data + self->cameras[i].plane_offset;
    if (!data) continue;

    const GstImageReprojectionPixelMap* mapping = self->equirect_maps[i];
    if (!mapping) continue;

    float* acc = self->equirect_accumulators[i];
    float* weights = self->equirect_weights[i];
    if (!acc || !weights) continue;

    const guint width_in = self->cameras[i].width > 0 ? (guint)self->cameras[i].width : 0;
    const guint height_in = self->cameras[i].height > 0 ? (guint)self->cameras[i].height : 0;

    if (width_in == 0 || height_in == 0) continue;

    const float max_u = (float)(width_in - 1);
    const float max_v = (float)(height_in - 1);

    for (gsize p = 0; p < pixel_count; ++p) {
      float u = mapping[p].u;
      float v = mapping[p].v;
      if (!isfinitef(u) || !isfinitef(v)) continue;
      if (u < 0.0f || u > max_u || v < 0.0f || v > max_v) continue;

      float colour[3];
      bilinear_sample(data, width_in, height_in, self->cameras[i].stride, u, v, colour);

      gsize base = p * 3;
      acc[base + 0] += colour[0];
      acc[base + 1] += colour[1];
      acc[base + 2] += colour[2];
      weights[p] += 1.0f;
    }
  }

  GstBuffer* out = gst_buffer_new_allocate(NULL, GST_VIDEO_INFO_SIZE(&self->output_info), NULL);
  if (!out) {
    return GST_FLOW_ERROR;
  }

  GstMapInfo out_map;
  if (!gst_buffer_map(out, &out_map, GST_MAP_WRITE)) {
    gst_buffer_unref(out);
    return GST_FLOW_ERROR;
  }

  guint8* out_data = out_map.data;
  memset(out_data, 0, out_map.size);
  const guint output_stride = GST_VIDEO_INFO_COMP_STRIDE(&self->output_info, 0);
  const float blend = (float)self->equirect.blend_factor;

  for (gsize p = 0; p < pixel_count; ++p) {
    float total_weight = 0.0f;
    float max_weight = -1.0f;
    guint dominant_idx = 0;

    for (guint i = 0; i < camera_count; ++i) {
      float w = self->equirect_weights[i] ? self->equirect_weights[i][p] : 0.0f;
      if (w <= 0.0f) continue;
      total_weight += w;
      if (w > max_weight) {
        max_weight = w;
        dominant_idx = i;
      }
    }

    guint8* dst = out_data + (p / self->equirect.width) * output_stride + (p % self->equirect.width) * 3;
    if (total_weight <= 0.0f) {
      dst[0] = dst[1] = dst[2] = 0;
      continue;
    }

    float dominant_colour[3] = {0.0f, 0.0f, 0.0f};
    float average_colour[3] = {0.0f, 0.0f, 0.0f};

    for (guint i = 0; i < camera_count; ++i) {
      float* acc = self->equirect_accumulators[i];
      float* weights = self->equirect_weights[i];
      if (!acc || !weights) continue;
      float w = weights[p];
      if (w <= 0.0f) continue;

      float inv_w = 1.0f / w;
      gsize base = p * 3;
      float colour[3] = {acc[base + 0] * inv_w, acc[base + 1] * inv_w, acc[base + 2] * inv_w};

      float normalized = w / total_weight;
      average_colour[0] += normalized * colour[0];
      average_colour[1] += normalized * colour[1];
      average_colour[2] += normalized * colour[2];

      if (i == dominant_idx) {
        dominant_colour[0] = colour[0];
        dominant_colour[1] = colour[1];
        dominant_colour[2] = colour[2];
      }
    }

    for (guint c = 0; c < 3; ++c) {
      float blended = (1.0f - blend) * dominant_colour[c] + blend * average_colour[c];
      dst[c] = (guint8)clampf(blended, 0.0f, 255.0f);
    }
  }

  gst_buffer_unmap(out, &out_map);
  *out_buffer_ptr = out;
  return GST_FLOW_OK;
}

static GstClockTime gst_image_reprojection_get_next_time(GstAggregator* aggregator) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(aggregator);
  for (GList* l = GST_ELEMENT(self)->sinkpads; l; l = l->next) {
    GstImageReprojectionPad* pad = GST_IMAGE_REPROJECTION_PAD(l->data);
    if (pad->index != 0) continue;
    GstBuffer* buffer = gst_aggregator_pad_peek_buffer(GST_AGGREGATOR_PAD(pad));
    if (!buffer) return GST_CLOCK_TIME_NONE;
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    gst_buffer_unref(buffer);
    return GST_CLOCK_TIME_IS_VALID(pts) ?
               gst_segment_to_running_time(&GST_AGGREGATOR_PAD(pad)->segment, GST_FORMAT_TIME, pts) :
               gst_aggregator_simple_get_next_time(aggregator);
  }
  return GST_CLOCK_TIME_NONE;
}

static GstFlowReturn gst_image_reprojection_aggregate(GstAggregator* aggregator, gboolean timeout) {
  GstImageReprojection* self = GST_IMAGE_REPROJECTION(aggregator);

  g_mutex_lock(&self->lock);
  if (!self->config_loaded) {
    GError* error = NULL;
    if (!gst_image_reprojection_load_config(self, &error)) {
      if (error) {
        GST_ERROR_OBJECT(self, "Failed to load config during aggregate: %s", error->message);
        g_clear_error(&error);
      }
      g_mutex_unlock(&self->lock);
      return GST_FLOW_ERROR;
    }
  }
  g_mutex_unlock(&self->lock);

  if (self->camera_count == 0) {
    GST_ERROR_OBJECT(self, "No cameras configured");
    return GST_FLOW_ERROR;
  }

  guint pad_count = g_list_length(GST_ELEMENT(self)->sinkpads);
  if (pad_count < self->camera_count && !self->warned_pad_count) {
    GST_WARNING_OBJECT(self, "Only %u sink pads connected but config expects %u cameras", pad_count, self->camera_count);
    self->warned_pad_count = TRUE;
  }

  GstBuffer** buffers = g_new0(GstBuffer*, self->camera_count);
  GstMapInfo* maps = g_new0(GstMapInfo, self->camera_count);
  GstImageReprojectionPad** pads = g_new0(GstImageReprojectionPad*, self->camera_count);
  GstBuffer* out_buffer = NULL;
  GstFlowReturn flow = GST_AGGREGATOR_FLOW_NEED_DATA;

  for (GList* l = GST_ELEMENT(self)->sinkpads; l; l = l->next) {
    GstImageReprojectionPad* ip = GST_IMAGE_REPROJECTION_PAD(l->data);
    if (ip->index >= self->camera_count) {
      GST_WARNING_OBJECT(self, "Ignoring sink pad %u beyond camera configuration", ip->index);
      continue;
    }
    pads[ip->index] = ip;
  }

  if (!pads[0]) goto need_more_data;

  if (self->sync_mode == GST_IMAGE_REPROJECTION_SYNC_LEAD_LATEST) {
    for (guint i = 1; i < self->camera_count; ++i) {
      if (!pads[i]) continue;
      GstBuffer* next = NULL;
      while ((next = gst_aggregator_pad_pop_buffer(GST_AGGREGATOR_PAD(pads[i]))) != NULL) {
        if (self->latest_buffers[i]) gst_buffer_unref(self->latest_buffers[i]);
        self->latest_buffers[i] = next;
      }
    }
    buffers[0] = gst_aggregator_pad_pop_buffer(GST_AGGREGATOR_PAD(pads[0]));
    if (!buffers[0]) {
      if (gst_aggregator_pad_is_eos(GST_AGGREGATOR_PAD(pads[0]))) flow = GST_FLOW_EOS;
      goto need_more_data;
    }
    for (guint i = 1; i < self->camera_count; ++i) {
      GstBuffer* candidate = self->latest_buffers[i];
      if (!candidate) continue;
      if (GST_BUFFER_PTS_IS_VALID(buffers[0]) && GST_BUFFER_PTS_IS_VALID(candidate)) {
        const GstClockTime lead_pts = GST_BUFFER_PTS(buffers[0]);
        const GstClockTime candidate_pts = GST_BUFFER_PTS(candidate);
        const GstClockTime gap = lead_pts >= candidate_pts ? lead_pts - candidate_pts : candidate_pts - lead_pts;
        if (gap > self->frame_time_tolerance) continue;
      }
      buffers[i] = gst_buffer_ref(candidate);
    }
  } else {
    buffers[0] = gst_aggregator_pad_peek_buffer(GST_AGGREGATOR_PAD(pads[0]));
    if (!buffers[0]) {
      if (gst_aggregator_pad_is_eos(GST_AGGREGATOR_PAD(pads[0]))) flow = GST_FLOW_EOS;
      goto need_more_data;
    }
    gboolean future_frame_seen = FALSE;
    guint ready_count = 1;
    for (guint i = 1; i < self->camera_count; ++i) {
      if (!pads[i]) continue;
      GstAggregatorPad* pad = GST_AGGREGATOR_PAD(pads[i]);
      GstBuffer* candidate = gst_aggregator_pad_peek_buffer(pad);
      while (candidate && GST_BUFFER_PTS_IS_VALID(buffers[0]) && GST_BUFFER_PTS_IS_VALID(candidate) &&
             GST_BUFFER_PTS(buffers[0]) > GST_BUFFER_PTS(candidate) &&
             GST_BUFFER_PTS(buffers[0]) - GST_BUFFER_PTS(candidate) > self->frame_time_tolerance) {
        gst_buffer_unref(candidate);
        GstBuffer* stale = gst_aggregator_pad_pop_buffer(pad);
        if (stale) gst_buffer_unref(stale);
        candidate = gst_aggregator_pad_peek_buffer(pad);
      }
      if (!candidate) continue;
      if (GST_BUFFER_PTS_IS_VALID(buffers[0]) && GST_BUFFER_PTS_IS_VALID(candidate)) {
        const GstClockTime lead_pts = GST_BUFFER_PTS(buffers[0]);
        const GstClockTime candidate_pts = GST_BUFFER_PTS(candidate);
        if (candidate_pts > lead_pts && candidate_pts - lead_pts > self->frame_time_tolerance) {
          future_frame_seen = TRUE;
          gst_buffer_unref(candidate);
          continue;
        }
      }
      buffers[i] = candidate;
      ++ready_count;
    }
    if (ready_count != self->camera_count && !timeout && !future_frame_seen) goto need_more_data;
    if (ready_count != self->camera_count && !self->wait_all_publish_partial) {
      GST_DEBUG_OBJECT(self, "Discarding incomplete frame with %u/%u cameras", ready_count, self->camera_count);
      for (guint i = 0; i < self->camera_count; ++i) {
        if (!buffers[i]) continue;
        GstBuffer* consumed = gst_aggregator_pad_pop_buffer(GST_AGGREGATOR_PAD(pads[i]));
        if (consumed) gst_buffer_unref(consumed);
      }
      goto need_more_data;
    }
    for (guint i = 0; i < self->camera_count; ++i) {
      if (!buffers[i]) continue;
      GstBuffer* consumed = gst_aggregator_pad_pop_buffer(GST_AGGREGATOR_PAD(pads[i]));
      if (consumed) gst_buffer_unref(consumed);
    }
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    if (!buffers[i]) continue;
    GstImageReprojectionPad* ip = pads[i];

    GstVideoInfo info = ip->info;
    if (info.width == 0 || info.height == 0) {
      GstCaps* caps = gst_pad_get_current_caps(GST_PAD(ip));
      if (caps) {
        if (!gst_video_info_from_caps(&info, caps)) {
          GST_WARNING_OBJECT(self, "Failed to parse caps for pad %u", ip->index);
        }
        gst_caps_unref(caps);
      }
    }
    if (!gst_buffer_map(buffers[i], &maps[i], GST_MAP_READ)) {
      GST_WARNING_OBJECT(self, "Failed to map buffer for pad %u", ip->index);
      flow = GST_FLOW_ERROR;
      goto need_more_data;
    }
    GstVideoMeta* meta = gst_buffer_get_video_meta(buffers[i]);
    const gint stride = meta ? meta->stride[0] : GST_VIDEO_INFO_COMP_STRIDE(&info, 0);
    const gsize offset = meta ? meta->offset[0] : info.offset[0];
    const guint width = meta ? meta->width : (guint)info.width;
    const guint height = meta ? meta->height : (guint)info.height;
    if (width == 0 || height == 0 || stride < (gint)(width * 3) || offset > maps[i].size ||
        (gsize)(height - 1) * stride + (gsize)width * 3 > maps[i].size - offset) {
      GST_ERROR_OBJECT(self, "Invalid BGR buffer layout on pad %u: %ux%u stride=%d offset=%zu size=%zu", i,
                       width, height, stride, offset, maps[i].size);
      flow = GST_FLOW_ERROR;
      goto need_more_data;
    }
    self->cameras[i].width = width;
    self->cameras[i].height = height;
    self->cameras[i].stride = stride;
    self->cameras[i].plane_offset = offset;
  }

  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR) {
    flow = gst_image_reprojection_process_planar(self, buffers, maps, &out_buffer);
  } else if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_EQUIRECT) {
    flow = gst_image_reprojection_process_equirect(self, buffers, maps, &out_buffer);
  } else {
    GST_ERROR_OBJECT(self, "Invalid active projection mode");
    flow = GST_FLOW_ERROR;
  }

  if (flow == GST_FLOW_OK && out_buffer) {
    if (!gst_image_reprojection_push_initial_events(self, aggregator)) {
      gst_buffer_unref(out_buffer);
      out_buffer = NULL;
      flow = GST_FLOW_ERROR;
      goto need_more_data;
    }

    GstClockTime pts = GST_CLOCK_TIME_NONE;
    GstClockTime dts = GST_CLOCK_TIME_NONE;
    GstClockTime duration = GST_CLOCK_TIME_NONE;

    for (guint i = 0; i < self->camera_count; ++i) {
      if (!buffers[i]) continue;
      if (GST_BUFFER_PTS_IS_VALID(buffers[i])) {
        pts = GST_BUFFER_PTS(buffers[i]);
        break;
      }
    }
    for (guint i = 0; i < self->camera_count; ++i) {
      if (!buffers[i]) continue;
      if (GST_BUFFER_DTS_IS_VALID(buffers[i])) {
        dts = GST_BUFFER_DTS(buffers[i]);
        break;
      }
    }
    for (guint i = 0; i < self->camera_count; ++i) {
      if (!buffers[i]) continue;
      if (GST_BUFFER_DURATION_IS_VALID(buffers[i])) {
        duration = GST_BUFFER_DURATION(buffers[i]);
        break;
      }
    }

    if (GST_CLOCK_TIME_IS_VALID(pts)) GST_BUFFER_PTS(out_buffer) = pts;
    if (GST_CLOCK_TIME_IS_VALID(dts)) GST_BUFFER_DTS(out_buffer) = dts;
    if (GST_CLOCK_TIME_IS_VALID(duration)) GST_BUFFER_DURATION(out_buffer) = duration;

    gst_aggregator_selected_samples(aggregator, pts, dts, duration, NULL);
    flow = gst_aggregator_finish_buffer(aggregator, out_buffer);
  } else {
    if (out_buffer) gst_buffer_unref(out_buffer);
  }

need_more_data:
  for (guint i = 0; i < self->camera_count; ++i) {
    if (maps[i].data) {
      gst_buffer_unmap(buffers[i], &maps[i]);
    }
    if (buffers[i]) {
      gst_buffer_unref(buffers[i]);
    }
  }
  g_free(maps);
  g_free(buffers);
  g_free(pads);

  return flow;
}

static void gst_image_reprojection_reset_planar(GstImageReprojectionPlanar* planar) {
  planar->enabled = FALSE;
  g_free(planar->frame_id);
  planar->frame_id = NULL;
  g_clear_pointer(&planar->x_norm, g_free);
  g_clear_pointer(&planar->y_norm, g_free);
  planar->pixel_count = 0;
  planar->width = 0;
  planar->height = 0;
  planar->fx = planar->fy = planar->cx = planar->cy = planar->depth = 0.0;
  planar->blend_factor = GST_IMAGE_REPROJECTION_DEFAULT_BLEND;
}

static void gst_image_reprojection_reset_equirect(GstImageReprojectionEquirect* eq) {
  eq->enabled = FALSE;
  g_free(eq->frame_id);
  eq->frame_id = NULL;
  g_clear_pointer(&eq->sin_lat, g_free);
  g_clear_pointer(&eq->cos_lat, g_free);
  g_clear_pointer(&eq->sin_lon, g_free);
  g_clear_pointer(&eq->cos_lon, g_free);
  eq->pixel_count = 0;
  eq->width = 0;
  eq->height = 0;
  eq->hfov_rad = eq->vfov_rad = eq->radius = 0.0;
  eq->blend_factor = GST_IMAGE_REPROJECTION_DEFAULT_BLEND;
}

static void gst_image_reprojection_clear_config(GstImageReprojection* self) {
  guint old_count = self->camera_count;

#ifdef HAVE_CUDA
  reprojection_cuda_destroy(self->gpu_context);
  reprojection_cuda_destroy(self->gpu_multi_context);
  self->gpu_context = NULL;
  self->gpu_multi_context = NULL;
  self->gpu_map = NULL;
  self->gpu_dominant_disabled = FALSE;
  self->gpu_multi_disabled = FALSE;
#endif

  if (self->latest_buffers) {
    for (guint i = 0; i < old_count; ++i) {
      if (self->latest_buffers[i]) gst_buffer_unref(self->latest_buffers[i]);
    }
    g_clear_pointer(&self->latest_buffers, g_free);
  }

  if (self->planar_maps) {
    for (guint i = 0; i < old_count; ++i) {
      g_free(self->planar_maps[i]);
    }
    g_free(self->planar_maps);
    self->planar_maps = NULL;
  }
  if (self->equirect_maps) {
    for (guint i = 0; i < old_count; ++i) {
      g_free(self->equirect_maps[i]);
    }
    g_free(self->equirect_maps);
    self->equirect_maps = NULL;
  }
  g_clear_pointer(&self->planar_dominant_map, g_free);
  g_clear_pointer(&self->equirect_dominant_map, g_free);
  if (self->planar_accumulators) {
    for (guint i = 0; i < old_count; ++i) {
      g_free(self->planar_accumulators[i]);
    }
    g_free(self->planar_accumulators);
    self->planar_accumulators = NULL;
  }
  if (self->planar_weights) {
    for (guint i = 0; i < old_count; ++i) {
      g_free(self->planar_weights[i]);
    }
    g_free(self->planar_weights);
    self->planar_weights = NULL;
  }
  if (self->equirect_accumulators) {
    for (guint i = 0; i < old_count; ++i) {
      g_free(self->equirect_accumulators[i]);
    }
    g_free(self->equirect_accumulators);
    self->equirect_accumulators = NULL;
  }
  if (self->equirect_weights) {
    for (guint i = 0; i < old_count; ++i) {
      g_free(self->equirect_weights[i]);
    }
    g_free(self->equirect_weights);
    self->equirect_weights = NULL;
  }

  if (self->cameras) {
    g_free(self->cameras);
    self->cameras = NULL;
  }
  self->camera_count = 0;
  self->sync_mode = GST_IMAGE_REPROJECTION_SYNC_WAIT_ALL;
  self->frame_timeout_sec = 1.0;
  self->frame_time_tolerance = 5 * GST_MSECOND;
  self->wait_all_publish_partial = TRUE;

  gst_image_reprojection_reset_planar(&self->planar);
  gst_image_reprojection_reset_equirect(&self->equirect);

  gst_caps_replace(&self->src_caps, NULL);
  self->initial_events_pushed = FALSE;
  gst_segment_init(&self->segment, GST_FORMAT_TIME);

  self->config_loaded = FALSE;
}

static gboolean gst_image_reprojection_parse_camera(JsonObject* cam_obj, GstImageReprojectionCamera* cam, GError** error) {
  JsonObject* intr_obj = json_object_get_object_member(cam_obj, "intrinsics");
  if (!intr_obj) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Camera intrinsics missing");
    return FALSE;
  }

  cam->fx = json_object_get_double_member(intr_obj, "fx");
  cam->fy = json_object_get_double_member(intr_obj, "fy");
  cam->cx = json_object_get_double_member(intr_obj, "cx");
  cam->cy = json_object_get_double_member(intr_obj, "cy");
  cam->width = (int)json_object_get_int_member(intr_obj, "width");
  cam->height = (int)json_object_get_int_member(intr_obj, "height");
  if (!isfinite(cam->fx) || !isfinite(cam->fy) || !isfinite(cam->cx) || !isfinite(cam->cy) ||
      cam->fx <= 0.0 || cam->fy <= 0.0 || cam->width <= 0 || cam->height <= 0 || cam->width > (G_MAXINT - 3) / 3) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Invalid camera intrinsics");
    return FALSE;
  }
  cam->map_width = cam->width;
  cam->map_height = cam->height;
  cam->stride = GST_ROUND_UP_4(cam->width * 3);

  JsonArray* planar_array = json_object_get_array_member(cam_obj, "planar_transform");
  cam->has_planar_matrix = planar_array && json_array_get_length(planar_array) >= 16;
  if (cam->has_planar_matrix) {
    for (guint i = 0; i < 16; ++i) {
      cam->planar_matrix[i] = json_array_get_double_element(planar_array, i);
    }
  }

  JsonArray* eq_array = json_object_get_array_member(cam_obj, "equirectangular_transform");
  cam->has_equirect_matrix = eq_array && json_array_get_length(eq_array) >= 16;
  if (cam->has_equirect_matrix) {
    for (guint i = 0; i < 16; ++i) {
      cam->equirect_matrix[i] = json_array_get_double_element(eq_array, i);
    }
  }

  return TRUE;
}

static gboolean gst_image_reprojection_parse_planar(JsonObject* root, GstImageReprojectionPlanar* planar, GError** error) {
  JsonObject* planar_obj = json_object_get_object_member(root, "planar");
  if (!planar_obj) {
    planar->enabled = FALSE;
    return TRUE;
  }

  planar->enabled = json_object_get_boolean_member_or(planar_obj, "enabled", FALSE);
  if (!planar->enabled) {
    return TRUE;
  }

  if (!json_object_has_member(planar_obj, "width") || !json_object_has_member(planar_obj, "height") ||
      !json_object_has_member(planar_obj, "fx") || !json_object_has_member(planar_obj, "fy") ||
      !json_object_has_member(planar_obj, "cx") || !json_object_has_member(planar_obj, "cy") ||
      !json_object_has_member(planar_obj, "depth") || !json_object_has_member(planar_obj, "blend_factor")) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Planar configuration missing required fields");
    return FALSE;
  }

  planar->width = (gint)json_object_get_int_member(planar_obj, "width");
  planar->height = (gint)json_object_get_int_member(planar_obj, "height");
  planar->fx = json_object_get_double_member(planar_obj, "fx");
  planar->fy = json_object_get_double_member(planar_obj, "fy");
  planar->cx = json_object_get_double_member(planar_obj, "cx");
  planar->cy = json_object_get_double_member(planar_obj, "cy");
  planar->depth = json_object_get_double_member(planar_obj, "depth");
  planar->blend_factor = json_object_get_double_member(planar_obj, "blend_factor");

  const gchar* frame_id = json_object_get_string_member_or(planar_obj, "frame_id", NULL);
  planar->frame_id = frame_id ? g_strdup(frame_id) : NULL;

  if (planar->width <= 0 || planar->height <= 0 || planar->fx <= 0.0 || planar->fy <= 0.0 || planar->depth <= 0.0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Invalid planar projection parameters");
    return FALSE;
  }

  planar->blend_factor = CLAMP(planar->blend_factor, 0.0, 1.0);
  planar->pixel_count = (gsize)planar->width * (gsize)planar->height;

  double inv_fx = planar->fx > 0.0 ? 1.0 / planar->fx : 0.0;
  double inv_fy = planar->fy > 0.0 ? 1.0 / planar->fy : 0.0;

  planar->x_norm = g_new0(double, planar->width);
  planar->y_norm = g_new0(double, planar->height);
  for (gint u = 0; u < planar->width; ++u) {
    planar->x_norm[u] = ((double)u - planar->cx) * inv_fx;
  }
  for (gint v = 0; v < planar->height; ++v) {
    planar->y_norm[v] = ((double)v - planar->cy) * inv_fy;
  }

  return TRUE;
}

static gboolean gst_image_reprojection_parse_equirect(JsonObject* root, GstImageReprojectionEquirect* eq, GError** error) {
  JsonObject* eq_obj = json_object_get_object_member(root, "equirectangular");
  if (!eq_obj) {
    eq->enabled = FALSE;
    return TRUE;
  }

  eq->enabled = json_object_get_boolean_member_or(eq_obj, "enabled", FALSE);
  if (!eq->enabled) {
    return TRUE;
  }

  if (!json_object_has_member(eq_obj, "width") || !json_object_has_member(eq_obj, "height") ||
      !json_object_has_member(eq_obj, "hfov_rad") || !json_object_has_member(eq_obj, "vfov_rad") ||
      !json_object_has_member(eq_obj, "radius") || !json_object_has_member(eq_obj, "blend_factor")) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Equirectangular configuration missing required fields");
    return FALSE;
  }

  eq->width = (gint)json_object_get_int_member(eq_obj, "width");
  eq->height = (gint)json_object_get_int_member(eq_obj, "height");
  eq->hfov_rad = json_object_get_double_member(eq_obj, "hfov_rad");
  eq->vfov_rad = json_object_get_double_member(eq_obj, "vfov_rad");
  eq->radius = json_object_get_double_member(eq_obj, "radius");
  eq->blend_factor = json_object_get_double_member(eq_obj, "blend_factor");

  const gchar* frame_id = json_object_get_string_member_or(eq_obj, "frame_id", NULL);
  eq->frame_id = frame_id ? g_strdup(frame_id) : NULL;

  GST_INFO("Equirect config: enabled=%d width=%d height=%d hfov=%.3f vfov=%.3f", eq->enabled, eq->width, eq->height, eq->hfov_rad,
           eq->vfov_rad);

  if (eq->width <= 0 || eq->height <= 0 || eq->radius <= 0.0 || eq->hfov_rad <= 0.0 || eq->vfov_rad <= 0.0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Invalid equirectangular projection parameters");
    return FALSE;
  }

  eq->blend_factor = CLAMP(eq->blend_factor, 0.0, 1.0);
  eq->pixel_count = (gsize)eq->width * (gsize)eq->height;

  eq->sin_lat = g_new0(double, eq->height);
  eq->cos_lat = g_new0(double, eq->height);
  eq->sin_lon = g_new0(double, eq->width);
  eq->cos_lon = g_new0(double, eq->width);

  for (gint v = 0; v < eq->height; ++v) {
    double v_norm = ((double)v + 0.5) / (double)eq->height;
    double lat = (0.5 - v_norm) * eq->vfov_rad;
    eq->sin_lat[v] = sin(lat);
    eq->cos_lat[v] = cos(lat);
  }
  for (gint u = 0; u < eq->width; ++u) {
    double u_norm = ((double)u + 0.5) / (double)eq->width;
    double lon = (u_norm - 0.5) * eq->hfov_rad;
    eq->sin_lon[u] = sin(lon);
    eq->cos_lon[u] = cos(lon);
  }

  return TRUE;
}

static gboolean gst_image_reprojection_parse_sync(JsonObject* root, GstImageReprojection* self, GError** error) {
  if (!json_object_has_member(root, "sync")) return TRUE;
  JsonObject* sync = json_object_get_object_member(root, "sync");
  if (!sync) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Sync configuration must be an object");
    return FALSE;
  }
  const gchar* mode = json_object_get_string_member_or(sync, "mode", "wait_all");
  if (g_strcmp0(mode, "wait_all") == 0 || g_strcmp0(mode, "all") == 0 || g_strcmp0(mode, "sync") == 0) {
    self->sync_mode = GST_IMAGE_REPROJECTION_SYNC_WAIT_ALL;
  } else if (g_strcmp0(mode, "lead_latest") == 0 || g_strcmp0(mode, "lead") == 0 ||
             g_strcmp0(mode, "lead_image") == 0) {
    self->sync_mode = GST_IMAGE_REPROJECTION_SYNC_LEAD_LATEST;
  } else {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Unsupported sync mode: %s", mode);
    return FALSE;
  }
  const double timeout = json_object_has_member(sync, "frame_timeout") ?
                             json_object_get_double_member(sync, "frame_timeout") : 1.0;
  const double tolerance = json_object_has_member(sync, "frame_time_tolerance") ?
                               json_object_get_double_member(sync, "frame_time_tolerance") : 0.005;
  if (!isfinite(timeout) || timeout < 0.0 || timeout > 3600.0 || !isfinite(tolerance) || tolerance < 0.0 ||
      tolerance > 3600.0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Invalid sync timeout or tolerance");
    return FALSE;
  }
  self->frame_timeout_sec = timeout;
  self->frame_time_tolerance = (GstClockTime)(tolerance * GST_SECOND);
  self->wait_all_publish_partial =
      json_object_get_boolean_member_or(sync, "wait_all_publish_partial", TRUE);
  return TRUE;
}

static gboolean gst_image_reprojection_load_config(GstImageReprojection* self, GError** error) {
  if (self->config_loaded) {
    return TRUE;
  }

  if (!self->config_path || !g_file_test(self->config_path, G_FILE_TEST_EXISTS)) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_NOT_FOUND, "Configuration path not set or file missing");
    return FALSE;
  }

  gst_image_reprojection_clear_config(self);

  JsonParser* parser = json_parser_new();
  if (!json_parser_load_from_file(parser, self->config_path, error)) {
    g_object_unref(parser);
    return FALSE;
  }

  JsonNode* root_node = json_parser_get_root(parser);
  if (!JSON_NODE_HOLDS_OBJECT(root_node)) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Configuration root is not an object");
    g_object_unref(parser);
    return FALSE;
  }

  JsonObject* root_obj = json_node_get_object(root_node);
  JsonArray* cam_array = json_object_get_array_member(root_obj, "cameras");
  if (!cam_array) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Configuration missing cameras array");
    g_object_unref(parser);
    return FALSE;
  }

  self->camera_count = json_array_get_length(cam_array);
  if (self->camera_count == 0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Configuration has no cameras");
    g_object_unref(parser);
    return FALSE;
  }

  self->cameras = g_new0(GstImageReprojectionCamera, self->camera_count);
  self->latest_buffers = g_new0(GstBuffer*, self->camera_count);
  for (guint i = 0; i < self->camera_count; ++i) {
    JsonObject* cam_obj = json_array_get_object_element(cam_array, i);
    if (!gst_image_reprojection_parse_camera(cam_obj, &self->cameras[i], error)) {
      g_object_unref(parser);
      return FALSE;
    }
  }

  if (!gst_image_reprojection_parse_planar(root_obj, &self->planar, error)) {
    g_object_unref(parser);
    return FALSE;
  }
  if (!gst_image_reprojection_parse_equirect(root_obj, &self->equirect, error)) {
    g_object_unref(parser);
    return FALSE;
  }
  if (!gst_image_reprojection_parse_sync(root_obj, self, error)) {
    g_object_unref(parser);
    return FALSE;
  }

  if (!self->planar.enabled && !self->equirect.enabled) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Configuration has no enabled projections");
    g_object_unref(parser);
    return FALSE;
  }

  self->active_mode = self->mode_property;
  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_AUTO) {
    if (self->planar.enabled) {
      self->active_mode = GST_IMAGE_REPROJECTION_MODE_PLANAR;
    } else {
      self->active_mode = GST_IMAGE_REPROJECTION_MODE_EQUIRECT;
    }
  }

  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR && !self->planar.enabled) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Planar projection requested but disabled in config");
    g_object_unref(parser);
    return FALSE;
  }
  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_EQUIRECT && !self->equirect.enabled) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Equirectangular projection requested but disabled in config");
    g_object_unref(parser);
    return FALSE;
  }

  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR) {
    if (!gst_image_reprojection_update_planar_maps(self, error)) {
      g_object_unref(parser);
      return FALSE;
    }
  } else {
    if (!gst_image_reprojection_update_equirect_maps(self, error)) {
      g_object_unref(parser);
      return FALSE;
    }
  }

  if (!gst_image_reprojection_ensure_output_caps(self, error)) {
    g_object_unref(parser);
    return FALSE;
  }

  self->config_loaded = TRUE;
  const GstClockTime latency = self->sync_mode == GST_IMAGE_REPROJECTION_SYNC_WAIT_ALL &&
                                       self->frame_timeout_sec > 0.0 ?
                                   (GstClockTime)(self->frame_timeout_sec * GST_SECOND) : GST_MSECOND;
  gst_aggregator_set_latency(GST_AGGREGATOR(self), latency, latency);
  self->warned_pad_count = FALSE;
  g_object_unref(parser);
  return TRUE;
}

static void multiply_point(const double matrix[16], double x, double y, double z, double* out_x, double* out_y, double* out_z) {
  *out_x = matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3];
  *out_y = matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7];
  *out_z = matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11];
}

static gboolean gst_image_reprojection_update_planar_maps(GstImageReprojection* self, GError** error) {
  if (!self->planar.enabled) {
    return TRUE;
  }

  const gsize pixel_count = self->planar.pixel_count;
  if (pixel_count == 0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Planar projection has zero pixels");
    return FALSE;
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    if (!self->cameras[i].has_planar_matrix) {
      g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Camera %u missing planar transform", i);
      return FALSE;
    }
  }

  self->planar_maps = g_new0(GstImageReprojectionPixelMap*, self->camera_count);
  if (self->planar.blend_factor != 0.0) {
    self->planar_accumulators = g_new0(float*, self->camera_count);
    self->planar_weights = g_new0(float*, self->camera_count);
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    self->planar_maps[i] = g_new0(GstImageReprojectionPixelMap, pixel_count);
    if (self->planar.blend_factor != 0.0) {
      self->planar_accumulators[i] = g_new0(float, pixel_count * 3);
      self->planar_weights[i] = g_new0(float, pixel_count);
    }
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    GstImageReprojectionPixelMap* mapping = self->planar_maps[i];
    const double* matrix = self->cameras[i].planar_matrix;

    for (gint v = 0; v < self->planar.height; ++v) {
      double y = self->planar.y_norm[v] * self->planar.depth;
      for (gint u = 0; u < self->planar.width; ++u) {
        double x = self->planar.x_norm[u] * self->planar.depth;
        double z = self->planar.depth;

        double cam_x, cam_y, cam_z;
        multiply_point(matrix, x, y, z, &cam_x, &cam_y, &cam_z);

        gsize idx = (gsize)v * self->planar.width + (gsize)u;
        if (cam_z <= kEpsilon) {
          mapping[idx].u = NAN;
          mapping[idx].v = NAN;
          continue;
        }

        double inv_z = 1.0 / cam_z;
        double u_in = self->cameras[i].fx * (cam_x * inv_z) + self->cameras[i].cx;
        double v_in = self->cameras[i].fy * (cam_y * inv_z) + self->cameras[i].cy;

        mapping[idx].u = (float)u_in;
        mapping[idx].v = (float)v_in;
      }
    }
  }

  if (self->planar.blend_factor == 0.0) {
    self->planar_dominant_map = gst_image_reprojection_build_dominant_map(self, self->planar_maps, pixel_count);
  }

  return TRUE;
}

static gboolean gst_image_reprojection_update_equirect_maps(GstImageReprojection* self, GError** error) {
  if (!self->equirect.enabled) {
    return TRUE;
  }

  const gsize pixel_count = self->equirect.pixel_count;
  if (pixel_count == 0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Equirectangular projection has zero pixels");
    return FALSE;
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    if (!self->cameras[i].has_equirect_matrix) {
      g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Camera %u missing equirectangular transform", i);
      return FALSE;
    }
  }

  self->equirect_maps = g_new0(GstImageReprojectionPixelMap*, self->camera_count);
  if (self->equirect.blend_factor != 0.0) {
    self->equirect_accumulators = g_new0(float*, self->camera_count);
    self->equirect_weights = g_new0(float*, self->camera_count);
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    self->equirect_maps[i] = g_new0(GstImageReprojectionPixelMap, pixel_count);
    if (self->equirect.blend_factor != 0.0) {
      self->equirect_accumulators[i] = g_new0(float, pixel_count * 3);
      self->equirect_weights[i] = g_new0(float, pixel_count);
    }
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    GstImageReprojectionPixelMap* mapping = self->equirect_maps[i];
    const double* matrix = self->cameras[i].equirect_matrix;

    for (gint v = 0; v < self->equirect.height; ++v) {
      double sin_lat = self->equirect.sin_lat[v];
      double cos_lat = self->equirect.cos_lat[v];
      for (gint u = 0; u < self->equirect.width; ++u) {
        double sin_lon = self->equirect.sin_lon[u];
        double cos_lon = self->equirect.cos_lon[u];

        double dir_x = cos_lat * sin_lon;
        double dir_y = -sin_lat;
        double dir_z = cos_lat * cos_lon;

        double x = dir_x * self->equirect.radius;
        double y = dir_y * self->equirect.radius;
        double z = dir_z * self->equirect.radius;

        double cam_x, cam_y, cam_z;
        multiply_point(matrix, x, y, z, &cam_x, &cam_y, &cam_z);

        gsize idx = (gsize)v * self->equirect.width + (gsize)u;
        if (cam_z <= kEpsilon) {
          mapping[idx].u = NAN;
          mapping[idx].v = NAN;
          continue;
        }

        double inv_z = 1.0 / cam_z;
        double u_in = self->cameras[i].fx * (cam_x * inv_z) + self->cameras[i].cx;
        double v_in = self->cameras[i].fy * (cam_y * inv_z) + self->cameras[i].cy;

        mapping[idx].u = (float)u_in;
        mapping[idx].v = (float)v_in;
      }
    }
  }

  if (self->equirect.blend_factor == 0.0) {
    self->equirect_dominant_map = gst_image_reprojection_build_dominant_map(self, self->equirect_maps, pixel_count);
  }

  return TRUE;
}

static gboolean gst_image_reprojection_ensure_output_caps(GstImageReprojection* self, GError** error) {
  GstVideoInfo info;
  gst_video_info_init(&info);

  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR) {
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_BGR, self->planar.width, self->planar.height);
  } else if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_EQUIRECT) {
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_BGR, self->equirect.width, self->equirect.height);
  } else {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Unsupported active projection mode");
    return FALSE;
  }

  GstCaps* caps = gst_video_info_to_caps(&info);
  if (!caps) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED, "Failed to create src caps");
    return FALSE;
  }

  gst_caps_replace(&self->src_caps, caps);
  self->output_info = info;
  self->initial_events_pushed = FALSE;
  gst_caps_unref(caps);
  return TRUE;
}

static gboolean gst_image_reprojection_push_initial_events(GstImageReprojection* self, GstAggregator* aggregator) {
  if (self->initial_events_pushed) {
    return TRUE;
  }

  GstPad* srcpad = GST_AGGREGATOR_SRC_PAD(aggregator);

  gchar* stream_id = g_strdup_printf("imagereprojection-%p-%" G_GUINT64_FORMAT, (void*)self, (guint64)g_get_monotonic_time());

  GstEvent* stream_event = gst_event_new_stream_start(stream_id);
  g_free(stream_id);
  if (!gst_pad_push_event(srcpad, stream_event)) {
    GST_WARNING_OBJECT(self, "Failed to push stream-start event");
    return FALSE;
  }

  GstCaps* caps = self->src_caps ? gst_caps_ref(self->src_caps) : NULL;
  if (!caps) {
    GST_WARNING_OBJECT(self, "No caps available for src pad");
    return FALSE;
  }

  gst_aggregator_set_src_caps(GST_AGGREGATOR(self), caps);
  gst_caps_unref(caps);

  gst_segment_init(&self->segment, GST_FORMAT_TIME);
  GstEvent* segment_event = gst_event_new_segment(&self->segment);
  if (!gst_pad_push_event(srcpad, segment_event)) {
    GST_WARNING_OBJECT(self, "Failed to push segment event");
    return FALSE;
  }

  self->initial_events_pushed = TRUE;
  return TRUE;
}

static void gst_image_reprojection_reset_scratch_planar(GstImageReprojection* self) {
  if (!self->planar_accumulators || !self->planar_weights) return;
  const gsize pixel_count = self->planar.pixel_count;
  const gsize acc_size = pixel_count * 3;
  for (guint i = 0; i < self->camera_count; ++i) {
    if (self->planar_accumulators[i]) memset(self->planar_accumulators[i], 0, sizeof(float) * acc_size);
    if (self->planar_weights[i]) memset(self->planar_weights[i], 0, sizeof(float) * pixel_count);
  }
}

static void gst_image_reprojection_reset_scratch_equirect(GstImageReprojection* self) {
  if (!self->equirect_accumulators || !self->equirect_weights) return;
  const gsize pixel_count = self->equirect.pixel_count;
  const gsize acc_size = pixel_count * 3;
  for (guint i = 0; i < self->camera_count; ++i) {
    if (self->equirect_accumulators[i]) memset(self->equirect_accumulators[i], 0, sizeof(float) * acc_size);
    if (self->equirect_weights[i]) memset(self->equirect_weights[i], 0, sizeof(float) * pixel_count);
  }
}

static gboolean gst_image_reprojection_plugin_init(GstPlugin* plugin) {
  return gst_element_register(plugin, "imagereprojection", GST_RANK_NONE, GST_TYPE_IMAGE_REPROJECTION);
}

static GType gst_image_reprojection_mode_get_type(void) {
  static GType mode_type = 0;
  if (mode_type == 0) {
    static const GEnumValue values[] = {{GST_IMAGE_REPROJECTION_MODE_AUTO, "Auto", "auto"},
                                        {GST_IMAGE_REPROJECTION_MODE_PLANAR, "Planar", "planar"},
                                        {GST_IMAGE_REPROJECTION_MODE_EQUIRECT, "Equirectangular", "equirectangular"},
                                        {0, NULL, NULL}};
    mode_type = g_enum_register_static("GstImageReprojectionMode", values);
  }
  return mode_type;
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR,
                  GST_VERSION_MINOR,
                  imagereprojection,
                  "Image Reprojection",
                  gst_image_reprojection_plugin_init,
                  "1.2.0",
                  "Apache-2.0",
                  "gst_image_reprojection",
                  "https://github.com/ika-rwth-aachen/image_reprojection")

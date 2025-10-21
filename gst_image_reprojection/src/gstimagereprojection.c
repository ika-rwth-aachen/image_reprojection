#include <gst/gst.h>
#include <gst/base/gstaggregator.h>
#include <gst/video/video.h>
#include <json-glib/json-glib.h>
#include <math.h>
#include <string.h>

#ifndef PACKAGE
#define PACKAGE "gst_image_reprojection"
#endif

#define GST_TYPE_IMAGE_REPROJECTION (gst_image_reprojection_get_type())
#define GST_IMAGE_REPROJECTION(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_IMAGE_REPROJECTION, GstImageReprojection))
#define GST_IMAGE_REPROJECTION_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_IMAGE_REPROJECTION, GstImageReprojectionClass))
#define GST_IS_IMAGE_REPROJECTION(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_IMAGE_REPROJECTION))
#define GST_IS_IMAGE_REPROJECTION_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_IMAGE_REPROJECTION))

#define GST_TYPE_IMAGE_REPROJECTION_PAD (gst_image_reprojection_pad_get_type())
#define GST_IMAGE_REPROJECTION_PAD(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_IMAGE_REPROJECTION_PAD, GstImageReprojectionPad))

#define GST_IMAGE_REPROJECTION_DEFAULT_BLEND 1.0

typedef enum {
  GST_IMAGE_REPROJECTION_MODE_AUTO = 0,
  GST_IMAGE_REPROJECTION_MODE_PLANAR,
  GST_IMAGE_REPROJECTION_MODE_EQUIRECT
} GstImageReprojectionMode;

static GType gst_image_reprojection_mode_get_type(void);
#define GST_TYPE_IMAGE_REPROJECTION_MODE (gst_image_reprojection_mode_get_type())

static gboolean json_object_get_boolean_member_or(JsonObject *object, const gchar *member, gboolean fallback) {
  if (!json_object_has_member(object, member)) {
    return fallback;
  }
  return json_object_get_boolean_member(object, member);
}

static const gchar *json_object_get_string_member_or(JsonObject *object, const gchar *member, const gchar *fallback) {
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
  double planar_matrix[16];
  double equirect_matrix[16];
  gboolean has_planar_matrix;
  gboolean has_equirect_matrix;
} GstImageReprojectionCamera;

typedef struct {
  gboolean enabled;
  gchar *frame_id;
  gint width;
  gint height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth;
  double blend_factor;
  gsize pixel_count;
  double *x_norm; /* width */
  double *y_norm; /* height */
} GstImageReprojectionPlanar;

typedef struct {
  gboolean enabled;
  gchar *frame_id;
  gint width;
  gint height;
  double hfov_rad;
  double vfov_rad;
  double radius;
  double blend_factor;
  gsize pixel_count;
  double *sin_lat; /* height */
  double *cos_lat; /* height */
  double *sin_lon; /* width */
  double *cos_lon; /* width */
} GstImageReprojectionEquirect;

typedef struct {
  float u;
  float v;
} GstImageReprojectionPixelMap;

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

  gchar *config_path;
  GstImageReprojectionMode mode_property;
  GstImageReprojectionMode active_mode;
  gboolean config_loaded;
  gboolean warned_pad_count;
  gboolean initial_events_pushed;

  GMutex lock;

  guint camera_count;
  GstImageReprojectionCamera *cameras;

  GstImageReprojectionPlanar planar;
  GstImageReprojectionEquirect equirect;

  GstImageReprojectionPixelMap **planar_maps; /* [camera][pixel] */
  GstImageReprojectionPixelMap **equirect_maps;

  float **planar_accumulators;
  float **planar_weights;
  float **equirect_accumulators;
  float **equirect_weights;

  GstCaps *src_caps;
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
};

static GstStaticPadTemplate gst_image_reprojection_sink_template =
    GST_STATIC_PAD_TEMPLATE("sink_%u",
                            GST_PAD_SINK,
                            GST_PAD_REQUEST,
                            GST_STATIC_CAPS("video/x-raw, format=(string)BGR"));

static GstStaticPadTemplate gst_image_reprojection_src_template =
    GST_STATIC_PAD_TEMPLATE("src",
                            GST_PAD_SRC,
                            GST_PAD_ALWAYS,
                            GST_STATIC_CAPS("video/x-raw, format=(string)BGR"));

static const double kEpsilon = 1e-9;

/* Forward declarations */
static void gst_image_reprojection_set_property(GObject *object, guint prop_id,
                                                const GValue *value, GParamSpec *pspec);
static void gst_image_reprojection_get_property(GObject *object, guint prop_id,
                                                GValue *value, GParamSpec *pspec);
static void gst_image_reprojection_dispose(GObject *object);
static gboolean gst_image_reprojection_start(GstAggregator *aggregator);
static gboolean gst_image_reprojection_stop(GstAggregator *aggregator);
static GstFlowReturn gst_image_reprojection_aggregate(GstAggregator *aggregator, gboolean timeout);
static GstAggregatorPad *gst_image_reprojection_create_new_pad(GstAggregator *aggregator,
                                                               GstPadTemplate *templ,
                                                               const gchar *name,
                                                               const GstCaps *caps);
static gboolean gst_image_reprojection_sink_event(GstAggregator *aggregator,
                                                  GstAggregatorPad *pad,
                                                  GstEvent *event);

static gboolean gst_image_reprojection_load_config(GstImageReprojection *self, GError **error);
static void gst_image_reprojection_clear_config(GstImageReprojection *self);
static gboolean gst_image_reprojection_update_planar_maps(GstImageReprojection *self, GError **error);
static gboolean gst_image_reprojection_update_equirect_maps(GstImageReprojection *self, GError **error);

static gboolean gst_image_reprojection_ensure_output_caps(GstImageReprojection *self, GError **error);
static gboolean gst_image_reprojection_push_initial_events(GstImageReprojection *self, GstAggregator *aggregator);

static void gst_image_reprojection_reset_scratch_planar(GstImageReprojection *self);
static void gst_image_reprojection_reset_scratch_equirect(GstImageReprojection *self);

static void gst_image_reprojection_class_init(GstImageReprojectionClass *klass) {
  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  GstAggregatorClass *aggregator_class = GST_AGGREGATOR_CLASS(klass);

  gobject_class->set_property = gst_image_reprojection_set_property;
  gobject_class->get_property = gst_image_reprojection_get_property;
  gobject_class->dispose = gst_image_reprojection_dispose;

  g_object_class_install_property(
      gobject_class,
      PROP_CONFIG_PATH,
      g_param_spec_string("config-path",
                          "Configuration Path",
                          "Path to JSON configuration exported by image_reprojection node",
                          NULL,
                          G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY));

  g_object_class_install_property(
      gobject_class,
      PROP_PROJECTION_MODE,
      g_param_spec_enum("projection-mode",
                        "Projection Mode",
                        "Projection to generate (auto/planar/equirectangular). In auto mode, planar is preferred if enabled",
                        GST_TYPE_IMAGE_REPROJECTION_MODE,
                        GST_IMAGE_REPROJECTION_MODE_AUTO,
                        G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY));

  gst_element_class_add_pad_template(element_class,
                                     gst_static_pad_template_get(&gst_image_reprojection_sink_template));
  gst_element_class_add_pad_template(element_class,
                                     gst_static_pad_template_get(&gst_image_reprojection_src_template));

  gst_element_class_set_static_metadata(element_class,
                                        "Image Reprojection",
                                        "Filter/Effect/Video",
                                        "Reprojects multiple input video streams using various projection methods",
                                        "Lennart Reiher <lennart.reiher@ika.rwth-aachen.de>");

  aggregator_class->start = gst_image_reprojection_start;
  aggregator_class->stop = gst_image_reprojection_stop;
  aggregator_class->aggregate = gst_image_reprojection_aggregate;
  aggregator_class->create_new_pad = gst_image_reprojection_create_new_pad;
  aggregator_class->sink_event = gst_image_reprojection_sink_event;
}

static void gst_image_reprojection_pad_class_init(GstImageReprojectionPadClass *klass) {
  (void)klass;
}

static void gst_image_reprojection_pad_init(GstImageReprojectionPad *pad) {
  pad->index = G_MAXUINT;
  gst_video_info_init(&pad->info);
}

static void gst_image_reprojection_init(GstImageReprojection *self) {
  self->config_path = NULL;
  self->mode_property = GST_IMAGE_REPROJECTION_MODE_AUTO;
  self->active_mode = GST_IMAGE_REPROJECTION_MODE_AUTO;
  self->config_loaded = FALSE;
  self->warned_pad_count = FALSE;
  self->initial_events_pushed = FALSE;
  g_mutex_init(&self->lock);

  self->camera_count = 0;
  self->cameras = NULL;

  memset(&self->planar, 0, sizeof(self->planar));
  memset(&self->equirect, 0, sizeof(self->equirect));

  self->planar_maps = NULL;
  self->equirect_maps = NULL;

  self->planar_accumulators = NULL;
  self->planar_weights = NULL;
  self->equirect_accumulators = NULL;
  self->equirect_weights = NULL;

  self->src_caps = NULL;
  gst_segment_init(&self->segment, GST_FORMAT_TIME);

  self->next_pad_index = 0;
}

static void gst_image_reprojection_dispose(GObject *object) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(object);

  gst_image_reprojection_clear_config(self);
  g_clear_pointer(&self->config_path, g_free);
  g_mutex_clear(&self->lock);

  G_OBJECT_CLASS(gst_image_reprojection_parent_class)->dispose(object);
}

static void gst_image_reprojection_set_property(GObject *object, guint prop_id,
                                                const GValue *value, GParamSpec *pspec) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(object);

  switch (prop_id) {
    case PROP_CONFIG_PATH: {
      const gchar *path = g_value_get_string(value);
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
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void gst_image_reprojection_get_property(GObject *object, guint prop_id,
                                                GValue *value, GParamSpec *pspec) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(object);

  switch (prop_id) {
    case PROP_CONFIG_PATH:
      g_value_set_string(value, self->config_path);
      break;
    case PROP_PROJECTION_MODE:
      g_value_set_enum(value, self->mode_property);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static GstAggregatorPad *gst_image_reprojection_create_new_pad(GstAggregator *aggregator,
                                                               GstPadTemplate *templ,
                                                               const gchar *name,
                                                               const GstCaps *caps) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(aggregator);
  GstImageReprojectionPad *pad;

  pad = g_object_new(GST_TYPE_IMAGE_REPROJECTION_PAD,
                     "name", name ? name : "sink_%u",
                     "direction", GST_PAD_SINK,
                     "template", templ,
                     NULL);
  pad->index = self->next_pad_index++;

  GST_DEBUG_OBJECT(self, "Created new sink pad %u", pad->index);

  if (caps) {
    gst_video_info_from_caps(&pad->info, caps);
  }

  return GST_AGGREGATOR_PAD(pad);
}

static gboolean gst_image_reprojection_sink_event(GstAggregator *aggregator,
                                                  GstAggregatorPad *pad,
                                                  GstEvent *event) {
  GstImageReprojectionPad *ip = GST_IMAGE_REPROJECTION_PAD(pad);

  if (GST_EVENT_TYPE(event) == GST_EVENT_CAPS) {
    GstCaps *caps = NULL;
    gst_event_parse_caps(event, &caps);
    if (caps) {
      gst_video_info_from_caps(&ip->info, caps);
    }
  }

  return GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->sink_event(aggregator, pad, event);
}

static gboolean gst_image_reprojection_start(GstAggregator *aggregator) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(aggregator);
  GError *error = NULL;

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

  if (GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->start) {
    return GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->start(aggregator);
  }

  return TRUE;
}

static gboolean gst_image_reprojection_stop(GstAggregator *aggregator) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(aggregator);

  g_mutex_lock(&self->lock);
  self->config_loaded = FALSE;
  self->initial_events_pushed = FALSE;
  g_mutex_unlock(&self->lock);

  if (GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->stop) {
    return GST_AGGREGATOR_CLASS(gst_image_reprojection_parent_class)->stop(aggregator);
  }
  return TRUE;
}

static inline gboolean isfinitef(float v) {
  return isfinite(v);
}

static inline float clampf(float v, float min_v, float max_v) {
  if (v < min_v) return min_v;
  if (v > max_v) return max_v;
  return v;
}

static inline void bilinear_sample(const guint8 *data, guint width, guint height, float u, float v, float out[3]) {
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

  const guint stride = width * 3;
  const guint8 *row0 = data + y0 * stride;
  const guint8 *row1 = data + y1 * stride;

  const guint8 *p00 = row0 + x0 * 3;
  const guint8 *p10 = row0 + x1 * 3;
  const guint8 *p01 = row1 + x0 * 3;
  const guint8 *p11 = row1 + x1 * 3;

  for (guint c = 0; c < 3; ++c) {
    float val0 = (1.0f - dx) * (float)p00[c] + dx * (float)p10[c];
    float val1 = (1.0f - dx) * (float)p01[c] + dx * (float)p11[c];
    out[c] = (1.0f - dy) * val0 + dy * val1;
  }
}

static GstFlowReturn gst_image_reprojection_process_planar(GstImageReprojection *self,
                                                           GstBuffer **buffers,
                                                           GstMapInfo *maps,
                                                           GstBuffer **out_buffer_ptr) {
  const guint camera_count = self->camera_count;
  const gsize pixel_count = self->planar.pixel_count;
  if (pixel_count == 0) {
    return GST_FLOW_ERROR;
  }

  gst_image_reprojection_reset_scratch_planar(self);

  for (guint i = 0; i < camera_count; ++i) {
    if (!buffers[i]) continue;
    const guint8 *data = maps[i].data;
    if (!data) continue;

    const GstImageReprojectionPixelMap *mapping = self->planar_maps[i];
    if (!mapping) continue;

    float *acc = self->planar_accumulators[i];
    float *weights = self->planar_weights[i];
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
      bilinear_sample(data, width_in, height_in, u, v, colour);

      gsize base = p * 3;
      acc[base + 0] += colour[0];
      acc[base + 1] += colour[1];
      acc[base + 2] += colour[2];
      weights[p] += 1.0f;
    }
  }

  GstBuffer *out = gst_buffer_new_allocate(NULL, self->planar.width * self->planar.height * 3, NULL);
  if (!out) {
    return GST_FLOW_ERROR;
  }

  GstMapInfo out_map;
  if (!gst_buffer_map(out, &out_map, GST_MAP_WRITE)) {
    gst_buffer_unref(out);
    return GST_FLOW_ERROR;
  }

  guint8 *out_data = out_map.data;
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

    guint8 *dst = &out_data[p * 3];
    if (total_weight <= 0.0f) {
      dst[0] = dst[1] = dst[2] = 0;
      continue;
    }

    float dominant_colour[3] = {0.0f, 0.0f, 0.0f};
    float average_colour[3] = {0.0f, 0.0f, 0.0f};

    for (guint i = 0; i < camera_count; ++i) {
      float *acc = self->planar_accumulators[i];
      float *weights = self->planar_weights[i];
      if (!acc || !weights) continue;
      float w = weights[p];
      if (w <= 0.0f) continue;

      float inv_w = 1.0f / w;
      gsize base = p * 3;
      float colour[3] = {
          acc[base + 0] * inv_w,
          acc[base + 1] * inv_w,
          acc[base + 2] * inv_w};

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

static GstFlowReturn gst_image_reprojection_process_equirect(GstImageReprojection *self,
                                                             GstBuffer **buffers,
                                                             GstMapInfo *maps,
                                                             GstBuffer **out_buffer_ptr) {
  const guint camera_count = self->camera_count;
  const gsize pixel_count = self->equirect.pixel_count;
  if (pixel_count == 0) {
    return GST_FLOW_ERROR;
  }

  gst_image_reprojection_reset_scratch_equirect(self);

  for (guint i = 0; i < camera_count; ++i) {
    if (!buffers[i]) continue;
    const guint8 *data = maps[i].data;
    if (!data) continue;

    const GstImageReprojectionPixelMap *mapping = self->equirect_maps[i];
    if (!mapping) continue;

    float *acc = self->equirect_accumulators[i];
    float *weights = self->equirect_weights[i];
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
      bilinear_sample(data, width_in, height_in, u, v, colour);

      gsize base = p * 3;
      acc[base + 0] += colour[0];
      acc[base + 1] += colour[1];
      acc[base + 2] += colour[2];
      weights[p] += 1.0f;
    }
  }

  GstBuffer *out = gst_buffer_new_allocate(NULL, self->equirect.width * self->equirect.height * 3, NULL);
  if (!out) {
    return GST_FLOW_ERROR;
  }

  GstMapInfo out_map;
  if (!gst_buffer_map(out, &out_map, GST_MAP_WRITE)) {
    gst_buffer_unref(out);
    return GST_FLOW_ERROR;
  }

  guint8 *out_data = out_map.data;
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

    guint8 *dst = &out_data[p * 3];
    if (total_weight <= 0.0f) {
      dst[0] = dst[1] = dst[2] = 0;
      continue;
    }

    float dominant_colour[3] = {0.0f, 0.0f, 0.0f};
    float average_colour[3] = {0.0f, 0.0f, 0.0f};

    for (guint i = 0; i < camera_count; ++i) {
      float *acc = self->equirect_accumulators[i];
      float *weights = self->equirect_weights[i];
      if (!acc || !weights) continue;
      float w = weights[p];
      if (w <= 0.0f) continue;

      float inv_w = 1.0f / w;
      gsize base = p * 3;
      float colour[3] = {
          acc[base + 0] * inv_w,
          acc[base + 1] * inv_w,
          acc[base + 2] * inv_w};

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

static GstFlowReturn gst_image_reprojection_aggregate(GstAggregator *aggregator, gboolean timeout) {
  GstImageReprojection *self = GST_IMAGE_REPROJECTION(aggregator);

  (void)timeout;

  g_mutex_lock(&self->lock);
  if (!self->config_loaded) {
    GError *error = NULL;
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
    GST_WARNING_OBJECT(self,
                       "Only %u sink pads connected but config expects %u cameras",
                       pad_count,
                       self->camera_count);
    self->warned_pad_count = TRUE;
  }

  GstBuffer **buffers = g_new0(GstBuffer *, self->camera_count);
  GstMapInfo *maps = g_new0(GstMapInfo, self->camera_count);
  GstBuffer *out_buffer = NULL;
  GstFlowReturn flow = GST_AGGREGATOR_FLOW_NEED_DATA;

  for (GList *l = GST_ELEMENT(self)->sinkpads; l; l = l->next) {
    GstAggregatorPad *pad = GST_AGGREGATOR_PAD(l->data);
    GstImageReprojectionPad *ip = GST_IMAGE_REPROJECTION_PAD(pad);
    if (ip->index >= self->camera_count) {
      GST_WARNING_OBJECT(self, "Ignoring sink pad %u beyond camera configuration", ip->index);
      continue;
    }

    GstBuffer *buffer = gst_aggregator_pad_pop_buffer(pad);
    if (!buffer) {
      if (gst_aggregator_pad_is_eos(pad)) {
        flow = GST_FLOW_EOS;
        goto need_more_data;
      }
      flow = GST_AGGREGATOR_FLOW_NEED_DATA;
      goto need_more_data;
    }

    GstVideoInfo info = ip->info;
    if (info.width == 0 || info.height == 0) {
      GstCaps *caps = gst_pad_get_current_caps(GST_PAD(pad));
      if (caps) {
        if (!gst_video_info_from_caps(&info, caps)) {
          GST_WARNING_OBJECT(self, "Failed to parse caps for pad %u", ip->index);
        }
        gst_caps_unref(caps);
      }
    }
    if (info.width > 0) {
      self->cameras[ip->index].width = info.width;
    }
    if (info.height > 0) {
      self->cameras[ip->index].height = info.height;
    }

    if (!gst_buffer_map(buffer, &maps[ip->index], GST_MAP_READ)) {
      GST_WARNING_OBJECT(self, "Failed to map buffer for pad %u", ip->index);
      gst_buffer_unref(buffer);
      flow = GST_FLOW_ERROR;
      goto need_more_data;
    }

    buffers[ip->index] = buffer;
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    if (!buffers[i]) {
      flow = GST_AGGREGATOR_FLOW_NEED_DATA;
      goto need_more_data;
    }
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

  return flow;
}

static void gst_image_reprojection_reset_planar(GstImageReprojectionPlanar *planar) {
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

static void gst_image_reprojection_reset_equirect(GstImageReprojectionEquirect *eq) {
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

static void gst_image_reprojection_clear_config(GstImageReprojection *self) {
  guint old_count = self->camera_count;

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

  gst_image_reprojection_reset_planar(&self->planar);
  gst_image_reprojection_reset_equirect(&self->equirect);

  gst_caps_replace(&self->src_caps, NULL);
  self->initial_events_pushed = FALSE;
  gst_segment_init(&self->segment, GST_FORMAT_TIME);

  self->config_loaded = FALSE;
}

static gboolean gst_image_reprojection_parse_camera(JsonObject *cam_obj,
                                                     GstImageReprojectionCamera *cam,
                                                     GError **error) {
  JsonObject *intr_obj = json_object_get_object_member(cam_obj, "intrinsics");
  if (!intr_obj) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Camera intrinsics missing");
    return FALSE;
  }

  cam->fx = json_object_get_double_member(intr_obj, "fx");
  cam->fy = json_object_get_double_member(intr_obj, "fy");
  cam->cx = json_object_get_double_member(intr_obj, "cx");
  cam->cy = json_object_get_double_member(intr_obj, "cy");
  cam->width = (int)json_object_get_int_member(intr_obj, "width");
  cam->height = (int)json_object_get_int_member(intr_obj, "height");

  JsonArray *planar_array = json_object_get_array_member(cam_obj, "planar_transform");
  cam->has_planar_matrix = planar_array && json_array_get_length(planar_array) >= 16;
  if (cam->has_planar_matrix) {
    for (guint i = 0; i < 16; ++i) {
      cam->planar_matrix[i] = json_array_get_double_element(planar_array, i);
    }
  }

  JsonArray *eq_array = json_object_get_array_member(cam_obj, "equirectangular_transform");
  cam->has_equirect_matrix = eq_array && json_array_get_length(eq_array) >= 16;
  if (cam->has_equirect_matrix) {
    for (guint i = 0; i < 16; ++i) {
      cam->equirect_matrix[i] = json_array_get_double_element(eq_array, i);
    }
  }

  return TRUE;
}

static gboolean gst_image_reprojection_parse_planar(JsonObject *root,
                                                    GstImageReprojectionPlanar *planar,
                                                    GError **error) {
  JsonObject *planar_obj = json_object_get_object_member(root, "planar");
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
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Planar configuration missing required fields");
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

  const gchar *frame_id = json_object_get_string_member_or(planar_obj, "frame_id", NULL);
  planar->frame_id = frame_id ? g_strdup(frame_id) : NULL;

  if (planar->width <= 0 || planar->height <= 0 || planar->fx <= 0.0 || planar->fy <= 0.0 || planar->depth <= 0.0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Invalid planar projection parameters");
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

static gboolean gst_image_reprojection_parse_equirect(JsonObject *root,
                                                      GstImageReprojectionEquirect *eq,
                                                      GError **error) {
  JsonObject *eq_obj = json_object_get_object_member(root, "equirectangular");
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
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Equirectangular configuration missing required fields");
    return FALSE;
  }

  eq->width = (gint)json_object_get_int_member(eq_obj, "width");
  eq->height = (gint)json_object_get_int_member(eq_obj, "height");
  eq->hfov_rad = json_object_get_double_member(eq_obj, "hfov_rad");
  eq->vfov_rad = json_object_get_double_member(eq_obj, "vfov_rad");
  eq->radius = json_object_get_double_member(eq_obj, "radius");
  eq->blend_factor = json_object_get_double_member(eq_obj, "blend_factor");

  const gchar *frame_id = json_object_get_string_member_or(eq_obj, "frame_id", NULL);
  eq->frame_id = frame_id ? g_strdup(frame_id) : NULL;

  GST_INFO("Equirect config: enabled=%d width=%d height=%d hfov=%.3f vfov=%.3f",
           eq->enabled,
           eq->width,
           eq->height,
           eq->hfov_rad,
           eq->vfov_rad);

  if (eq->width <= 0 || eq->height <= 0 || eq->radius <= 0.0 || eq->hfov_rad <= 0.0 || eq->vfov_rad <= 0.0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Invalid equirectangular projection parameters");
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

static gboolean gst_image_reprojection_load_config(GstImageReprojection *self, GError **error) {
  if (self->config_loaded) {
    return TRUE;
  }

  if (!self->config_path || !g_file_test(self->config_path, G_FILE_TEST_EXISTS)) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_NOT_FOUND,
                "Configuration path not set or file missing");
    return FALSE;
  }

  gst_image_reprojection_clear_config(self);

  JsonParser *parser = json_parser_new();
  if (!json_parser_load_from_file(parser, self->config_path, error)) {
    g_object_unref(parser);
    return FALSE;
  }

  JsonNode *root_node = json_parser_get_root(parser);
  if (!JSON_NODE_HOLDS_OBJECT(root_node)) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Configuration root is not an object");
    g_object_unref(parser);
    return FALSE;
  }

  JsonObject *root_obj = json_node_get_object(root_node);
  JsonArray *cam_array = json_object_get_array_member(root_obj, "cameras");
  if (!cam_array) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Configuration missing cameras array");
    g_object_unref(parser);
    return FALSE;
  }

  self->camera_count = json_array_get_length(cam_array);
  if (self->camera_count == 0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Configuration has no cameras");
    g_object_unref(parser);
    return FALSE;
  }

  self->cameras = g_new0(GstImageReprojectionCamera, self->camera_count);
  for (guint i = 0; i < self->camera_count; ++i) {
    JsonObject *cam_obj = json_array_get_object_element(cam_array, i);
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

  if (!self->planar.enabled && !self->equirect.enabled) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Configuration has no enabled projections");
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
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Planar projection requested but disabled in config");
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
  self->warned_pad_count = FALSE;
  g_object_unref(parser);
  return TRUE;
}

static void multiply_point(const double matrix[16], double x, double y, double z,
                           double *out_x, double *out_y, double *out_z) {
  *out_x = matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3];
  *out_y = matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7];
  *out_z = matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11];
}

static gboolean gst_image_reprojection_update_planar_maps(GstImageReprojection *self, GError **error) {
  if (!self->planar.enabled) {
    return TRUE;
  }

  const gsize pixel_count = self->planar.pixel_count;
  if (pixel_count == 0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Planar projection has zero pixels");
    return FALSE;
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    if (!self->cameras[i].has_planar_matrix) {
      g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                  "Camera %u missing planar transform", i);
      return FALSE;
    }
  }

  self->planar_maps = g_new0(GstImageReprojectionPixelMap *, self->camera_count);
  self->planar_accumulators = g_new0(float *, self->camera_count);
  self->planar_weights = g_new0(float *, self->camera_count);

  for (guint i = 0; i < self->camera_count; ++i) {
    self->planar_maps[i] = g_new0(GstImageReprojectionPixelMap, pixel_count);
    self->planar_accumulators[i] = g_new0(float, pixel_count * 3);
    self->planar_weights[i] = g_new0(float, pixel_count);
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    GstImageReprojectionPixelMap *mapping = self->planar_maps[i];
    const double *matrix = self->cameras[i].planar_matrix;

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

  return TRUE;
}

static gboolean gst_image_reprojection_update_equirect_maps(GstImageReprojection *self, GError **error) {
  if (!self->equirect.enabled) {
    return TRUE;
  }

  const gsize pixel_count = self->equirect.pixel_count;
  if (pixel_count == 0) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Equirectangular projection has zero pixels");
    return FALSE;
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    if (!self->cameras[i].has_equirect_matrix) {
      g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                  "Camera %u missing equirectangular transform", i);
      return FALSE;
    }
  }

  self->equirect_maps = g_new0(GstImageReprojectionPixelMap *, self->camera_count);
  self->equirect_accumulators = g_new0(float *, self->camera_count);
  self->equirect_weights = g_new0(float *, self->camera_count);

  for (guint i = 0; i < self->camera_count; ++i) {
    self->equirect_maps[i] = g_new0(GstImageReprojectionPixelMap, pixel_count);
    self->equirect_accumulators[i] = g_new0(float, pixel_count * 3);
    self->equirect_weights[i] = g_new0(float, pixel_count);
  }

  for (guint i = 0; i < self->camera_count; ++i) {
    GstImageReprojectionPixelMap *mapping = self->equirect_maps[i];
    const double *matrix = self->cameras[i].equirect_matrix;

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

  return TRUE;
}

static gboolean gst_image_reprojection_ensure_output_caps(GstImageReprojection *self, GError **error) {
  GstVideoInfo info;
  gst_video_info_init(&info);

  if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_PLANAR) {
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_BGR, self->planar.width, self->planar.height);
  } else if (self->active_mode == GST_IMAGE_REPROJECTION_MODE_EQUIRECT) {
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_BGR, self->equirect.width, self->equirect.height);
  } else {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Unsupported active projection mode");
    return FALSE;
  }

  GstCaps *caps = gst_video_info_to_caps(&info);
  if (!caps) {
    g_set_error(error, GST_RESOURCE_ERROR, GST_RESOURCE_ERROR_FAILED,
                "Failed to create src caps");
    return FALSE;
  }

  gst_caps_replace(&self->src_caps, caps);
  self->initial_events_pushed = FALSE;
  gst_caps_unref(caps);
  return TRUE;
}

static gboolean gst_image_reprojection_push_initial_events(GstImageReprojection *self, GstAggregator *aggregator) {
  if (self->initial_events_pushed) {
    return TRUE;
  }

  GstPad *srcpad = GST_AGGREGATOR_SRC_PAD(aggregator);

  gchar *stream_id = g_strdup_printf("imagereprojection-%p-%" G_GUINT64_FORMAT,
                                     (void *)self,
                                     (guint64)g_get_monotonic_time());

  GstEvent *stream_event = gst_event_new_stream_start(stream_id);
  g_free(stream_id);
  if (!gst_pad_push_event(srcpad, stream_event)) {
    GST_WARNING_OBJECT(self, "Failed to push stream-start event");
    return FALSE;
  }

  GstCaps *caps = self->src_caps ? gst_caps_ref(self->src_caps) : NULL;
  if (!caps) {
    GST_WARNING_OBJECT(self, "No caps available for src pad");
    return FALSE;
  }

  gst_aggregator_set_src_caps(GST_AGGREGATOR(self), caps);
  gst_caps_unref(caps);

  gst_segment_init(&self->segment, GST_FORMAT_TIME);
  GstEvent *segment_event = gst_event_new_segment(&self->segment);
  if (!gst_pad_push_event(srcpad, segment_event)) {
    GST_WARNING_OBJECT(self, "Failed to push segment event");
    return FALSE;
  }

  self->initial_events_pushed = TRUE;
  return TRUE;
}

static void gst_image_reprojection_reset_scratch_planar(GstImageReprojection *self) {
  if (!self->planar_accumulators || !self->planar_weights) return;
  const gsize pixel_count = self->planar.pixel_count;
  const gsize acc_size = pixel_count * 3;
  for (guint i = 0; i < self->camera_count; ++i) {
    if (self->planar_accumulators[i]) memset(self->planar_accumulators[i], 0, sizeof(float) * acc_size);
    if (self->planar_weights[i]) memset(self->planar_weights[i], 0, sizeof(float) * pixel_count);
  }
}

static void gst_image_reprojection_reset_scratch_equirect(GstImageReprojection *self) {
  if (!self->equirect_accumulators || !self->equirect_weights) return;
  const gsize pixel_count = self->equirect.pixel_count;
  const gsize acc_size = pixel_count * 3;
  for (guint i = 0; i < self->camera_count; ++i) {
    if (self->equirect_accumulators[i]) memset(self->equirect_accumulators[i], 0, sizeof(float) * acc_size);
    if (self->equirect_weights[i]) memset(self->equirect_weights[i], 0, sizeof(float) * pixel_count);
  }
}

static gboolean gst_image_reprojection_plugin_init(GstPlugin *plugin) {
  return gst_element_register(plugin, "imagereprojection", GST_RANK_NONE, GST_TYPE_IMAGE_REPROJECTION);
}

static GType gst_image_reprojection_mode_get_type(void) {
  static GType mode_type = 0;
  if (mode_type == 0) {
    static const GEnumValue values[] = {
        {GST_IMAGE_REPROJECTION_MODE_AUTO, "Auto", "auto"},
        {GST_IMAGE_REPROJECTION_MODE_PLANAR, "Planar", "planar"},
        {GST_IMAGE_REPROJECTION_MODE_EQUIRECT, "Equirectangular", "equirectangular"},
        {0, NULL, NULL}};
    mode_type = g_enum_register_static("GstImageReprojectionMode", values);
  }
  return mode_type;
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    imagereprojection,
    "Image Reprojection",
    gst_image_reprojection_plugin_init,
    "1.0.0",
    "MIT",
    "gst_image_reprojection",
    "https://github.com/ika-rwth-aachen")

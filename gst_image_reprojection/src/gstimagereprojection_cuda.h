#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint64_t source_offset;
  uint32_t camera;
  uint32_t right_step;
  uint32_t down_step;
  float dx;
  float dy;
} GstImageReprojectionCudaMap;

typedef struct {
  float u;
  float v;
} GstImageReprojectionCudaPixelMap;

typedef struct GstImageReprojectionCuda GstImageReprojectionCuda;

#ifdef __cplusplus
extern "C" {
#endif
GstImageReprojectionCuda* gst_image_reprojection_cuda_create(
    const GstImageReprojectionCudaMap* maps, size_t pixel_count, unsigned width,
    unsigned height, unsigned output_stride, const size_t* camera_bytes, unsigned camera_count);
bool gst_image_reprojection_cuda_render(GstImageReprojectionCuda* context,
                                        const uint8_t* const* inputs, uint8_t* output);
GstImageReprojectionCuda* gst_image_reprojection_cuda_create_multi(
    const GstImageReprojectionCudaPixelMap* maps, size_t pixel_count, unsigned width,
    unsigned height, unsigned output_stride, const size_t* camera_bytes,
    const unsigned* camera_widths, const unsigned* camera_heights,
    const unsigned* camera_strides, unsigned camera_count);
bool gst_image_reprojection_cuda_render_multi(GstImageReprojectionCuda* context,
                                              const uint8_t* const* inputs,
                                              uint8_t* output, float blend);
void gst_image_reprojection_cuda_destroy(GstImageReprojectionCuda* context);
#ifdef __cplusplus
}
#endif

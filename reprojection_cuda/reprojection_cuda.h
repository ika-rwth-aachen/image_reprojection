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
} ReprojectionCudaMap;

typedef struct {
  float u;
  float v;
} ReprojectionCudaPixelMap;

typedef struct ReprojectionCuda ReprojectionCuda;

#ifdef __cplusplus
extern "C" {
#endif
ReprojectionCuda* reprojection_cuda_create(
    const ReprojectionCudaMap* maps, size_t pixel_count, unsigned width,
    unsigned height, unsigned output_stride, const size_t* camera_bytes, unsigned camera_count);
bool reprojection_cuda_render(ReprojectionCuda* context,
                                        const uint8_t* const* inputs, uint8_t* output);
ReprojectionCuda* reprojection_cuda_create_multi(
    const ReprojectionCudaPixelMap* maps, size_t pixel_count, unsigned width,
    unsigned height, unsigned output_stride, const size_t* camera_bytes,
    const unsigned* camera_widths, const unsigned* camera_heights,
    const unsigned* camera_strides, unsigned camera_count);
bool reprojection_cuda_render_multi(ReprojectionCuda* context,
                                              const uint8_t* const* inputs,
                                              uint8_t* output, float blend);
void reprojection_cuda_destroy(ReprojectionCuda* context);
#ifdef __cplusplus
}
#endif

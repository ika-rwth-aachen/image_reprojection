#include "gstimagereprojection_cuda.h"

#include <cuda_runtime.h>
#include <stdlib.h>
#include <string.h>

struct GstImageReprojectionCuda {
  GstImageReprojectionCudaMap* maps;
  GstImageReprojectionCudaPixelMap* pixel_maps;
  uint8_t* inputs;
  uint8_t* output;
  uint8_t* active;
  uint8_t* host_active;
  size_t* camera_bytes;
  size_t* camera_offsets;
  size_t* device_camera_offsets;
  unsigned* camera_widths;
  unsigned* camera_heights;
  unsigned* camera_strides;
  size_t output_bytes;
  size_t pixel_count;
  unsigned width;
  unsigned output_stride;
  unsigned camera_count;
};

static __device__ void sample_bgr(const uint8_t* input, unsigned stride, unsigned width,
                                  unsigned height, float u, float v, float color[3]) {
  const unsigned x0 = (unsigned)floorf(u);
  const unsigned y0 = (unsigned)floorf(v);
  const unsigned x1 = x0 + 1 < width ? x0 + 1 : x0;
  const unsigned y1 = y0 + 1 < height ? y0 + 1 : y0;
  const float dx = u - (float)x0;
  const float dy = v - (float)y0;
  const uint8_t* top = input + (size_t)y0 * stride;
  const uint8_t* bottom = input + (size_t)y1 * stride;
  for (unsigned c = 0; c < 3; ++c) {
    const float upper = (1.0f - dx) * top[x0 * 3 + c] + dx * top[x1 * 3 + c];
    const float lower = (1.0f - dx) * bottom[x0 * 3 + c] + dx * bottom[x1 * 3 + c];
    color[c] = (1.0f - dy) * upper + dy * lower;
  }
}

static __global__ void project_multi(const GstImageReprojectionCudaPixelMap* maps,
                                     const uint8_t* inputs, const uint8_t* active,
                                     const size_t* camera_offsets, const unsigned* camera_widths,
                                     const unsigned* camera_heights, const unsigned* camera_strides,
                                     uint8_t* output, size_t pixel_count, unsigned width,
                                     unsigned output_stride, unsigned camera_count, float blend) {
  const size_t p = (size_t)blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= pixel_count) return;
  uint8_t* dst = output + (p / width) * output_stride + (p % width) * 3;
  float dominant[3] = {0.0f, 0.0f, 0.0f};
  float total[3] = {0.0f, 0.0f, 0.0f};
  unsigned contributors = 0;
  for (unsigned i = 0; i < camera_count; ++i) {
    if (!active[i]) continue;
    const GstImageReprojectionCudaPixelMap map = maps[(size_t)i * pixel_count + p];
    const unsigned in_width = camera_widths[i], in_height = camera_heights[i];
    if (!isfinite(map.u) || !isfinite(map.v) || map.u < 0.0f || map.v < 0.0f ||
        map.u > (float)(in_width - 1) || map.v > (float)(in_height - 1)) continue;
    float color[3];
    sample_bgr(inputs + camera_offsets[i], camera_strides[i], in_width, in_height,
               map.u, map.v, color);
    if (contributors == 0) {
      dominant[0] = color[0];
      dominant[1] = color[1];
      dominant[2] = color[2];
    }
    if (blend == 0.0f) {
      dst[0] = (uint8_t)color[0];
      dst[1] = (uint8_t)color[1];
      dst[2] = (uint8_t)color[2];
      return;
    }
    total[0] += color[0];
    total[1] += color[1];
    total[2] += color[2];
    ++contributors;
  }
  if (contributors == 0) {
    dst[0] = dst[1] = dst[2] = 0;
    return;
  }
  const float inv_count = 1.0f / (float)contributors;
  for (unsigned c = 0; c < 3; ++c) {
    const float value = (1.0f - blend) * dominant[c] + blend * total[c] * inv_count;
    dst[c] = (uint8_t)fminf(fmaxf(value, 0.0f), 255.0f);
  }
}

static __global__ void project(const GstImageReprojectionCudaMap* maps,
                               const uint8_t* inputs, uint8_t* output,
                               size_t pixel_count, unsigned width, unsigned output_stride) {
  const size_t p = (size_t)blockIdx.x * blockDim.x + threadIdx.x;
  if (p >= pixel_count) return;
  const GstImageReprojectionCudaMap entry = maps[p];
  uint8_t* dst = output + (p / width) * output_stride + (p % width) * 3;
  if (entry.camera == UINT32_MAX) {
    dst[0] = dst[1] = dst[2] = 0;
    return;
  }
  const uint8_t* top = inputs + entry.source_offset;
  const uint8_t* bottom = top + entry.down_step;
  for (unsigned c = 0; c < 3; ++c) {
    const float upper = (1.0f - entry.dx) * top[c] + entry.dx * top[entry.right_step + c];
    const float lower = (1.0f - entry.dx) * bottom[c] + entry.dx * bottom[entry.right_step + c];
    dst[c] = (uint8_t)((1.0f - entry.dy) * upper + entry.dy * lower);
  }
}

extern "C" GstImageReprojectionCuda* gst_image_reprojection_cuda_create(
    const GstImageReprojectionCudaMap* maps, size_t pixel_count, unsigned width,
    unsigned height, unsigned output_stride, const size_t* camera_bytes, unsigned camera_count) {
  if (!maps || !pixel_count || !width || !height || !camera_count) return NULL;
  GstImageReprojectionCuda* ctx = (GstImageReprojectionCuda*)calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;
  ctx->pixel_count = pixel_count;
  ctx->width = width;
  ctx->output_stride = output_stride;
  ctx->output_bytes = (size_t)height * output_stride;
  ctx->camera_count = camera_count;
  ctx->camera_bytes = (size_t*)malloc(camera_count * sizeof(size_t));
  ctx->camera_offsets = (size_t*)malloc(camera_count * sizeof(size_t));
  GstImageReprojectionCudaMap* combined =
      (GstImageReprojectionCudaMap*)malloc(pixel_count * sizeof(*combined));
  size_t total_input_bytes = 0;
  if (!ctx->camera_bytes || !ctx->camera_offsets || !combined) goto failed;
  for (unsigned i = 0; i < camera_count; ++i) {
    ctx->camera_bytes[i] = camera_bytes[i];
    ctx->camera_offsets[i] = total_input_bytes;
    total_input_bytes += camera_bytes[i];
  }
  memcpy(combined, maps, pixel_count * sizeof(*combined));
  for (size_t p = 0; p < pixel_count; ++p) {
    if (combined[p].camera == UINT32_MAX) continue;
    if (combined[p].camera >= camera_count) goto failed;
    combined[p].source_offset += ctx->camera_offsets[combined[p].camera];
  }
  if (cudaMalloc(&ctx->maps, pixel_count * sizeof(*combined)) != cudaSuccess ||
      cudaMalloc(&ctx->inputs, total_input_bytes) != cudaSuccess ||
      cudaMalloc(&ctx->output, ctx->output_bytes) != cudaSuccess ||
      cudaMemcpy(ctx->maps, combined, pixel_count * sizeof(*combined), cudaMemcpyHostToDevice) != cudaSuccess) {
    goto failed;
  }
  if (cudaMemset(ctx->inputs, 0, total_input_bytes) != cudaSuccess ||
      cudaMemset(ctx->output, 0, ctx->output_bytes) != cudaSuccess) goto failed;
  project<<<(pixel_count + 255) / 256, 256>>>(
      ctx->maps, ctx->inputs, ctx->output, pixel_count, width, output_stride);
  if (cudaGetLastError() != cudaSuccess || cudaDeviceSynchronize() != cudaSuccess) goto failed;
  free(combined);
  return ctx;
failed:
  free(combined);
  gst_image_reprojection_cuda_destroy(ctx);
  return NULL;
}

extern "C" bool gst_image_reprojection_cuda_render(GstImageReprojectionCuda* ctx,
                                                    const uint8_t* const* inputs, uint8_t* output) {
  if (!ctx || !inputs || !output) return false;
  for (unsigned i = 0; i < ctx->camera_count; ++i) {
    if (cudaMemcpy(ctx->inputs + ctx->camera_offsets[i], inputs[i], ctx->camera_bytes[i],
                   cudaMemcpyHostToDevice) != cudaSuccess) return false;
  }
  if (ctx->output_stride != ctx->width * 3 &&
      cudaMemset(ctx->output, 0, ctx->output_bytes) != cudaSuccess) return false;
  project<<<(ctx->pixel_count + 255) / 256, 256>>>(
      ctx->maps, ctx->inputs, ctx->output, ctx->pixel_count, ctx->width, ctx->output_stride);
  return cudaGetLastError() == cudaSuccess &&
         cudaMemcpy(output, ctx->output, ctx->output_bytes, cudaMemcpyDeviceToHost) == cudaSuccess;
}

extern "C" GstImageReprojectionCuda* gst_image_reprojection_cuda_create_multi(
    const GstImageReprojectionCudaPixelMap* maps, size_t pixel_count, unsigned width,
    unsigned height, unsigned output_stride, const size_t* camera_bytes,
    const unsigned* camera_widths, const unsigned* camera_heights,
    const unsigned* camera_strides, unsigned camera_count) {
  if (!maps || !pixel_count || !width || !height || !camera_count) return NULL;
  GstImageReprojectionCuda* ctx = (GstImageReprojectionCuda*)calloc(1, sizeof(*ctx));
  if (!ctx) return NULL;
  ctx->pixel_count = pixel_count;
  ctx->width = width;
  ctx->output_stride = output_stride;
  ctx->output_bytes = (size_t)height * output_stride;
  ctx->camera_count = camera_count;
  ctx->camera_bytes = (size_t*)malloc(camera_count * sizeof(size_t));
  ctx->camera_offsets = (size_t*)malloc(camera_count * sizeof(size_t));
  ctx->host_active = (uint8_t*)malloc(camera_count);
  size_t total_input_bytes = 0;
  if (!ctx->camera_bytes || !ctx->camera_offsets || !ctx->host_active) goto failed;
  for (unsigned i = 0; i < camera_count; ++i) {
    ctx->camera_bytes[i] = camera_bytes[i];
    ctx->camera_offsets[i] = total_input_bytes;
    total_input_bytes += camera_bytes[i];
  }
  if (cudaMalloc(&ctx->pixel_maps, (size_t)camera_count * pixel_count * sizeof(*maps)) != cudaSuccess ||
      cudaMalloc(&ctx->inputs, total_input_bytes) != cudaSuccess ||
      cudaMalloc(&ctx->output, ctx->output_bytes) != cudaSuccess ||
      cudaMalloc(&ctx->active, camera_count) != cudaSuccess ||
      cudaMalloc(&ctx->device_camera_offsets, camera_count * sizeof(size_t)) != cudaSuccess ||
      cudaMalloc(&ctx->camera_widths, camera_count * sizeof(unsigned)) != cudaSuccess ||
      cudaMalloc(&ctx->camera_heights, camera_count * sizeof(unsigned)) != cudaSuccess ||
      cudaMalloc(&ctx->camera_strides, camera_count * sizeof(unsigned)) != cudaSuccess ||
      cudaMemcpy(ctx->pixel_maps, maps, (size_t)camera_count * pixel_count * sizeof(*maps),
                 cudaMemcpyHostToDevice) != cudaSuccess ||
      cudaMemcpy(ctx->device_camera_offsets, ctx->camera_offsets, camera_count * sizeof(size_t),
                 cudaMemcpyHostToDevice) != cudaSuccess ||
      cudaMemcpy(ctx->camera_widths, camera_widths, camera_count * sizeof(unsigned),
                 cudaMemcpyHostToDevice) != cudaSuccess ||
      cudaMemcpy(ctx->camera_heights, camera_heights, camera_count * sizeof(unsigned),
                 cudaMemcpyHostToDevice) != cudaSuccess ||
      cudaMemcpy(ctx->camera_strides, camera_strides, camera_count * sizeof(unsigned),
                 cudaMemcpyHostToDevice) != cudaSuccess ||
      cudaMemset(ctx->active, 0, camera_count) != cudaSuccess ||
      cudaMemset(ctx->output, 0, ctx->output_bytes) != cudaSuccess) goto failed;
  project_multi<<<(pixel_count + 255) / 256, 256>>>(
      ctx->pixel_maps, ctx->inputs, ctx->active, ctx->device_camera_offsets,
      ctx->camera_widths, ctx->camera_heights, ctx->camera_strides,
      ctx->output, pixel_count, width, output_stride, camera_count, 1.0f);
  if (cudaGetLastError() != cudaSuccess || cudaDeviceSynchronize() != cudaSuccess) goto failed;
  return ctx;
failed:
  gst_image_reprojection_cuda_destroy(ctx);
  return NULL;
}

extern "C" bool gst_image_reprojection_cuda_render_multi(GstImageReprojectionCuda* ctx,
                                                          const uint8_t* const* inputs,
                                                          uint8_t* output, float blend) {
  if (!ctx || !inputs || !output) return false;
  for (unsigned i = 0; i < ctx->camera_count; ++i) {
    ctx->host_active[i] = inputs[i] != NULL;
    if (!inputs[i]) continue;
    if (cudaMemcpy(ctx->inputs + ctx->camera_offsets[i], inputs[i], ctx->camera_bytes[i],
                   cudaMemcpyHostToDevice) != cudaSuccess) return false;
  }
  if (cudaMemcpy(ctx->active, ctx->host_active, ctx->camera_count,
                 cudaMemcpyHostToDevice) != cudaSuccess) return false;
  if (ctx->output_stride != ctx->width * 3 &&
      cudaMemset(ctx->output, 0, ctx->output_bytes) != cudaSuccess) return false;
  project_multi<<<(ctx->pixel_count + 255) / 256, 256>>>(
      ctx->pixel_maps, ctx->inputs, ctx->active, ctx->device_camera_offsets,
      ctx->camera_widths, ctx->camera_heights, ctx->camera_strides,
      ctx->output, ctx->pixel_count, ctx->width, ctx->output_stride,
      ctx->camera_count, blend);
  return cudaGetLastError() == cudaSuccess &&
         cudaMemcpy(output, ctx->output, ctx->output_bytes, cudaMemcpyDeviceToHost) == cudaSuccess;
}

extern "C" void gst_image_reprojection_cuda_destroy(GstImageReprojectionCuda* ctx) {
  if (!ctx) return;
  if (ctx->maps) cudaFree(ctx->maps);
  if (ctx->pixel_maps) cudaFree(ctx->pixel_maps);
  if (ctx->inputs) cudaFree(ctx->inputs);
  if (ctx->output) cudaFree(ctx->output);
  if (ctx->active) cudaFree(ctx->active);
  if (ctx->device_camera_offsets) cudaFree(ctx->device_camera_offsets);
  if (ctx->camera_widths) cudaFree(ctx->camera_widths);
  if (ctx->camera_heights) cudaFree(ctx->camera_heights);
  if (ctx->camera_strides) cudaFree(ctx->camera_strides);
  free(ctx->host_active);
  free(ctx->camera_bytes);
  free(ctx->camera_offsets);
  free(ctx);
}

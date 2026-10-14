#!/usr/bin/env python3
"""Compare CPU and CUDA from the last appsrc push to the matching appsink sample."""

import json
import math
import statistics
import tempfile
import time

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: F401

Gst.init(None)

IW, IH = 1280, 720


def config(mode, count, blend, partial):
    cameras = []
    for i in range(count):
        planar = [1, 0, 0, (i - 1) * -0.95, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        yaw = 2 * math.pi * i / count
        equi = [math.cos(yaw), 0, -math.sin(yaw), 0,
                0, 1, 0, 0, math.sin(yaw), 0, math.cos(yaw), 0,
                0, 0, 0, 1]
        cameras.append({"intrinsics": {"fx": 850, "fy": 850, "cx": IW / 2, "cy": IH / 2,
                                        "width": IW, "height": IH},
                        "planar_transform": planar, "equirectangular_transform": equi})
    return {"camera_count": count, "cameras": cameras,
            "planar": {"enabled": mode == "planar", "width": 2560, "height": 590,
                       "fx": 850, "fy": 850, "cx": 1280, "cy": 295,
                       "depth": 1, "blend_factor": blend},
            "equirectangular": {"enabled": mode == "equirectangular",
                                "width": 2560, "height": 640,
                                "hfov_rad": 2 * math.pi, "vfov_rad": math.pi / 2,
                                "radius": 1, "blend_factor": blend},
            "sync": {"mode": "lead_latest" if partial else "wait_all", "frame_timeout": 0.05,
                     "frame_time_tolerance": 0.005, "wait_all_publish_partial": True}}


def run(mode, count, use_gpu, blend, partial):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as f:
        json.dump(config(mode, count, blend, partial), f)
        f.flush()
        pipeline = Gst.Pipeline.new(None)
        element = Gst.ElementFactory.make("imagereprojection")
        sink = Gst.ElementFactory.make("appsink")
        if not element or not sink:
            raise RuntimeError("missing GStreamer element")
        element.set_property("config-path", f.name)
        element.set_property("projection-mode", mode)
        element.set_property("use-gpu", use_gpu)
        sink.set_property("sync", False)
        sink.set_property("max-buffers", 1)
        pipeline.add(element)
        pipeline.add(sink)
        assert element.link(sink)
        sources, templates = [], []
        stride = IW * 3
        for cam in range(count):
            src = Gst.ElementFactory.make("appsrc")
            src.set_property("is-live", True)
            src.set_property("format", Gst.Format.TIME)
            src.set_property("caps", Gst.Caps.from_string(
                f"video/x-raw,format=BGR,width={IW},height={IH},framerate=30/1"))
            pipeline.add(src)
            pad = element.request_pad(element.get_pad_template("sink_%u"), None, None)
            assert src.get_static_pad("src").link(pad) == Gst.PadLinkReturn.OK
            sources.append(src)
            b = Gst.Buffer.new_allocate(None, stride * IH, None)
            pixels = bytearray(stride * IH)
            for y in range(IH):
                pixels[y * stride:(y + 1) * stride] = bytes(
                    ((x * 17 + y * 7 + cam * 25 + 11) % 251 for x in range(stride)))
            b.fill(0, pixels)
            templates.append(b)
        startup_start = time.perf_counter()
        assert pipeline.set_state(Gst.State.PLAYING) != Gst.StateChangeReturn.FAILURE
        startup_ms = (time.perf_counter() - startup_start) * 1000
        times, image, first = [], None, None
        try:
            for j in range(35):
                for cam in list(range(1, count)) + [0]:
                    if partial and cam == count - 1:
                        continue
                    src = sources[cam]
                    b = templates[cam].copy()
                    b.pts = j * Gst.SECOND // 30
                    b.duration = Gst.SECOND // 30
                    assert src.emit("push-buffer", b) == Gst.FlowReturn.OK
                sent = time.perf_counter()
                sample = sink.try_pull_sample(2 * Gst.SECOND)
                if sample is None:
                    error = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
                    raise RuntimeError(f"no output frame {j}: {error.parse_error() if error else 'timeout'}")
                done = time.perf_counter()
                if j == 0:
                    first = (done - sent) * 1000
                if j >= 5:
                    times.append((done - sent) * 1000)
                if j == 5:
                    mapped, data = sample.get_buffer().map(Gst.MapFlags.READ)
                    assert mapped
                    image = bytes(data.data)
                    sample.get_buffer().unmap(data)
        finally:
            pipeline.set_state(Gst.State.NULL)
        return times, image, first, startup_ms


for mode, count in (("planar", 3), ("equirectangular", 8)):
    for blend, partial in ((0.0, False), (0.0, True), (0.5, False), (1.0, False), (1.0, True)):
        cpu, cpu_image, cpu_first, cpu_startup = run(mode, count, False, blend, partial)
        gpu, gpu_image, gpu_first, gpu_startup = run(mode, count, True, blend, partial)
        assert len(cpu_image) == len(gpu_image)
        max_error = max(abs(a - b) for a, b in zip(cpu_image, gpu_image))
        print(f"{mode} {count - partial}/{count} cameras blend={blend}, 30 frames after 5 warmup: "
              f"CPU average={statistics.mean(cpu):.3f}ms median={statistics.median(cpu):.3f}ms "
              f"GPU average={statistics.mean(gpu):.3f}ms median={statistics.median(gpu):.3f}ms "
              f"speedup={statistics.mean(cpu)/statistics.mean(gpu):.2f}x max_error={max_error} "
              f"first_frame_cpu={cpu_first:.1f}ms first_frame_gpu={gpu_first:.1f}ms "
              f"startup_cpu={cpu_startup:.1f}ms startup_gpu={gpu_startup:.1f}ms",
              flush=True)

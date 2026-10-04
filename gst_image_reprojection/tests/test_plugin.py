#!/usr/bin/env python3
"""Run with GST_PLUGIN_PATH pointing at the built plugin directory."""

import json
import tempfile
import time
import unittest

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402,F401

Gst.init(None)


def make_config(projection, partial=True, sync_mode="wait_all", input_width=4, output_width=8):
    identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    cameras = []
    for shift in (0.5, -0.5):
        transform = identity.copy()
        transform[3] = shift
        cameras.append({
            "intrinsics": {"fx": 4, "fy": 4, "cx": input_width / 2, "cy": 2,
                           "width": input_width, "height": 4},
            "planar_transform": transform,
            "equirectangular_transform": transform,
        })
    return {
        "camera_count": 2,
        "cameras": cameras,
        "planar": {
            "enabled": projection == "planar", "width": output_width, "height": 4,
            "fx": 4, "fy": 4, "cx": output_width / 2, "cy": 2, "depth": 1, "blend_factor": 0,
        },
        "equirectangular": {
            "enabled": projection == "equirectangular", "width": output_width, "height": 4,
            "hfov_rad": 3.141592653589793, "vfov_rad": 1.5707963267948966,
            "radius": 1, "blend_factor": 0,
        },
        "sync": {
            "mode": sync_mode, "frame_timeout": 0.05,
            "frame_time_tolerance": 0.005,
            "wait_all_publish_partial": partial,
        },
    }


def run_pipeline(projection, sequence, partial=True, sync_mode="wait_all", input_width=4, output_width=8):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as config_file:
        json.dump(make_config(projection, partial, sync_mode, input_width, output_width), config_file)
        config_file.flush()
        pipeline = Gst.Pipeline.new(None)
        reprojection = Gst.ElementFactory.make("imagereprojection")
        if reprojection is None:
            raise RuntimeError("imagereprojection plugin not found in GST_PLUGIN_PATH")
        reprojection.set_property("config-path", config_file.name)
        reprojection.set_property("projection-mode", projection)
        sink = Gst.ElementFactory.make("appsink")
        sink.set_property("sync", False)
        pipeline.add(reprojection)
        pipeline.add(sink)
        assert reprojection.link(sink)

        sources = []
        for camera in range(2):
            source = Gst.ElementFactory.make("appsrc")
            source.set_property("is-live", True)
            source.set_property("format", Gst.Format.TIME)
            source.set_property("caps", Gst.Caps.from_string(
                f"video/x-raw,format=BGR,width={input_width},height=4,framerate=30/1"))
            pipeline.add(source)
            pad = reprojection.request_pad(reprojection.get_pad_template("sink_%u"), None, None)
            assert pad.get_name() == f"sink_{camera}"
            assert source.get_static_pad("src").link(pad) == Gst.PadLinkReturn.OK
            sources.append(source)

        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("pipeline did not start")
        outputs = []
        try:
            for camera, pts in sequence:
                input_stride = (input_width * 3 + 3) & ~3
                buffer = Gst.Buffer.new_allocate(None, input_stride * 4, None)
                pixels = bytearray()
                for y in range(4):
                    for x in range(input_width):
                        pixels.extend((
                            (x * 40 + y * 7 + camera * 50) % 256,
                            (x * 9 + y * 31 + camera * 30) % 256,
                            (x * 17 + y * 13 + camera * 20) % 256,
                        ))
                    pixels.extend(b"\xff" * (input_stride - input_width * 3))
                buffer.fill(0, pixels)
                buffer.pts = pts
                buffer.duration = Gst.SECOND // 30
                assert sources[camera].emit("push-buffer", buffer) == Gst.FlowReturn.OK
                time.sleep(0.005)
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                sample = sink.try_pull_sample(10 * Gst.MSECOND)
                if sample:
                    buffer = sample.get_buffer()
                    ok, data = buffer.map(Gst.MapFlags.READ)
                    assert ok
                    outputs.append((buffer.pts, bytes(data.data)))
                    buffer.unmap(data)
            error = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            if error:
                raise RuntimeError(str(error.parse_error()))
        finally:
            pipeline.set_state(Gst.State.NULL)
        return outputs


class PluginTest(unittest.TestCase):
    def test_wait_all_complete_and_partial(self):
        for projection in ("planar", "equirectangular"):
            with self.subTest(projection=projection):
                full = run_pipeline(projection, [(1, 0), (0, 0)])
                partial = run_pipeline(projection, [(0, 0)])
                discarded = run_pipeline(projection, [(0, 0)], partial=False)
                self.assertEqual([pts for pts, _ in full], [0])
                self.assertEqual([pts for pts, _ in partial], [0])
                self.assertEqual(discarded, [])
                self.assertEqual(len(full[0][1]), 8 * 4 * 3)
                self.assertNotEqual(full[0][1], partial[0][1])
                if projection == "planar":
                    right_pixel = (1 * 8 + 7) * 3
                    self.assertNotEqual(full[0][1][right_pixel:right_pixel + 3], b"\0\0\0")
                    self.assertEqual(partial[0][1][right_pixel:right_pixel + 3], b"\0\0\0")

    def test_wait_all_timeout_after_stream_stops(self):
        for partial in (True, False):
            outputs = run_pipeline("planar", [(1, 0), (0, 0), (0, Gst.SECOND // 30)], partial=partial)
            self.assertEqual([pts for pts, _ in outputs], [0, Gst.SECOND // 30] if partial else [0])

    def test_lead_latest_reuses_recent_camera(self):
        outputs = run_pipeline("planar", [(1, 0), (0, 0), (0, Gst.MSECOND)], sync_mode="lead_latest")
        self.assertEqual([pts for pts, _ in outputs], [0, Gst.MSECOND])
        right_pixel = (1 * 8 + 7) * 3
        self.assertNotEqual(outputs[1][1][right_pixel:right_pixel + 3], b"\0\0\0")

    def test_padded_bgr_rows(self):
        output = run_pipeline("planar", [(1, 0), (0, 0)], input_width=5, output_width=7)
        self.assertEqual(len(output), 1)
        self.assertEqual(len(output[0][1]), 24 * 4)
        self.assertNotEqual(output[0][1][24:27], b"\0\0\0")
        self.assertEqual(output[0][1][21:24], b"\0\0\0")


if __name__ == "__main__":
    unittest.main()

echo "export GST_PLUGIN_PATH=/docker-ros/ws/install/gst_image_reprojection/lib:\${GST_PLUGIN_PATH}" >> ~/.bashrc

# Fail the docker-ros image build if it would silently compile the CPU-only node.
command -v nvcc >/dev/null || { echo "CUDA compiler missing before colcon build" >&2; exit 1; }

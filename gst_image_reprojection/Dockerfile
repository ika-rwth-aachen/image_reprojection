FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libjson-glib-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --config Release \
    && cmake --install build --prefix /opt/gstreamer

FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    libjson-glib-1.0-0 \
  && rm -rf /var/lib/apt/lists/*

COPY --from=build /opt/gstreamer /opt/gstreamer

ENV GST_PLUGIN_PATH=/opt/gstreamer/lib/gstreamer-1.0${GST_PLUGIN_PATH:+:${GST_PLUGIN_PATH}}

CMD ["gst-inspect-1.0", "imagereprojection"]

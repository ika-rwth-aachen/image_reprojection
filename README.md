# image_reprojection

<p align="center">
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/ika-rwth-aachen/image_reprojection/releases/latest"><img src="https://img.shields.io/github/v/release/ika-rwth-aachen/image_reprojection"/></a>
  <a href="https://github.com/ika-rwth-aachen/image_reprojection/blob/main/LICENSE"><img src="https://img.shields.io/github/license/ika-rwth-aachen/image_reprojection"/></a>
  <br>
  <a href="https://github.com/ika-rwth-aachen/image_reprojection/actions/workflows/docker-ros.yml"><img src="https://github.com/ika-rwth-aachen/image_reprojection/actions/workflows/docker-ros.yml/badge.svg"/></a>
</p>

**Planar and Equirectangular Image Reprojection/Stitching for ROS 2 and GStreamer**

The [image_reprojection](./image_reprojection/README.md) node reprojects an arbitrary number of input camera images to a common stitched output image. The node uses available camera intrinsics (CameraInfo) and extrinsics (tf) to compute the projection. The [gst_image_reprojection](./gst_image_reprojection/README.md) GStreamer plugin provides the same functionality as a GStreamer element. Two projection methods are supported:
- **Planar Projection:** All input images are projected to a planar surface at a given depth.
- **Equirectangular Projection:** All input images are projected to a sphere with radius with a given radius and then unrolled to a 2D image using [equirectangular projection](https://www.researchgate.net/profile/Kailun-Yang/publication/347957084/figure/fig1/AS:974408793481217@1609328568414/llustration-of-the-equirectangular-projection_W640.jpg).

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>

## 🚀 Quick Start

**TODO: Teaser GIF**

1. Launch the [`demo/docker-compose.yml`](./demo/docker-compose.yml) setup. This will play a ROS 2 bag containing multiple camera feeds, launch the image reprojection node, and visualizate both planar and equirectangular projections in RViz.
    ```bash
    cd demo
    xhost +local: # allow GUI forwarding from containers
    docker compose up -d
    ```
2. In the RQt window that has also been opened, modify the reprojection parameters (e.g., `output.projection.planar.fov_x` or `output.projection.equirectangular.radius`) and see the effect in RViz.
3. Stop the demo and clean up.
    ```bash
    docker compose down
    xhost -local: # revoke GUI forwarding permissions
    ```

For an exemplary GStreamer pipeline, see [Example Pipeline](./gst_image_reprojection/README.md#example-pipeline).

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/ika-rwth-aachen/image_reprojection.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd image_reprojection
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```

## 📝 Documentation

Package and node interfaces are documented in the respective package READMEs listed below.

| Package | Description |
| --- | --- |
| [image_reprojection](image_reprojection/README.md) | ROS 2 package to reproject multiple camera images using various projection methods |
| [gst_image_reprojection](gst_image_reprojection/README.md) | GStreamer plugin to reproject multiple camera images using various projection methods |

## 📚 Citation

This repository has been published in the context of the following journal article. We hope that our tools can help your research. If this is the case, please cite it using the metadata specified in [CITATION.cff](CITATION.cff), or click on *Cite this repository* in GitHub's *About* section on the top right.

> **Digital Triplet for Developing Future Reliable 6G Cooperative Intelligent Transport Systems: Case Studies from Teleoperation to Platooning**
>
> Hendrik Schippers, Niklas A. Wagner, Lennart Reiher, Harun Teper, Lukas Ostendorf, Jian-Jia Chen, Lutz Eckstein, Christian Wietfeld
>
> <sup><sub>
> *Abstract* –
> Future Cooperative Intelligent Transport Systems (C-ITSs) will impose stringent requirements
> on future 6G mobile networks in terms of latency, data rate, and reliability. These requirements must be met
> in rapidly changing wireless channel conditions. Hence, their fulfillment requires rigorous and reproducible
> testing environments that enable a seamless transition from simulation to real-world deployment. However,
> existing testbeds and simulation environments either lack physical realism or fail to scale cost-effectively,
> hindering comprehensive C-ITS evaluation. To overcome these limitations, we implement the concept of a
> Digital Triplet, comprising three complementary layers: a full-scale real-world test field, a co-simulation
> environment for integrating additional simulated entities, and a scaled-down replica of the test field. This
> architecture enables reliable, reproducible, and scalable testing of future 6G-enabled C-ITS applications
> across different levels of abstraction. We demonstrate the applicability of the Digital Triplet through multiple
> case studies. For remote driving, we deploy and optimize a low-latency video streaming system and simulate
> a mmWave communication channel incorporating Reconfigurable Intelligent Surfaces (RISs). An up to 74%
> reduced median Glass-to-Glass (G2G) latency is achieved in a mobile network compared to related work
> using a local network. Moreover, emulating real-world vehicle localization in the scaled test field enables the
> implementation of cross-domain Nonlinear Model Predictive Control (NMPC)-based vehicular platooning.
> Finally, we evaluate the NMPC platooning controller in both decentralized (on-vehicle) and centralized
> (edge cloud) configurations. The results highlight the strong potential of the Digital Triplet to accelerate
> the development and validation of C-ITS applications within future 6G networks.
> </sup></sup>

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

Development and maintenance of this repository are supported by the following projects. We acknowledge the funding of the respective institutions.

| Project | Funding Institution | Grant Number |
| --- | --- | --- |
| [6GEM+](https://6gem.de) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 16KIS2409K |
| [6GEM](https://6gem.de) | 🇩🇪 Federal Ministry for Research, Technology and Space (BMFTR) | 16KISK036K |

<p>
  <img src="https://www.drought.uni-freiburg.de/stressres/images/bmftr-logo/image" height=70>
</p>

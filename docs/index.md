---
hide:
  - navigation
---

# modular-slam

<figure markdown="span">
  ![Image title](img/modular_slam_main_img.png){ width="300" }
</figure>

## Description

`modular-slam` is a flexible, high-performance library for building and experimenting with Simultaneous Localization and Mapping (SLAM) systems in both Python and C++.

It’s designed to make SLAM prototyping fast and enjoyable: you can mix and match components - data acquisition blocks, optimization backends, data association strategies, and more—without fighting a large, monolithic codebase.

Use `modular-slam` when you want to:

- :wrench: **Prototype quickly** – swap components in and out with minimal boilerplate.
- :microscope: **Explore new ideas** – compare different models, solvers, or data-association strategies side by side.
- :rocket: **Scale to real systems** – go from small experiments to real-world SLAM deployments without rewriting everything.

In short: experiment freely, iterate quickly, and keep full control over your SLAM pipeline. :compass:


## Features

- :jigsaw: **Modular architecture**  
  Compose SLAM pipelines from interchangeable pieces: localization, mapping, optimization, sensor fusion, and more.

- :arrows_counterclockwise: **Multi-language**  
  Use Python for rapid prototyping and C++ for performance-critical components, without maintaining two completely separate codebases.

- :package: **Prebuilt Components**  
  Common SLAM building blocks such as visual odometry, IMU fusion, and graph-based optimization are provided out of the box.

- :bricks: **Extensible by design**  
  Plug in your own sensors, cost functions, or backends via a clean extension / plugin system.

- :globe_with_meridians: **Cross-platform**  
  Target Linux, macOS, and Windows with a single codebase.

- :stopwatch: **Real-time oriented**  
  Built around efficient data structures and parallel execution to support real-time SLAM workloads.


## Installation

!!! warning "`modular-slam` is under active development."

    The commands below describe **planned** installation options and are **not yet available** on PyPI/Conan.

=== "pip (Python)"

    ```bash
    pip install modular-slam
    ```
=== "conan (C++)"

    ```bash
    conan install --requires=boost/1.88.0
    ```


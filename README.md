# modular-slam

Modular SLAM is library for Simultanous Localization and Mapping problem. It's highly focused on modularity allowing users to customize their SLAM algorithm to their needs.
Modular SLAM is written in C++17 and Python3. It's designed to work as separate library but it also supports ROS.


# Compilation

Modular SLAM uses Conan to manage their dependencies. To install conan:

```
pip3 install conan # for default python2 in the system
pip install conan # for default python3 in the system
```

Afterwards, go to Modular SLAM directory and type:

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug # or Release
```


# Documentation

To generate doxygen documentation make sure Doxygen is installed and type:


```
...
cmake .. -DCMAKE_BUILD_TYPE=Debug -DGEN_DOCS=ON # or Release
make doc
```

# modular-slam

Modular SLAM is library for Simultanous Localization and Mapping problem. It's highly focused on modularity allowing users to customize their SLAM algorithm to their needs.
Modular SLAM is written in C++17 and Python3. It's designed to work as separate library but it also supports ROS.



# Compilation

Modular SLAM uses Conan to manage their dependencies. To install conan:

```
pip install conan
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

# clang-format before commit (pre-commit hook)
Before install hook be sure that you have **clang-format** installed.
To install clang-format on ubuntu run:

`sudo apt-get install clang-format`

To install git pre-commit hooks in project root directory run:


    cp ci/scripts/pre-commit .git/hooks/
    chmod +x .git/hooks/pre-commit

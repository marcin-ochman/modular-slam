

# File py\_modular\_slam.cpp

[**File List**](files.md) **>** [**cpp**](dir_dfdbda394c3f7a3aa55229f33a559c41.md) **>** [**lib**](dir_dad8a9c0c98047bf60bfc83527d74517.md) **>** [**modular\_slam**](dir_462d9d1e7dff16c7e45b1d3ebe1d409a.md) **>** [**python\_bindings**](dir_2561bd41b00cb16bde1ad400968620f7.md) **>** [**py\_modular\_slam.cpp**](py__modular__slam_8cpp.md)

[Go to the documentation of this file](py__modular__slam_8cpp.md)


```C++
#include <pybind11/pybind11.h>

int add(int x, int y)
{
    return x + y;
}

PYBIND11_MODULE(_modular_slam, m)
{
    m.doc() = "C++ core bindings for your_project";

    m.def("add", &add, "Add two integers");
}
```



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

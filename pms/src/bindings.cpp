#include <nanobind/nanobind.h>

#include "function.cpp"

NB_MODULE(pms, m) {
    m.def("add", &add);
}

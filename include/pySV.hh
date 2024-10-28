// C++
#include <iostream>
#include <vector>

// Pybind
#include "pybind11/pybind11.h"
#include "pybind11/embed.h"  // Everything needed for embedding Python
#include "pybind11/stl.h"    // pybind11 support for STL containers like vector
#include <pybind11/numpy.h>  // For handling NumPy arrays

namespace py = pybind11;

// FFmpeg
extern "C"{
    #include "libavcodec/avcodec.h"
    #include <libavutil/imgutils.h>
}

void py_test();

void py_process(AVFrame* frame, AVFrame*& res);

void initialize_python_environment();

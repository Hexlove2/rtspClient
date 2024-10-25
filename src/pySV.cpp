#include "pySV.hh"

// Python environment and module can be initialized once and reused
py::scoped_interpreter* guard = nullptr;  // Start the Python interpreter once
py::module sys;
py::module processing_module;

void initialize_python_environment() {

    guard = new py::scoped_interpreter();  // Start the Python interpreter once
    sys = py::module::import("sys");

    // Ensure sys.path is correctly set once
    sys.attr("path").attr("append")("/Users/sunday/Desktop/Code/rtspClient/src");
    sys.attr("path").attr("append")("/opt/anaconda3/envs/media/lib/python3.9/site-packages");

    // Import the module once
    processing_module = py::module::import("superV");
}


void py_test() {
    py::scoped_interpreter guard{};  // Start the interpreter

    py::module sys = py::module::import("sys");
    std::cout << "Python version: " << py::str(sys.attr("version")) << std::endl;
    std::cout << "Python path: " << py::str(sys.attr("path")) << std::endl;

    sys.attr("path").attr("append")("/opt/anaconda3/envs/media/lib/python3.9/site-packages");

    // Import the numpy module
    py::module numpy = py::module::import("numpy");

    // Define a C++ std::vector and cast it to a Python list using py::cast
    std::vector<int> vec = {1, 2, 3};
    py::list arr = py::cast(vec);

    // Call numpy.array() on the converted list
    py::print(numpy.attr("array")(arr));  // This will print the numpy array
}


py::tuple convert_avframe_to_numpy(AVFrame* frame) {
    int width = frame->width;
    int height = frame->height;

    // Create NumPy arrays wrapping the Y, U, and V planes
    py::array_t<uint8_t> y_plane = py::array_t<uint8_t>({height, width}, {frame->linesize[0], 1}, frame->data[0]);
    py::array_t<uint8_t> u_plane = py::array_t<uint8_t>({height / 2, width / 2}, {frame->linesize[1], 1}, frame->data[1]);
    py::array_t<uint8_t> v_plane = py::array_t<uint8_t>({height / 2, width / 2}, {frame->linesize[2], 1}, frame->data[2]);

    // Return the YUV planes as a tuple to Python
    return py::make_tuple(y_plane, u_plane, v_plane);
}

void py_process(AVFrame* frame){

    // Convert AVFrame to NumPy array
    py::tuple numpy_frame = convert_avframe_to_numpy(frame);

    // Call the process_frame function in the Python module
    processing_module.attr("process_yuv_frame")(numpy_frame[0], numpy_frame[1], numpy_frame[2], frame->width, frame->height);
}
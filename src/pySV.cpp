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

void py_process(AVFrame* frame, AVFrame*& res){

    // Convert AVFrame to NumPy array
    // Py::tuple numpy_frame = convert_avframe_to_numpy(frame);


    py::array_t<uint8_t> y_plane({frame->height, frame->width}, frame->data[0]);
    py::array_t<uint8_t> u_plane({frame->height / 2, frame->width / 2}, frame->data[1]);
    py::array_t<uint8_t> v_plane({frame->height / 2, frame->width / 2}, frame->data[2]);


    py::tuple result = processing_module.attr("process_yuv_frame")(y_plane, u_plane,
                                                     v_plane, frame->width, frame->height);

    py::array_t<uint8_t> new_y_plane = result[0].cast<py::array_t<uint8_t>>();
    py::array_t<uint8_t> new_u_plane = result[1].cast<py::array_t<uint8_t>>();
    py::array_t<uint8_t> new_v_plane = result[2].cast<py::array_t<uint8_t>>();

    auto y_info = new_y_plane.request();
    auto u_info = new_u_plane.request();
    auto v_info = new_v_plane.request();

    AVFrame* new_frame = av_frame_alloc();
    new_frame->format = frame->format;
    new_frame->width = frame->width;
    new_frame->height = frame->height;

    int buffer_size = av_image_alloc(new_frame->data, new_frame->linesize,
                                     new_frame->width, new_frame->height,
                                     static_cast<AVPixelFormat>(new_frame->format), 1);
    
    //将返回的数组内容复制回 frame->data
    std::memmove(new_frame->data[0], y_info.ptr, frame->height * frame->width);
    std::memmove(new_frame->data[1], u_info.ptr, (frame->height / 2) * (frame->width / 2));
    std::memmove(new_frame->data[2], v_info.ptr, (frame->height / 2) * (frame->width / 2));

    res = new_frame;
    std::cout<<"test";
}

    // Call the process_frame function in the Python module
    // uint8_t* data1 = new uint8_t[frame->width*frame->height];
    // uint8_t* data2 = new uint8_t[frame->width*frame->height/4];
    // uint8_t* data3 = new uint8_t[frame->width*frame->height/4];
    // memcpy(data1, frame->data[0], frame->height*frame->width);
    // memcpy(data2, frame->data[1], frame->height*frame->width/4);
    // memcpy(data3, frame->data[2], frame->height*frame->width/4);
    // processing_module.attr("process_yuv_frame")(reinterpret_cast<uintptr_t>(data1),
    //                                         reinterpret_cast<uintptr_t>(data2),
    //                                         reinterpret_cast<uintptr_t>(data3),
    //                                         frame->width, frame->height);
    // memcpy(frame->data[0], data1, frame->height*frame->width);
    // memcpy(frame->data[1], data2, frame->height*frame->width/4);
    // memcpy(frame->data[2], data3, frame->height*frame->width/4);
    // delete[] data1;
    // delete[] data2;
    // delete[] data3;
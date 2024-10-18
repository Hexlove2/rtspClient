# 最低的 CMake 版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称和 C++ 标准设置
project(rtspClient)

# 设置 C++ 标准版本
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 生成 compile_commands.json 文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_OSX_ARCHITECTURES "arm64")

# 设置输出目录为 bin/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# 添加 include 目录
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/ffmpeg)
include_directories(${CMAKE_SOURCE_DIR}/include/live555/BasicUsageEnvironment)
include_directories(${CMAKE_SOURCE_DIR}/include/live555/groupsock)
include_directories(${CMAKE_SOURCE_DIR}/include/live555/liveMedia)
include_directories(${CMAKE_SOURCE_DIR}/include/live555/UsageEnvironment)
include_directories(${CMAKE_SOURCE_DIR}/include/ffmpeg)
include_directories(/opt/homebrew/opt/openssl@3/include)


# 设置库路径
link_directories(${CMAKE_SOURCE_DIR}/lib)

# 查找 src 文件夹中的所有 .cpp 文件并添加到项目中
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# 添加可执行文件
add_executable(rtspClient ${SOURCES})

# 链接静态库和必要的库
target_link_libraries(rtspClient
    libavcodec.61.19.100.dylib
    libliveMedia.a
    libgroupsock.a
    libBasicUsageEnvironment.a
    libUsageEnvironment.a
    -lpthread
    -lssl
    -lcrypto
    "-framework AudioToolbox"     # 用于音频编解码
    "-framework CoreFoundation"   # 基础框架
    "-framework VideoToolbox"     # 视频硬件加速
    "-framework CoreMedia"        # 媒体类型
    "-framework CoreVideo"        # 视频缓冲区管理
    "-framework CoreAudio"        # 音频 I/O

)

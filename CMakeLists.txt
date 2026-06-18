cmake_minimum_required(VERSION 3.16)
project(CppSLAM CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)

add_executable(slam
    src/main.cpp
    src/PlyParser.cpp
    src/KDTree.cpp
    src/ICP.cpp
    src/PoseGraph.cpp
    src/ImuOdometry.cpp
    src/ScanContext.cpp
    src/VoxelGrid.cpp
    src/Odometry.cpp
    src/SubmapRegistration.cpp
    src/SubmapBuilder.cpp
    src/LoopCloser.cpp
    src/MapBuilder.cpp
    src/BagParser.cpp
    src/IMUPreintegrator.cpp
)

target_include_directories(slam PRIVATE src)
target_compile_options(slam PRIVATE -O2 -Wall)

find_package(Pangolin REQUIRED)

# macOS Homebrew GLEW 경로 직접 지정
set(GLEW_INCLUDE "/opt/homebrew/include")
set(GLEW_LIB    "/opt/homebrew/lib/libGLEW.dylib")

target_sources(slam PRIVATE src/PangolinViewer.cpp)
target_include_directories(slam PRIVATE
    ${Pangolin_INCLUDE_DIRS}
    ${GLEW_INCLUDE}
    /usr/local/include
)
target_link_libraries(slam
    gtsam
    ${Pangolin_LIBRARIES}
    ${GLEW_LIB}
)

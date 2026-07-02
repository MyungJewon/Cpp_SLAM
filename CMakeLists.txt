cmake_minimum_required(VERSION 3.16)
project(CppSLAM CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)

# slam / mapmerge / test_merge가 공유하는 GUI-비의존 코어 소스.
set(CORE_SOURCES
    src/PlyParser.cpp
    src/KDTree.cpp
    src/ICP.cpp
    src/PoseGraph.cpp
    src/ScanContext.cpp
    src/VoxelGrid.cpp
    src/SubmapRegistration.cpp
    src/SubmapBuilder.cpp
    src/LoopCloser.cpp
    src/MapBuilder.cpp
    src/IMUPreintegrator.cpp
    src/SessionIO.cpp
    src/SessionMerger.cpp
)

add_executable(slam
    ${CORE_SOURCES}
    src/main.cpp
    src/ImuOdometry.cpp
    src/Odometry.cpp
    src/BagParser.cpp
)

target_include_directories(slam PRIVATE src)
target_compile_options(slam PRIVATE -O3 -Wall)

# 오프라인 병합 도구 (GUI/Pangolin 불필요).
add_executable(mapmerge src/mapmerge_main.cpp ${CORE_SOURCES})
target_include_directories(mapmerge PRIVATE src)
target_compile_options(mapmerge PRIVATE -O3 -Wall)
target_link_libraries(mapmerge gtsam)

# 단위 테스트.
add_executable(test_merge tests/test_merge.cpp ${CORE_SOURCES})
target_include_directories(test_merge PRIVATE src)
target_link_libraries(test_merge gtsam)
enable_testing()
add_test(NAME merge_unit COMMAND test_merge)

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

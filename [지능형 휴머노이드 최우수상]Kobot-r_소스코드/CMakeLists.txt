cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH}")
find_package(raspicam REQUIRED)
find_package(OpenCV)

add_executable(main main.cpp RobotProtocol.cpp Koo.cpp)
target_link_libraries(main ${raspicam_CV_LIBS})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

cmake_minimum_required(VERSION 3.5)
project(DroneOnboardController)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS -pthread)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

option(USE_OPENCV "Should compile with OpenCV" YES)
IF ( USE_OPENCV )
    MESSAGE( STATUS "Including OpenCV on DroneOnboardController build" )
    find_package( OpenCV 3.4 REQUIRED)
    if (OpenCV_LIBS)
        add_definitions(-DWITH_OPENCV)
    else()
        message("You requested OPENCV in the build, but none was found.")
    endif()
ENDIF ()


#TARGET_LINK_LIBRARIES( ${execName})
add_executable(DroneOnboardController
        main.cpp
        StatsServer.cpp
        StatsServer.h
        CameraModule.cpp
        CameraModule.h
        DroneImage.cpp
        DroneImage.h
        ../Utils/MutexBool.cpp
        ../Utils/MutexBool.h)


if (OpenCV_LIBS)
    target_link_libraries(DroneOnboardController ${OpenCV_LIBS})
    target_include_directories(DroneOnboardController PUBLIC Utils)

endif()
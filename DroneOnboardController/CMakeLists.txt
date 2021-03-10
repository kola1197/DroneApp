cmake_minimum_required(VERSION 3.5)
project(DroneOnboardController)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS -pthread)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/bin)


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
#include_directories($(SOURCE_DIR)/../../../include)

#if(WIN32)
#	find_library(Guidance_Lib DJI_guidance.lib HINTS $(SOURCE_DIR)/../../../../lib/2013/x86)
#else()
#	target_link_libraries( dji_guidance_usb pthread usb-1.0)
#endif()

#TARGET_LINK_LIBRARIES( ${execName})
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    message("-- ARM architecture detected")
    option(USE_OPENCV "Should compile with OpenCV" YES)
    IF ( USE_OPENCV )
        MESSAGE( STATUS "Including OpenCV on DroneOnboardController build" )
        find_package( OpenCV 4.5.1 REQUIRED)
        if (OpenCV_LIBS)
            add_definitions(-DWITH_OPENCV)
        else()
            message("You requested OPENCV in the build, but none was found.")
        endif()
    ENDIF ()
    add_executable(
        DroneOnboardController
        main.cpp
        StatsServer.cpp
        StatsServer.h
        CameraModule.cpp
        CameraModule.h
        DroneImage.cpp
        DroneImage.h
        #DJI_guidance.h
        #DJI_utility.h
        #DJI_utility.cpp
        ../Utils/MutexBool.cpp
        ../Utils/MutexBool.h
        ../Utils/Messages.cpp
        ../Utils/Messages.h
        OdometryModule.cpp
        OdometryModule.h
        ../Utils/AsyncVar.h
        ../Utils/AsyncVar.cpp
            Settings.cpp Settings.h)
    #find_library(Guidance_Lib DJI_guidance HINTS $(SOURCE_DIR)/..)
    #target_link_libraries(DroneOnboardController ${OpenCV_LIBS} ${Guidance_Lib} usb-1.0 pthread)
ELSE()
    message("-- x64 architecture detected")
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
    #set(CMAKE_AUTOUIC ON)
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    find_package(realsense2 REQUIRED)
    add_executable(
        DroneOnboardController
        main.cpp
        StatsServer.cpp
        StatsServer.h
        CameraModule.cpp
        CameraModule.h
        DroneImage.cpp
        DroneImage.h
        ../Utils/MutexBool.cpp
        ../Utils/MutexBool.h
        ../Utils/Messages.cpp
        ../Utils/Messages.h
        ../Utils/AsyncVar.h
        ../Utils/AsyncVar.cpp
        OdometryModule.cpp
        OdometryModule.h
        depthmapcalibrator.ui
        depthmapcalibrator.cpp
        depthmapcalibrator.h
        MAVConnector.cpp
        MAVConnector.h
        #dmcalibrator.ui
        #dmcalibrator.cpp
        #mcalibrator.h
        Settings.cpp
        Settings.h
        PX4Comannder.cpp
        PX4Comannder.h
        commandinput.cpp
        commandinput.h
        command.cpp
        command.h
        StoppableThread.cpp
        StoppableThread.h
        joystickinput.h
        joystickinput.cpp
    )
    target_link_libraries(DroneOnboardController PRIVATE Qt5::Widgets ${OpenCV_LIBS} ${realsense2_LIBRARY} pthread )
    #target_link_libraries(DroneOnboardController ${OpenCV_LIBS} pthread)
ENDIF()
target_include_directories(DroneOnboardController PUBLIC Utils)


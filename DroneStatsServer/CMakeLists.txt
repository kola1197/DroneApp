cmake_minimum_required(VERSION 3.5)

project(DroneStatsServer LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS -pthread)

option(USE_OPENCV "Should compile with OpenCV" YES)
IF ( USE_OPENCV )
    MESSAGE( STATUS "Including OpenCV on DroneStatsServer build" )
    find_package( OpenCV 3.4 REQUIRED)
    if (OpenCV_LIBS)
        add_definitions(-DWITH_OPENCV)
    else()
        message("You requested OPENCV in the build, but none was found.")
    endif()
ENDIF ()


# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)


  add_executable(DroneStatsServer
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    statsclient.cpp
    statsclient.h
    ../Utils/MutexBool.cpp
    ../Utils/MutexBool.h
    ../Utils/Messages.cpp
    ../Utils/Messages.h
  )


target_link_libraries(DroneStatsServer PRIVATE Qt5::Widgets ${OpenCV_LIBS} )

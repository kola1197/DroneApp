cmake_minimum_required(VERSION 3.5)

project(DroneApp)
set(TEST_SUBDIRECTORIES
   DroneOnboardController
   DroneStatsServer
)

foreach(test_subdirectory ${TEST_SUBDIRECTORIES})
    message(STATUS "adding subdirectory applications/${test_subdirectory}")
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/${test_subdirectory})
endforeach(test_subdirectory)


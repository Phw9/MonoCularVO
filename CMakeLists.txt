cmake_minimum_required(VERSION 3.12)
project(MonoVO)


message(STATUS "${PROJECT_NAME}")
message(STATUS "${PROJECT_SOURCE_DIR}")
message(STATUS "${PROJECT_BINARY_DIR}") 

find_package(OpenCV 3.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.2 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/include/)
include_directories(${PROJECT_SOURCE_DIR}/include/)

add_executable(MonoVO
${PROJECT_SOURCE_DIR}/main/main.cpp
${PROJECT_SOURCE_DIR}/main/Init.h
${PROJECT_SOURCE_DIR}/main/Init.cpp)

target_link_libraries(MonoVO ${OpenCV_LIBS})
target_link_libraries(MonoVO 
                     ${PROJECT_SOURCE_DIR}/src/BundleAdjustment.cpp
                     ${PROJECT_SOURCE_DIR}/src/CalcMatrix.cpp
                     ${PROJECT_SOURCE_DIR}/src/FeatureDetection.cpp
                     ${PROJECT_SOURCE_DIR}/src/KeyData.cpp
                     ${PROJECT_SOURCE_DIR}/src/PoseEstimation.cpp
                     ${PROJECT_SOURCE_DIR}/src/Triangulation.cpp)
target_link_libraries(MonoVO
                     ${PROJECT_SOURCE_DIR}/main/libgtest_main.a
                     ${PROJECT_SOURCE_DIR}/main/libgtest.a)

target_COMPILE_OPTIONS(MonoVO PUBLIC -g -Wall)
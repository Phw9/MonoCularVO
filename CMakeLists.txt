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

add_executable(MonoVO
${PROJECT_SOURCE_DIR}/main/main.cpp
${PROJECT_SOURCE_DIR}/main/Init.h
${PROJECT_SOURCE_DIR}/main/Init.cpp
${PROJECT_SOURCE_DIR}/include/BundleAdjustment.h
${PROJECT_SOURCE_DIR}/include/CalcMatrix.h
${PROJECT_SOURCE_DIR}/include/FeatureDetection.h
${PROJECT_SOURCE_DIR}/include/KeyData.h
${PROJECT_SOURCE_DIR}/include/Triangulation.h
${PROJECT_SOURCE_DIR}/src/BundleAdjustment.cpp
${PROJECT_SOURCE_DIR}/src/CalcMatrix.cpp
${PROJECT_SOURCE_DIR}/src/FeatureDetection.cpp
${PROJECT_SOURCE_DIR}/src/KeyData.cpp
${PROJECT_SOURCE_DIR}/src/Triangulation.cpp)

target_link_libraries(MonoVO ${OpenCV_LIBS})

target_COMPILE_OPTIONS(MonoVO PUBLIC -g -Wall)
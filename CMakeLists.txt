cmake_minimum_required(VERSION 2.8)
project(MonoVO)


message(STATUS "${PROJECT_NAME}")
message(STATUS "${PROJECT_SOURCE_DIR}")
message(STATUS "${PROJECT_BINARY_DIR}") 

find_package(OpenCV 3.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV 3.2 not found.")
   endif()

find_package(OpenGL REQUIRED)
   if(NOT OpenGL_FOUND)
      message(FATAL_ERROR "OpenGL not found.")
   endif()

find_package(GLUT REQUIRED)
   if(NOT GLUT_FOUND)
      message(FATAL_ERROR "GLUT not found.")
   endif()

find_package(Pangolin REQUIRED)
   if(NOT Pangolin_FOUND)
      message(FATAL_ERROR "Pangolin not found.")
   endif()


MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

include_directories(
${OpenCV_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include/include/
${PROJECT_SOURCE_DIR}/include/
${OPENGL_INCLUDE_DIRS}
${GLUT_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)


add_executable(MonoVO
${PROJECT_SOURCE_DIR}/main/main.cpp
${PROJECT_SOURCE_DIR}/main/Init.h
${PROJECT_SOURCE_DIR}/main/Init.cpp
)

target_link_libraries(MonoVO ${OpenCV_LIBS})

target_link_libraries(MonoVO 
                     ${PROJECT_SOURCE_DIR}/src/BundleAdjustment.cpp
                     ${PROJECT_SOURCE_DIR}/src/CalcMatrix.cpp
                     ${PROJECT_SOURCE_DIR}/src/FeatureDetection.cpp
                     ${PROJECT_SOURCE_DIR}/src/KeyData.cpp
                     ${PROJECT_SOURCE_DIR}/src/PoseEstimation.cpp
                     ${PROJECT_SOURCE_DIR}/src/Triangulation.cpp)
                     
# add_library(MonoVO STATIC
#             ${PROJECT_SOURCE_DIR}/src/BundleAdjustment.cpp
#             ${PROJECT_SOURCE_DIR}/src/CalcMatrix.cpp
#             ${PROJECT_SOURCE_DIR}/src/FeatureDetection.cpp
#             ${PROJECT_SOURCE_DIR}/src/KeyData.cpp
#             ${PROJECT_SOURCE_DIR}/src/PoseEstimation.cpp
#             ${PROJECT_SOURCE_DIR}/src/Triangulation.cpp)

target_link_libraries(MonoVO
                     ${PROJECT_SOURCE_DIR}/main/libgtest_main.a
                     ${PROJECT_SOURCE_DIR}/main/libgtest.a
		     )
	  
target_link_libraries(MonoVO 
${OPENGL_LIBRARIES}
${GLUT_LIBRARY}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
pango_display
pango_python
)

target_COMPILE_OPTIONS(MonoVO PUBLIC -g -Wall)



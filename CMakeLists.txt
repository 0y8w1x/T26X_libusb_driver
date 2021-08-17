cmake_minimum_required(VERSION 3.16.3)
project(t265_arm_android)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

add_executable(test_usb test_usb.cpp)
target_link_libraries(test_usb ${OpenCV_LIBS} ${realsense2_LIBRARY} ${glfw3_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} usb-1.0)

add_executable(write_firmware write_firmware.cpp)
target_link_libraries(write_firmware usb-1.0)

#add_executable(rs_image rs_image.cpp)
#target_link_libraries(rs_image ${OpenCV_LIBS} ${realsense2_LIBRARY} ${glfw3_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

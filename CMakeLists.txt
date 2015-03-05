cmake_minimum_required(VERSION 2.8.3)
project(kinect_driver2)

find_package(catkin REQUIRED COMPONENTS
    rgbd
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIR}/libfreenect2/tinythread
    ${catkin_INCLUDE_DIRS}
)

add_executable(kinect_driver2
    src/driver.cpp
)
target_link_libraries(kinect_driver2 ${catkin_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBRARIES})





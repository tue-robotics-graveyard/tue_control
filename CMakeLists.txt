cmake_minimum_required(VERSION 2.8.3)
project(tue_control)

find_package(catkin REQUIRED COMPONENTS
    scl_filters
    tue_config
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tue_control
  CATKIN_DEPENDS scl_filters
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES *.cpp)
file(GLOB_RECURSE HEADER_FILES *.h)

add_library(tue_control ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(tue_control ${catkin_LIBRARIES})

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
  src/controller.cpp
  src/controller_factory.cpp
  src/supervised_controller.cpp

  src/setpoint_controller.cpp    
  src/generic_controller.cpp
)

set(HEADER_FILES
  include/tue/control/generic.h
  include/tue/control/controller.h
  include/tue/control/controller_input.h
  include/tue/control/controller_output.h
  include/tue/control/controller_factory.h
  include/tue/control/supervised_controller.h

  include/tue/control/setpoint_controller.h  
  include/tue/control/generic_controller.h
)

add_library(tue_control ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(tue_control ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                              TEST
# ------------------------------------------------------------------------------------------------

add_executable(test_controller test/test_controller.cpp)
target_link_libraries(test_controller tue_control)

# cmake_minimum_required(VERSION 3.0.2)
# project(hybrid_astar_planner)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_COMPILER "g++")

# cmake_minimum_required(VERSION 3.10)
# project(hybrid_astar_planner LANGUAGES CXX C)

# message(STATUS "main:")
# message(STATUS "opt_planner")
# message(STATUS "Author: Long qinying")

# #set(CMAKE_VERBOSE_MAKEFILE "true")

# # Compiler
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

# if (COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif (COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else ()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
#                          Please use a different C++ compiler")
# endif ()
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# # set( CMAKE_BUILD_TYPE "Debug" )
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE Release)
# endif()

# set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

cmake_minimum_required(VERSION 3.10)
project(hybrid_astar_planner LANGUAGES CXX C)

message(STATUS "bspline optimization")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set( CMAKE_BUILD_TYPE "Debug" )
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")


# add_subdirectory(test_the_plugin)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pluginlib
  costmap_2d
  roscpp
  nav_core
  tf2_ros
  ompl
  common
  bspline_opt
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  geometry_msgs
  pluginlib
  roscpp
  nav_core
  tf2_ros
  ompl
  common
  bspline_opt
)

include_directories(
  include
  test_the_plugin/include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# find_package(ompl REQUIRED)

# if(NOT OMPL_FOUND)
#     message(AUTHOR_WARNING,"Open Motion Planning Library not found")
# endif(NOT OMPL_FOUND)


add_executable(test_planner test_the_plugin/src/test.cpp test_the_plugin/src/test_plugins.cpp)
target_link_libraries(test_planner ${catkin_LIBRARIES})

add_executable(tf_test_broadcaster test_the_plugin/src/tf_broadcaster.cpp)
target_link_libraries(tf_test_broadcaster ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/planner_core.cpp 
  src/hybrid_astar.cpp 
  src/node2d.cpp
  src/a_start.cpp
  src/visualize.cpp
  src/node3d.cpp
  src/hybrid_astar.cpp
  src/algorithm.cpp
  src/dubins.cpp
  src/ReedsShepp.cpp
)
# ${catkin_LIBRARIES} COMMON_LIB BSPLINE_OPT_LIB
link_directories(${catkin_LIB_DIRS}  /home/stitch/test/testdeploy_ws/devel/lib)


target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} COMMON_LIB BSPLINE_OPT_LIB)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include ${EIGEN3_INCLUDE_DIR} )

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
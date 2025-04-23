cmake_minimum_required(VERSION 3.0.2)
project(single_uwb_relative_localization)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  rospy
  std_msgs
filters
gnuplot
nlink_parser
  message_generation
cmake_modules
)
find_package(Eigen3 REQUIRED)

catkin_package(
   CATKIN_DEPENDS message_runtime eigen
#  INCLUDE_DIRS include
#  LIBRARIES scan_speed
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)
include_directories(/usr/include/suitesparse/)

include_directories(${Eigen_INCLUDE_DIRS})

include_directories(../filters/src/)

include_directories(../gnuplot/src/)

include_directories(./src/include/)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})

SET(LIBS_LINK cholmod cxsparse g2o_stuff g2o_core g2o_parser g2o_solver_eigen g2o_types_sim3 g2o_cli g2o_incremental g2o_simulator g2o_solver_pcg g2o_types_data g2o_types_slam2d_addons g2o_interactive g2o_solver_cholmod g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_csparse_extension g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d_addons g2o_ext_freeglut_minimal g2o_opengl_helper g2o_solver_dense  g2o_types_sclam2d g2o_types_slam3d)

add_executable(relative_localization_node src/relative_localization_node.cpp)

target_link_libraries(relative_localization_node pf_6d)
target_link_libraries(relative_localization_node gnuplot)
target_link_libraries(relative_localization_node ${LIBS_LINK}
  ${catkin_LIBRARIES} 
)


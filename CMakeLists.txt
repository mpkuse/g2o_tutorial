cmake_minimum_required(VERSION 3.12)
project(my_g2o_project)
set(CMAKE_CXX_STANDARD 17)


#### Eigen 
find_package(Eigen3 REQUIRED)
message( ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

#### g2o 
# Lib paths 
#list(APPEND CMAKE_MODULE_PATH "/root/Downloads/g2o/cmake_modules/")
#find_package(G2O REQUIRED)
#message( ${G2O_CORE_LIBRARY})
#message( ${G2O_STUFF_LIBRARY})
#message( ${G2O_INCLUDE_DIR})
#message( ${G2O_SOLVERS_FOUND})

set(G2O_INSTALLATION /root/custom_libs_installation )
set(G2O_INCLUDE_DIR ${G2O_INSTALLATION}/include)
set(G2O_LIBS_DIR ${G2O_INSTALLATION}/lib)
include_directories(${G2O_INCLUDE_DIR})
#LINK_DIRECTORIES( ${G2O_LIBS_DIR} )
link_directories( ${G2O_LIBS_DIR} )




add_executable(simple_optimize
  src/simple_optimize.cpp
)

target_link_libraries( simple_optimize g2o_core g2o_stuff g2o_types_slam2d g2o_types_slam3d g2o_solver_eigen g2o_solver_dense )


add_executable(circle_fit
  src/circle_fit.cpp
)

target_link_libraries( circle_fit g2o_core g2o_stuff g2o_types_slam2d g2o_types_slam3d g2o_solver_eigen g2o_solver_dense)
cmake_minimum_required(VERSION 3.12)
project(my_g2o_project)
set(CMAKE_CXX_STANDARD 17)

# ### Eigen
find_package(Eigen3 REQUIRED)
message(${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# ### g2o
find_package(g2o REQUIRED)


#### exec1
add_executable(simple_optimize
  src/simple_optimize.cpp
)

target_include_directories(simple_optimize PRIVATE ${G2O_INCLUDE_DIRS})
target_link_libraries(simple_optimize PRIVATE g2o_core g2o_stuff g2o_types_slam3d g2o_solver_dense fmt)


#### exec2
add_executable(circle_fit
  src/circle_fit.cpp
)

target_link_libraries(circle_fit PRIVATE g2o_core g2o_stuff g2o_types_slam3d g2o_solver_dense fmt)

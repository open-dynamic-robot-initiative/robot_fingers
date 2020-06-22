cmake_minimum_required(VERSION 3.1)
project(robot_fingers)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# stop build on first error
string(APPEND CMAKE_CXX_FLAGS " -Wfatal-errors")


find_package(catkin REQUIRED COMPONENTS
  blmc_robots
  robot_interfaces
  mpi_cmake_modules
  pybind11_catkin
  trifinger_object_tracking
  trifinger_cameras
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


catkin_python_setup()


## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES robot_fingers
  CATKIN_DEPENDS
    blmc_robots
    robot_interfaces
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_library(trifinger_platform_frontend
    src/trifinger_platform_frontend.cpp
)
target_link_libraries(trifinger_platform_frontend
    ${catkin_LIBRARIES}
)

# Python Bindings

# TODO move this macro to mpi_cmake_modules?  More or less the same block is
# copy-pasted all over our workspace (link libraries may differ).
macro(add_python_bindings module_name)
    pybind11_add_module(${module_name} srcpy/${module_name}.cpp)
    target_link_libraries(${module_name} PRIVATE ${catkin_LIBRARIES})
    set_target_properties(${module_name} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
    install(TARGETS ${module_name} DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
endmacro()

# FIXME use catkin_add_pybind11_module
add_python_bindings(py_one_joint)
add_python_bindings(py_two_joint)
add_python_bindings(py_real_finger)
add_python_bindings(py_trifinger)


# Demo executable
add_executable(demo_trifinger_platform src/demo_platform_frontend.cpp)
target_link_libraries(demo_trifinger_platform
    trifinger_platform_frontend
    ${OpenCV_LIBRARIES}
)


# Building documentation
build_doxygen_documentation()

# Automatic code formatting (set FORMAT_CODE=ON to reformat on build)
format_code()


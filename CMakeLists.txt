cmake_minimum_required(VERSION 2.8.3)
project(robot_fingers)

find_package(catkin REQUIRED COMPONENTS
  blmc_robots
  mpi_cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()


## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robot_fingers
  CATKIN_DEPENDS
    blmc_robots
    mpi_cmake_modules
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)


# building documentation
build_doxygen_documentation()

# automatic code formatting (set FORMAT_CODE=ON to reformat on build)
format_code()

cmake_minimum_required(VERSION 3.5)
project(robot_fingers)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# libraries need to be position independent otherwise building Python modules
# fails
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# stop build on first error
string(APPEND CMAKE_CXX_FLAGS " -Wfatal-errors -Werror=return-type")


find_package(ament_cmake REQUIRED)
find_package(pybind11 REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(blmc_drivers REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(trifinger_cameras REQUIRED)
find_package(trifinger_object_tracking REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR ${PROJECT_NAME})


###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
    ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
    Eigen3::Eigen
    yaml_utils::yaml_utils
    blmc_drivers::blmc_drivers
    robot_interfaces::robot_interfaces
)


add_library(trifinger_platform_frontend INTERFACE)
target_include_directories(trifinger_platform_frontend INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(trifinger_platform_frontend INTERFACE
    ${OpenCV_LIBRARIES}
    robot_interfaces::robot_interfaces
    # FIXME linking against whole detector, only for observation?
    trifinger_object_tracking::cube_detector
    trifinger_cameras::camera_observations
)

add_library(trifinger_platform_log INTERFACE)
target_include_directories(trifinger_platform_log INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(trifinger_platform_log INTERFACE
    trifinger_platform_frontend
)


# Python Bindings
add_pybind11_module(py_one_joint srcpy/py_one_joint.cpp
    LINK_LIBRARIES ${PROJECT_NAME}
)
add_pybind11_module(py_two_joint srcpy/py_two_joint.cpp
    LINK_LIBRARIES ${PROJECT_NAME}
)
add_pybind11_module(py_solo_eight srcpy/py_solo_eight.cpp
    LINK_LIBRARIES ${PROJECT_NAME}
)
add_pybind11_module(py_real_finger srcpy/py_real_finger.cpp
    LINK_LIBRARIES ${PROJECT_NAME}
)
add_pybind11_module(py_trifinger srcpy/py_trifinger.cpp
    LINK_LIBRARIES ${PROJECT_NAME}
        trifinger_platform_frontend
        trifinger_platform_log
)
add_pybind11_module(pybullet_drivers srcpy/pybullet_drivers.cpp
    LINK_LIBRARIES
        robot_interfaces::robot_interfaces
)
# using pybind11 types, therefore visibility needs to be hidden
set_target_properties(pybullet_drivers
    PROPERTIES CXX_VISIBILITY_PRESET hidden)



# Demo executable
add_executable(demo_trifinger_platform src/demo_platform_frontend.cpp)
target_include_directories(demo_trifinger_platform PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(demo_trifinger_platform
    trifinger_platform_frontend
)


# Installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(
    TARGETS
        trifinger_platform_frontend
        trifinger_platform_log
        demo_trifinger_platform
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install_scripts(
    demos/demo_data_logging.py
    demos/demo_fake_finger.py
    demos/demo_finger_forward_kinematics.py
    demos/demo_single_finger_position_control.py
    demos/demo_single_finger_torque_control.py
    demos/demo_simulation_driver.py
    demos/demo_simulation_object_tracker.py
    demos/demo_sparse_position_control.py
    demos/demo_trifinger.py
    demos/demo_trifinger_platform_log.py
    demos/demo_trifingeredu.py
    demos/demo_trifingerpro.py
    demos/demo_solo_eight.py

    scripts/calibrate_home_offset.py
    scripts/check_finger_position_control_error.py
    scripts/claw_crane.py
    scripts/construct_object_reset_trajectory.py
    scripts/demonstrate_trajectory.py
    scripts/evaluate_log.py
    scripts/fingeredu_endurance_test.py
    scripts/fingerpro_endurance_test.py
    scripts/joint_friction_calibration.py
    scripts/plot_logged_data.py
    scripts/plot_run_duration_log.py
    scripts/position_control_on_off.py
    scripts/solo_eight_backend.py
    scripts/print_position.py
    scripts/pybullet_backend.py
    scripts/record_and_play_trajectories.py
    scripts/replay_trajectory.py
    scripts/robot_log_dat2csv.py
    scripts/run_onejoint_tests.py
    scripts/single_finger_test.py
    scripts/trifinger_backend.py
    scripts/trifinger_data_backend.py
    scripts/trifinger_robot_backend.py
    scripts/trifingerpro_offset_calibration.py
    scripts/trifingerpro_post_submission.py
    scripts/trifingerpro_print_push_sensor.py
    scripts/trifingerpro_random.py

    DESTINATION lib/${PROJECT_NAME}
)

# install this demo _with_ file extension to avoid collision with C++ executable
# with same name
install(
    PROGRAMS demos/demo_trifinger_platform.py
    DESTINATION lib/${PROJECT_NAME}
)


# Tests
if (BUILD_TESTING)
    find_package(ament_cmake_nose REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # Python tests
    ament_add_nose_test(test_pybullet_backend_py test/test_pybullet_backend.py)

    # C++ tests
    ament_add_gtest(test_pybullet_backend
      test/test_pybullet_backend.cpp
    )
    target_include_directories(test_pybullet_backend PRIVATE include)
    target_link_libraries(test_pybullet_backend
        pybind11::pybind11
        robot_interfaces::robot_interfaces
    )
    # using pybind11 types, therefore visibility needs to be hidden
    set_target_properties(test_pybullet_backend
        PROPERTIES CXX_VISIBILITY_PRESET hidden)
    install(TARGETS test_pybullet_backend DESTINATION lib/${PROJECT_NAME})


    # Create a usefull macro to build different unit tests suits.
    macro(add_cpp_test test_name)

      set(test_target_name test_${test_name})

      # create the executable
      ament_add_gtest(${test_target_name} test/${test_target_name}.cpp)
      # Add the include dependencies.
      target_include_directories(
        ${test_target_name}
        PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
               $<INSTALL_INTERFACE:include>)
      # link the dependecies to it
      target_link_libraries(${test_target_name} ${PROJECT_NAME})
      install(TARGETS ${test_target_name} DESTINATION lib/${PROJECT_NAME})

    endmacro()

    add_cpp_test(process_action)
    add_cpp_test(n_joint_blmc_robot_driver)
    add_cpp_test(clamp)

endif()


# Building documentation
add_documentation()


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
    blmc_drivers
    robot_interfaces
    trifinger_cameras
    trifinger_object_tracking
)
ament_package()

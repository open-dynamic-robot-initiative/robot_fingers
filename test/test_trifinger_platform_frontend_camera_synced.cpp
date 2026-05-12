#include <gtest/gtest.h>

#include <real_time_tools/timer.hpp>
#include <robot_fingers/trifinger_platform_frontend.hpp>
#include <robot_interfaces/sensors/sensor_data.hpp>

namespace
{
using CameraObservation = trifinger_cameras::TriCameraObservation;
using Action = robot_interfaces::TriFingerTypes::Action;
using Observation = robot_interfaces::TriFingerTypes::Observation;
using Frontend =
    robot_fingers::T_TriFingerPlatformFrontendCameraSynced<CameraObservation>;
using RobotTimeIndex = robot_fingers::RobotTimeIndex;

Observation make_observation(double marker)
{
    Observation observation;
    observation.position[0] = marker;
    return observation;
}

CameraObservation make_camera_observation(double timestamp_ms)
{
    CameraObservation observation;
    for (auto &camera : observation.cameras)
    {
        camera.timestamp = timestamp_ms / 1000;
    }
    return observation;
}
}  // namespace

TEST(TriFingerPlatformFrontendCameraSynced, PicksClosestRobotTimestamp)
{
    auto robot_data =
        std::make_shared<robot_interfaces::TriFingerTypes::SingleProcessData>(
            10);
    auto camera_data = std::make_shared<
        robot_interfaces::SingleProcessSensorData<CameraObservation>>(10);

    Frontend frontend(robot_data, camera_data);

    robot_data->observation->append(make_observation(0.0));
    real_time_tools::Timer::sleep_ms(2);
    robot_data->observation->append(make_observation(1.0));
    real_time_tools::Timer::sleep_ms(2);
    robot_data->observation->append(make_observation(2.0));

    const auto t1 = robot_data->observation->timestamp_ms(1);
    const auto t2 = robot_data->observation->timestamp_ms(2);

    const double avg_close_to_t1 = t1 + (t2 - t1) * 0.25;
    const double avg_close_to_t2 = t1 + (t2 - t1) * 0.75;
    const double avg_after_t2 = t2 + 1000.0;

    camera_data->observation->append(make_camera_observation(avg_close_to_t1));
    camera_data->observation->append(make_camera_observation(avg_close_to_t2));
    camera_data->observation->append(make_camera_observation(avg_after_t2));

    EXPECT_DOUBLE_EQ(frontend.get_robot_observation(0).position[0], 1.0);
    EXPECT_DOUBLE_EQ(frontend.get_robot_observation(1).position[0], 2.0);
    EXPECT_DOUBLE_EQ(frontend.get_robot_observation(2).position[0], 2.0);

    // also test direct access to robot observations
    EXPECT_DOUBLE_EQ(
        frontend.robot_get_robot_observation(RobotTimeIndex(0)).position[0],
        0.0);
    EXPECT_DOUBLE_EQ(
        frontend.robot_get_robot_observation(RobotTimeIndex(1)).position[0],
        1.0);
    EXPECT_DOUBLE_EQ(
        frontend.robot_get_robot_observation(RobotTimeIndex(2)).position[0],
        2.0);

    // check timestamps
    EXPECT_DOUBLE_EQ(frontend.get_robot_timestamp_ms(0), t1);
    EXPECT_DOUBLE_EQ(frontend.get_robot_timestamp_ms(1), t2);
    EXPECT_DOUBLE_EQ(frontend.get_robot_timestamp_ms(2), t2);
    EXPECT_NE(frontend.robot_get_robot_timestamp_ms(RobotTimeIndex(0)), t1);
    EXPECT_DOUBLE_EQ(frontend.robot_get_robot_timestamp_ms(RobotTimeIndex(1)),
                     t1);
    EXPECT_DOUBLE_EQ(frontend.robot_get_robot_timestamp_ms(RobotTimeIndex(2)),
                     t2);
}

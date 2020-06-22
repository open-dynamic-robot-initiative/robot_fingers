#include <robot_fingers/trifinger_platform_frontend.hpp>
#include <opencv2/highgui.hpp>

using namespace robot_fingers;

int main()
{
    auto frontend = TriFingerPlatformFrontend();

    TriFingerPlatformFrontend::Action::Vector position;

    position << -0.2, -0.9, -1.7, -0.2, -0.9, -1.7, -0.2, -0.9, -1.7;

    while (true)
    {
        time_series::Index t;

        position[0] *= -1;
        position[3] *= -1;
        position[6] *= -1;

        auto action = TriFingerPlatformFrontend::Action::Position(position);

        for (int i = 0; i < 300; i++)
        {
            t = frontend.append_desired_action(action);
            frontend.wait_until_timeindex(t);
        }

        auto robot_observation = frontend.get_robot_observation(t);
        std::cout << "Robot[:3]: " << robot_observation.position[0] << ", "
                  << robot_observation.position[1] << ", "
                  << robot_observation.position[2] << std::endl;

        auto object_pose = frontend.get_object_pose(t);
        std::cout << "Object: " << object_pose.position[0] << ", "
                  << object_pose.position[0] << ", " << object_pose.position[0]
                  << std::endl;

        auto images = frontend.get_camera_observation(t);
        cv::imshow("camera60", images.cameras[0].image);
        cv::imshow("camera180", images.cameras[1].image);
        cv::imshow("camera300", images.cameras[2].image);
        cv::waitKey(3);

        std::cout << std::endl;
    }
}

#include <opencv2/highgui.hpp>
#include <robot_fingers/trifinger_platform_frontend.hpp>

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
        // images are RGB, need to convert to BGR for visualization with OpenCV
        cv::Mat bgr_images[3];
        for (int i = 0; i < 3; i++)
        {
            cv::cvtColor(
                images.cameras[i].image, bgr_images[i], cv::COLOR_RGB2BGR);
        }
        cv::imshow("camera60", bgr_images[0]);
        cv::imshow("camera180", bgr_images[1]);
        cv::imshow("camera300", bgr_images[2]);
        cv::waitKey(3);

        std::cout << std::endl;
    }
}

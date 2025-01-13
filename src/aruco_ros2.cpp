#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class ArucoRos2Node : public rclcpp::Node
{
public:
    ArucoRos2Node() : Node("aruco_ros2")
    {
        // Publisher for marker information
        marker_info_publisher_ = this->create_publisher<std_msgs::msg::String>("aruco_marker_info", 10);
        // Set up ArUco marker detector
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        aruco_parameters_ = cv::aruco::DetectorParameters::create();
    }

    void initialize()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing");

        // Image transport subscriber
        it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
        image_subscriber_ = it_->subscribe("/camera/color/image_raw", 1,
                                           std::bind(&ArucoRos2Node::image_callback, this, std::placeholders::_1));

        // Publisher for marker information
        marker_info_publisher_ = this->create_publisher<std_msgs::msg::String>("aruco_marker_info", 10);

        // Camera info subscriber to get intrinsic parameters
        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10, std::bind(&ArucoRos2Node::camera_info_callback, this, std::placeholders::_1));

        // TF broadcaster for publishing transforms
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Set up ArUco marker detector
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        aruco_parameters_ = cv::aruco::DetectorParameters::create();
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data()));
        RCLCPP_INFO(this->get_logger(), "Received camera info.");
        received_camera_info_ = true;
    }

    // Callback for image subscription
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (!received_camera_info_)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for camera info.");
            return;
        }

        try
        {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat &image = cv_image_ptr->image;

            // Detect ArUco markers
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
            cv::aruco::detectMarkers(image, aruco_dict_, marker_corners, marker_ids, aruco_parameters_, rejected_candidates);

            if (!marker_ids.empty())
            {
                // Draw the detected markers on the image
                cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

                // Estimate the pose of the ArUco markers (using solvePnP)
                cv::Mat rvec, tvec;
                std::vector<cv::Point3f> marker_points = {
                    {0.0f, 0.0f, 0.0f}, // Center of marker
                    {1.0f, 0.0f, 0.0f}, // Right corner
                    {0.0f, 1.0f, 0.0f}, // Top corner
                    {0.0f, 0.0f, 1.0f}  // Front corner
                };

                // Camera intrinsics (example values, you should replace with actual camera parameters)
                cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 640, 0, 320, 0, 640, 240, 0, 0, 1); // Fx, Fy, Cx, Cy
                cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);                                    // No lens distortion

                cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.1, camera_matrix_, dist_coeffs, rvec, tvec);

                // Get the position and orientation of the detected marker
                if (tvec.empty() || rvec.empty())
                    return;

                // Broadcast transform from 'map' to 'aruco_marker'
                geometry_msgs::msg::TransformStamped marker_transform;
                marker_transform.header.stamp = this->get_clock()->now();
                marker_transform.header.frame_id = "camera_rgb_optical_frame"; // Parent frame (map)
                marker_transform.child_frame_id = "aruco_marker";              // Detected marker frame
                marker_transform.transform.translation.x = tvec.at<double>(0);
                marker_transform.transform.translation.y = tvec.at<double>(1);
                marker_transform.transform.translation.z = tvec.at<double>(2);

                marker_transform.transform.rotation.x = rvec.at<double>(0);
                marker_transform.transform.rotation.y = rvec.at<double>(1);
                marker_transform.transform.rotation.z = rvec.at<double>(2);
                marker_transform.transform.rotation.w = 1.0; // Assuming no rotation (simplified)

                tf_broadcaster_->sendTransform(marker_transform);

                RCLCPP_INFO(this->get_logger(), "Published transform from map to aruco_marker");
            }

            // Display the image (for debugging)
            cv::imshow("Image with ArUco markers", image);
            cv::waitKey(1); // Necessary to update the window
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }
    // ROS 2 Publisher for ArUco marker info
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_info_publisher_;

    // Image subscriber (using image_transport)
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;

    // Camera info subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ArUco marker detector variables
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;

    // Camera intrinsics
    cv::Mat camera_matrix_;

    bool received_camera_info_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aruco_node = std::make_shared<ArucoRos2Node>();
    aruco_node->initialize();
    rclcpp::spin(aruco_node);
    rclcpp::shutdown();
    return 0;
}

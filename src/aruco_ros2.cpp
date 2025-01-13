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
        this->declare_parameter("marker_size", 0.1);
        this->declare_parameter("camera_frame", "camera_rgb_optical_frame");
        this->declare_parameter("image_topic", "/camera/color/image_raw");
        this->declare_parameter("camera_info_topic", "/camera/color/camera_info");
        marker_size_ = this->get_parameter("marker_size").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        image_topic_ = this->get_parameter("image_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();

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
        image_subscriber_ = it_->subscribe(image_topic_, 1,
                                           std::bind(&ArucoRos2Node::image_callback, this, std::placeholders::_1));

        // Publisher for marker information
        marker_info_publisher_ = this->create_publisher<std_msgs::msg::String>("aruco_marker_info", 10);

        // Camera info subscriber to get intrinsic parameters
        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 10, std::bind(&ArucoRos2Node::camera_info_callback, this, std::placeholders::_1));

        // Image publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/markers_overlay", 10);

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
        camera_distortion_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double *>(msg->d.data()));
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
                // cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

                // Estimate the pose of the ArUco markers (using solvePnP)
                cv::Mat rvec, tvec;
                std::vector<cv::Point3f> marker_points = {
                    {0.0f, 0.0f, 0.0f}, // Center of marker
                    {1.0f, 0.0f, 0.0f}, // Right corner
                    {0.0f, 1.0f, 0.0f}, // Top corner
                    {0.0f, 0.0f, 1.0f}  // Front corner
                };

                // Camera intrinsics (example values, you should replace with actual camera parameters)
                cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F); // No lens distortion

                cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs, rvec, tvec);

                // Get the position and orientation of the detected marker
                if (tvec.empty() || rvec.empty())
                    return;

                // Broadcast transform from 'map' to 'aruco_marker'
                geometry_msgs::msg::TransformStamped marker_transform;
                marker_transform.header.stamp = this->get_clock()->now();
                marker_transform.header.frame_id = camera_frame_; // Parent frame (map)
                marker_transform.child_frame_id = "aruco_marker"; // Detected marker frame
                marker_transform.transform.translation.x = tvec.at<double>(0);
                marker_transform.transform.translation.y = tvec.at<double>(1);
                marker_transform.transform.translation.z = tvec.at<double>(2);

                marker_transform.transform.rotation.x = rvec.at<double>(0);
                marker_transform.transform.rotation.y = rvec.at<double>(1);
                marker_transform.transform.rotation.z = rvec.at<double>(2);
                marker_transform.transform.rotation.w = 1.0; // Assuming no rotation (simplified)

                tf_broadcaster_->sendTransform(marker_transform);

                RCLCPP_INFO(this->get_logger(), "Published transform from map to aruco_marker");

                draw3dAxis(image, tvec, rvec, 1);
                // Convert OpenCV image back to ROS message
                auto overlay_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
                // Publish the modified image
                image_pub_->publish(*overlay_msg);
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

    void draw3dAxis(cv::Mat &Image, cv::Mat &tvec, cv::Mat &rvec, int lineSize)
    {
        float size = marker_size_ * 0.6;
        cv::Mat objectPoints(4, 3, CV_32FC1);

        // origin
        objectPoints.at<float>(0, 0) = 0;
        objectPoints.at<float>(0, 1) = 0;
        objectPoints.at<float>(0, 2) = 0;

        // (1,0,0)
        objectPoints.at<float>(1, 0) = size;
        objectPoints.at<float>(1, 1) = 0;
        objectPoints.at<float>(1, 2) = 0;

        // (0,1,0)
        objectPoints.at<float>(2, 0) = 0;
        objectPoints.at<float>(2, 1) = size;
        objectPoints.at<float>(2, 2) = 0;

        // (0,0,1)
        objectPoints.at<float>(3, 0) = 0;
        objectPoints.at<float>(3, 1) = 0;
        objectPoints.at<float>(3, 2) = size;

        std::vector<cv::Point2f> imagePoints;
        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

        cv::projectPoints(objectPoints, rvec, tvec, camera_matrix_, dist_coeffs, imagePoints);
        cv::line(Image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255, 255), lineSize);
        cv::line(Image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0, 255), lineSize);
        cv::line(Image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0, 255), lineSize);

        putText(Image, "x", imagePoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
        putText(Image, "y", imagePoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255), 2);
        putText(Image, "z", imagePoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255), 2);
    }

    // ROS 2 Publisher for ArUco marker info
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_info_publisher_;

    // Image subscriber (using image_transport)
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;

    // Camera info subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ArUco marker detector variables
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;

    // Camera intrinsics
    cv::Mat camera_matrix_;
    cv::Mat camera_distortion_;

    bool received_camera_info_ = false;
    double marker_size_;
    std::string camera_frame_;
    std::string image_topic_;
    std::string camera_info_topic_;
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

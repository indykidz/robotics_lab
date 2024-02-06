#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert image to grayscale
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    // Set up the detector with parameters
    cv::SimpleBlobDetector::Params params;
    
    params.minThreshold = 1;
    params.maxThreshold = 250;
    
    params.filterByColor = true;
    params.blobColor = 0;
    params.minRepeatability = 4;
    params.minDistBetweenBlobs = 1;
    
    params.filterByArea = true;
    params.minArea = 1500;
    params.filterByCircularity = true;
    params.minCircularity = 0.1;
    params.filterByConvexity = true;
    params.minConvexity = 0.87;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    // Set up detector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray, keypoints);

    // Draw blobs on the image
    cv::Mat im_with_keypoints;
    cv::drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, im_with_keypoints);
    cv::waitKey(1);

    // Publish the modified image
    sensor_msgs::ImagePtr output_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_with_keypoints).toImageMsg();
    image_pub_.publish(output_image_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


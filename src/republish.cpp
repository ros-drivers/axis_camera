
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub_;
int flags_ = CV_LOAD_IMAGE_UNCHANGED;


void image_callback(const sensor_msgs::CompressedImageConstPtr& message)

{
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    // Copy message header
    cv_ptr->header = message->header;

    // Decode color/mono image
    try {
        cv_ptr->image = cv::imdecode(cv::Mat(message->data), flags_);

        switch (cv_ptr->image.channels()) {
            case 1:
                cv_ptr->encoding = enc::MONO8;
                break;
            case 3:
                cv_ptr->encoding = enc::BGR8;
                break;
            default:
                ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
                break;
        }
        size_t rows = cv_ptr->image.rows;
        size_t cols = cv_ptr->image.cols;

        if ((rows > 0) && (cols > 0)) {
            pub_.publish(cv_ptr->toImageMsg());
        }
    } catch (cv::Exception& e) {
        ROS_ERROR("%s", e.what());
    }

}


int main(int argc, char *argv[]) 
{
    std::string mode("unchanged");
    ros::init(argc,argv,"republish");
    ros::NodeHandle nh("~");
    nh.param("mode",mode,mode);
    if ((mode == "gray") || (mode == "grey")) {
        flags_ = CV_LOAD_IMAGE_GRAYSCALE;
    } else if ((mode == "color") || (mode == "colour")) {
        flags_ = CV_LOAD_IMAGE_COLOR;
    } else /*if (mode == "unchanged")*/ {
        flags_ = CV_LOAD_IMAGE_UNCHANGED;
    } 
    ros::Subscriber sub = nh.subscribe("compressed",1,image_callback);
    pub_ = nh.advertise<sensor_msgs::Image>("out",1);
    ros::spin();
    return 0;
}

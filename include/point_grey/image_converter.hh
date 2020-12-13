#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <point_grey/PointGray.h>
#include <ios>
#include <fstream>
#include <string>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
   private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_mono_;
    ros::ServiceServer point_grey_srv_;
    std::string file_path_;

    sensor_msgs::ImageConstPtr msg_image;
    sensor_msgs::ImageConstPtr msg_image_mono;

      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> PointGreyPolicy;
      typedef message_filters::Synchronizer<PointGreyPolicy> Sync;
      boost::shared_ptr<Sync> sync_;

     public:
      ImageConverter();
      ~ImageConverter();
      void initSubscriber(ros::NodeHandle& nh);
      void initServices(ros::NodeHandle& nh);
      void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msg_mono);

      bool serviceCB(point_grey::PointGray::Request& req,
                     point_grey::PointGray::Response& res);
};

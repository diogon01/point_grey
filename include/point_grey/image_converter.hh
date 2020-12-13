#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <point_grey/PointGray.h>
#include <point_grey/ListFolders.h>
#include <ios>
#include <fstream>
#include <string>
#include <filesystem>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
   private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_position_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_position_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    ros::ServiceServer point_grey_srv_;
    ros::ServiceServer list_folder_srv_;
    std::string file_path_;

    sensor_msgs::ImageConstPtr ptr_image_left_;
    sensor_msgs::ImageConstPtr ptr_image_right_;
    sensor_msgs::NavSatFixConstPtr ptr_gps_position_;
    sensor_msgs::NavSatFixConstPtr ptr_rtk_position_;
    sensor_msgs::ImuConstPtr ptr_imu_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                            sensor_msgs::NavSatFix, sensor_msgs::NavSatFix, sensor_msgs::Imu>
        PointGreyPolicy;
    typedef message_filters::Synchronizer<PointGreyPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

   public:
    ImageConverter();
    ~ImageConverter();
    void initSubscriber(ros::NodeHandle& nh);
    void initServices(ros::NodeHandle& nh);
    void imageCb(const sensor_msgs::ImageConstPtr& msg_left_image,
                 const sensor_msgs::ImageConstPtr& msg_right_image, const sensor_msgs::NavSatFixConstPtr& msg_gps,
                 const sensor_msgs::NavSatFixConstPtr& msg_rtk, const sensor_msgs::ImuConstPtr& msg_imu);

    bool serviceCB(point_grey::PointGray::Request& req,
                   point_grey::PointGray::Response& res);

    bool serviceListFolders(point_grey::ListFolders::Request& req,
                   point_grey::ListFolders::Response& res);

    std::vector<std::string> get_directories(const std::string& s);
};

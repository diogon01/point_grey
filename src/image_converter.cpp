#include <image_converter.hh>

ImageConverter::ImageConverter() {

    initSubscriber(nh_);
    initServices(nh_);
}

ImageConverter::~ImageConverter() {
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msg_mono) {
    cv_bridge::CvImagePtr cv_ptr;
    msg_image = msg;
    msg_image_mono = msg_mono;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

  /*   // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg()); */
}

bool ImageConverter::serviceCB(point_grey::PointGray::Request& req,
                               point_grey::PointGray::Response& res) {
    cv_bridge::CvImagePtr cv_ptr;
    file_path_ = req.file_path;

    try {
        std::ofstream file;
        file.open(file_path_, std::ios::out | std::ios::app);
        if (file.fail()) {
            res.result = false;
            //throw std::ios_base::failure(std::strerror(errno));
        }
        cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);

        cv::imwrite("teste.png", cv_ptr->image);;
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        res.result = false;
        return res.result;
    }

    res.result = true;
    return res.result;
}

void ImageConverter::initServices(ros::NodeHandle& nh) {
    try{
        point_grey_srv_ = nh_.advertiseService("point_grey/take_picture", &ImageConverter::serviceCB, this);
        ROS_INFO("Service point_grey/take_picture initialize");
    }catch(ros::Exception &e){
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }

}

void ImageConverter::initSubscriber(ros::NodeHandle& nh) {
     try{
        image_sub_.subscribe(nh_, "/stereo/left/image_mono", 1);
        image_mono_.subscribe(nh_, "/stereo/right/image_mono", 1);
        sync_.reset(new Sync(PointGreyPolicy(10), image_sub_, image_mono_));
        sync_->registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));

        ROS_INFO("Subscribe complet");
    }catch(ros::Exception &e){
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

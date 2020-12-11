#include <image_converter.hh>

ImageConverter::ImageConverter() : it_(nh_),
                                   image_sub_(nh_, "/camera/image_raw", 1),
                                   image_mono_(nh_, "/camera/image_mono", 1),
                                   sync_(PontGreyPolicy(100), image_sub_, image_mono_) {
    // Subscrive to input video feed and publish output video feed
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    point_grey_srv_ = nh_.advertiseService("point_grey/take_picture", &ImageConverter::serviceCB, this);

    sync_.registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));

    cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
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

bool ImageConverter::initSubscriber(ros::NodeHandle& nh) {
    image_sub_.subscribe(nh, "/camera/image_raw", 1);
    image_mono_.subscribe(nh, "/camera/image_mono", 1);
    
    sync_2.reset(new Sync(PontGreyPolicy(100), image_sub_, image_mono_));
    sync_2->registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));
    return true;
}

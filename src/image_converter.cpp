#include <image_converter.hh>

ImageConverter::ImageConverter() {
    initSubscriber(nh_);
    initServices(nh_);
}

ImageConverter::~ImageConverter() {
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg_left_image,
                             const sensor_msgs::ImageConstPtr& msg_right_image, const sensor_msgs::NavSatFixConstPtr& msg_gps,
                             const sensor_msgs::NavSatFixConstPtr& msg_rtk, const sensor_msgs::ImuConstPtr& msg_imu) {
    ptr_image_left_ = msg_left_image;
    ptr_image_right_ = msg_right_image;
    ptr_gps_position_ = msg_gps;
    ptr_rtk_position_ = msg_rtk;
    ptr_imu_ = msg_imu;
}

bool ImageConverter::serviceCB(point_grey::PointGray::Request& req,
                               point_grey::PointGray::Response& res) {
    cv_bridge::CvImagePtr cv_ptr_L;
    cv_bridge::CvImagePtr cv_ptr_R;
    file_path_ = req.file_path;
    std::string file_name_ = req.file_name;
    counter_ = req.reset_counter ? 0 : counter_;

    try {
        std::ofstream file;
        file.open(file_path_ + file_name_, std::ios::out | std::ios::app);
        if (file.fail()) {
            res.result = false;
            //throw std::ios_base::failure(std::strerror(errno));
        }
        cv_ptr_L = cv_bridge::toCvCopy(ptr_image_left_, sensor_msgs::image_encodings::BGR8);
        cv_ptr_R = cv_bridge::toCvCopy(ptr_image_right_, sensor_msgs::image_encodings::BGR8);

        cv::imwrite(file_path_ + "image_left_" + std::to_string(counter_) + ".png", cv_ptr_L->image);
        cv::imwrite(file_path_ + "image_right_" + std::to_string(counter_) + ".png", cv_ptr_R->image);
        // Update GUI Window

        file << "image_left_" + std::to_string(counter_) + ".png"
             << "\t"
             << "image_right_" + std::to_string(counter_) + ".png"
             << "\t"
             << ptr_gps_position_->longitude << "\t"
             << ptr_gps_position_->latitude << "\t"
             << ptr_gps_position_->altitude << "\t"
             << ptr_rtk_position_->longitude << "\t"
             << ptr_rtk_position_->latitude << "\t"
             << ptr_rtk_position_->altitude << "\t"
             << std::endl;

        file.close();
        counter_++;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        res.result = false;
        return res.result;
    }

    res.result = true;
    return res.result;
}

bool ImageConverter::serviceListFolders(point_grey::ListFolders::Request& req,
                                        point_grey::ListFolders::Response& res) {
    std::vector<std::string> r;
    for (auto& p : std::filesystem::recursive_directory_iterator(req.file_path))
        if (p.is_directory())
            r.push_back(p.path().string());

    res.directory_names = r;
    res.result = true;
    return true;
}

void ImageConverter::initServices(ros::NodeHandle& nh) {
    try {
        point_grey_srv_ = nh_.advertiseService("point_grey/take_picture", &ImageConverter::serviceCB, this);
        ROS_INFO("Service point_grey/take_picture initialize");
        list_folder_srv_ = nh_.advertiseService("point_grey/list_folders", &ImageConverter::serviceListFolders, this);
        ROS_INFO("Service point_grey/list_folders initialize");

    } catch (ros::Exception& e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void ImageConverter::initSubscriber(ros::NodeHandle& nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string left_topic_, right_topic_, gps_topic_, rtk_topic_, imu_topic_;
        nh_private.param("camera_left_topic", left_topic_, std::string("/stereo/left/image_raw"));
        nh_private.param("camera_right_topic", right_topic_, std::string("/stereo/right/image_raw"));
        nh_private.param("gps_topic", gps_topic_, std::string("/dji_sdk/gps_position"));
        nh_private.param("rtk_topic", rtk_topic_, std::string("/dji_sdk/rtk_position"));
        nh_private.param("imu_topic", imu_topic_, std::string("/dji_sdk/imu"));

        image_left_sub_.subscribe(nh, left_topic_, 1);
        ROS_INFO("Subscriber in Camera Left Topic: %s", left_topic_.c_str());
        image_right_sub_.subscribe(nh, right_topic_, 1);
        ROS_INFO("Subscriber in Camera Right Topic: %s", right_topic_.c_str());
        gps_position_sub_.subscribe(nh, gps_topic_, 1);
        ROS_INFO("Subscriber in Camera GPS Topic: %s", gps_topic_.c_str());
        rtk_position_sub_.subscribe(nh, rtk_topic_, 1);
        ROS_INFO("Subscriber in Camera RTK Topic: %s", rtk_topic_.c_str());
        imu_sub_.subscribe(nh, imu_topic_, 1);
        ROS_INFO("Subscriber in Camera IMU Topic: %s", imu_topic_.c_str());

        sync_.reset(new Sync(PointGreyPolicy(10), image_left_sub_, image_right_sub_, gps_position_sub_, rtk_position_sub_, imu_sub_));
        sync_->registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2, _3, _4, _5));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception& e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

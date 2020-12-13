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

    try {
        std::ofstream file;
        file.open(file_path_ + file_name_, std::ios::out | std::ios::app);
        if (file.fail()) {
            res.result = false;
            //throw std::ios_base::failure(std::strerror(errno));
        }
        cv_ptr_L = cv_bridge::toCvCopy(ptr_image_left_, sensor_msgs::image_encodings::BGR8);
        cv_ptr_R = cv_bridge::toCvCopy(ptr_image_right_, sensor_msgs::image_encodings::BGR8);

        file << "something you want to add to your outputfile" << std::endl;

        cv::imwrite(file_path_ + "_image_left.png", cv_ptr_L->image);
        cv::imwrite(file_path_ + "_image_right.png", cv_ptr_L->image);
        // Update GUI Window
        cv::waitKey(3);

        file.close();

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
    for(auto& p : std::filesystem::recursive_directory_iterator(req.file_path))
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
        image_left_sub_.subscribe(nh, "/stereo/left/image_mono", 1);
        image_right_sub_.subscribe(nh, "/stereo/right/image_mono", 1);
        gps_position_sub_.subscribe(nh, "/dji_sdk/gps_position", 1);
        rtk_position_sub_.subscribe(nh, "/dji_sdk/rtk_position", 1);
        imu_sub_.subscribe(nh, "/dji_sdk/imu", 1);

        sync_.reset(new Sync(PointGreyPolicy(10), image_left_sub_, image_right_sub_, gps_position_sub_, rtk_position_sub_, imu_sub_));
        sync_->registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2, _3, _4, _5));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception& e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

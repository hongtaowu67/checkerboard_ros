#ifndef CAPTURE_H
#define CAPTURE_H

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/calib3d.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

using std::vector;

class CapturePoseCheckerboard
{
    public:
        CapturePoseCheckerboard(const string& check_pose_topic_input, 
                                const string& pose_result_topic_input,
                                const string& rgb_topic_input, 
                                const string& rgb_cam_info_topic_input,
                                const bool& use_rect_input)
        {
            check_pose_topic = check_pose_topic_input;
            pose_result_topic = pose_result_topic_input;
            rgb_topic = rgb_topic_input;
            rgb_cam_info_topic = rgb_cam_info_topic_input;

            use_rect = use_rect_input;
            
            ROS_INFO("##### Publisher #####\n");
            ROS_INFO("checkerboard pose topic: %s", check_pose_topic.c_str());
            ROS_INFO("pose result topic: %s", pose_result_topic.c_str());
            ROS_INFO("##### Subscriber #####\n");
            ROS_INFO("use_rect: %s", use_rect ? "true" : "false");
            ROS_INFO("rgb topic: %s", rgb_topic.c_str());
            ROS_INFO("rgb camera_info topic: %s", rgb_cam_info_topic.c_str());
            
            patternsize = Size(col_num, row_num);

            object_corners.clear();
            for( int i=0; i<row_num; i++ )
                for( int j=0; j<col_num; j++ )
                    object_corners.push_back( Point3f( float(i * square_size), float(j * square_size), 0.0f ) );
            
            image_transport::ImageTransport it(nh); 
            check_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ( check_pose_topic, 1 );
            pose_result_pub = it.advertise ( pose_result_topic, 1 );

            rgb_sub.subscribe( nh, rgb_topic, 1 );
            info_sub.subscribe( nh, rgb_cam_info_topic, 1);

            // Sleep for a second
            ros::Duration(1.0).sleep();

            sync_ptr.reset(new Sync( MySyncPolicy(10), rgb_sub, info_sub ) ); 
            sync_ptr->registerCallback( boost::bind( &CapturePoseCheckerboard::callBack, this, _1, _2) );
            
        }; // Constructor

        // Camera model for camera_info
        void setCameraInfo( const CameraInfoConstPtr& cam_info );
        // Callback function for the subscriber
        void callBack( const ImageConstPtr& rgb, const CameraInfoConstPtr& cam_info );


    private:
        ros::NodeHandle nh; // Handle
        bool use_rect = use_rect; // true if the rgb is rectified; false if the rgb is not rectified

        // Topic name
        string check_pose_topic;
        string pose_result_topic;
        string rgb_topic;
        string rgb_cam_info_topic;

        // Checkerboard parameter
        int col_num = 7;
        int row_num = 6;
        double square_size = 0.0256; // unit: meter
        cv::Size patternsize;

        // Subscriber
        message_filters::Subscriber<Image> rgb_sub;
        message_filters::Subscriber<CameraInfo> info_sub;
        
        // Syncing the subscriber
        typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_ptr;
        
        // cv_bridge
        cv_bridge::CvImagePtr input_bridge;
        cv::Mat image_grey;
        cv::Mat image_rgb;

        std::vector<cv::Point2f> image_corners;
        std::vector<cv::Point3f> object_corners;
        
        image_geometry::PinholeCameraModel cam_model;
        cv::Mat_<double> intrinsic_matrix = cv::Mat_<double>::eye (4, 4);
        cv::Mat dist_coeff;
        cv::Mat_<double> extrinsic_matrix = cv::Mat_<double>::eye (4, 4);
        cv::Mat_<double> projection_matrix;
        cv::Mat_<double> cam_pose_matrix = cv::Mat_<double>::eye (4, 4);

        // Publisher for publishing the checkerboard frame in the camera frame
        ros::Publisher check_pose_pub;
        cv::Vec3d rotation_vec;
        cv::Vec3d translation_vec;
        tf::Transform transform_check;
        geometry_msgs::PoseStamped pose_check;
        tf::Quaternion q_check;
        tf::Matrix3x3 R_check;
        tf::Vector3 t_check;

        // Publisher for the publishing the camera pose in the checkerboard frame
        ros::Publisher cam_pose_pub;
        tf::Transform transform_cam;
        geometry_msgs::PoseStamped pose_cam;
        tf::Quaternion q_cam;
        tf::Matrix3x3 R_cam;
        tf::Vector3 t_cam;

        // Publisher for publishing the pose estimation result
        image_transport::Publisher pose_result_pub;
        sensor_msgs::Image pose_result;
        sensor_msgs::ImagePtr pose_result_msg;

        bool adaptive_thresh = true;
        bool normalize_image = false;
        bool filter_quads = false;
        bool fast_check = false;

        // Display image
        int subpixelfit_window_size = 8;

};

#endif //CAPTURE_H

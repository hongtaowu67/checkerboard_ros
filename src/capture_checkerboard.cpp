#include <iostream>
#include <string>
#include <fstream>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "capture_checkerboard.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

using std::vector;

void CapturePoseCheckerboard::setCameraInfo(const CameraInfoConstPtr& cam_info)
{

    if (use_rect) // rgb is rectified
    {
        intrinsic_matrix.at<double>(0,0) = cam_info->P[0];   intrinsic_matrix.at<double>(0,1) = cam_info->P[1];   intrinsic_matrix.at<double>(0,2) = cam_info->P[2];
        intrinsic_matrix.at<double>(1,0) = cam_info->P[4];   intrinsic_matrix.at<double>(1,1) = cam_info->P[5];   intrinsic_matrix.at<double>(1,2) = cam_info->P[6];
        intrinsic_matrix.at<double>(2,0) = cam_info->P[8];   intrinsic_matrix.at<double>(2,1) = cam_info->P[9];   intrinsic_matrix.at<double>(2,2) = cam_info->P[10];

        dist_coeff = Mat::zeros ( 1,5,CV_32F );
    }
    else // rgb is not raw
    {

        intrinsic_matrix.at<double>(0,0) = cam_info->K[0];   intrinsic_matrix.at<double>(0,1) = cam_info->K[1];   intrinsic_matrix.at<double>(0,2) = cam_info->K[2];
        intrinsic_matrix.at<double>(1,0) = cam_info->K[3];   intrinsic_matrix.at<double>(1,1) = cam_info->K[4];   intrinsic_matrix.at<double>(1,2) = cam_info->K[5];
        intrinsic_matrix.at<double>(2,0) = cam_info->K[6];   intrinsic_matrix.at<double>(2,1) = cam_info->K[7];   intrinsic_matrix.at<double>(2,2) = cam_info->K[8];

        cam_model.fromCameraInfo ( cam_info );
        dist_coeff = Mat ( cam_model.distortionCoeffs() );
    }

}

void CapturePoseCheckerboard::callBack( const ImageConstPtr& rgb, const CameraInfoConstPtr& cam_info )
{
    if (ros::ok())
    {
        setCameraInfo(cam_info); // set camera_info
        
        input_bridge = cv_bridge::toCvCopy( rgb, image_encodings::MONO8 );
        image_grey = input_bridge->image;

        int flags = 0;
        if ( adaptive_thresh ) flags += CV_CALIB_CB_ADAPTIVE_THRESH;
        if ( normalize_image ) flags += CV_CALIB_CB_NORMALIZE_IMAGE;
        if ( filter_quads ) flags += CV_CALIB_CB_FILTER_QUADS;
        if ( fast_check ) flags += CALIB_CB_FAST_CHECK;
        bool patternfound = findChessboardCorners ( image_grey, patternsize, image_corners, flags );

        if ( patternfound )
        {
            cv::cornerSubPix( image_grey, image_corners, Size(subpixelfit_window_size, subpixelfit_window_size), 
                            Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            cv::solvePnP( object_corners, image_corners, cv::Mat( intrinsic_matrix, cv::Rect(0, 0, 3, 3) ), dist_coeff, rotation_vec, translation_vec );

            extrinsic_matrix = cv::Mat_<double>::eye (4, 4);
            extrinsic_matrix(0, 3) = translation_vec(0);
            extrinsic_matrix(1, 3) = translation_vec(1);
            extrinsic_matrix(2, 3) = translation_vec(2);

            Rodrigues ( rotation_vec, cv::Mat( extrinsic_matrix, cv::Rect(0, 0, 3, 3)), noArray() );
            projection_matrix = intrinsic_matrix * extrinsic_matrix;

            // generate tf model to camera
            R_check = tf::Matrix3x3  ( extrinsic_matrix( 0, 0 ), extrinsic_matrix( 0, 1 ), extrinsic_matrix( 0, 2 ),
                                    extrinsic_matrix( 1, 0 ), extrinsic_matrix( 1, 1 ), extrinsic_matrix( 1, 2 ),
                                    extrinsic_matrix( 2, 0 ), extrinsic_matrix( 2, 1 ), extrinsic_matrix( 2, 2 ) );

            t_check = tf::Vector3  ( translation_vec ( 0 ), translation_vec ( 1 ), translation_vec ( 2 ) );
            transform_check =  tf::Transform ( R_check, t_check );
            q_check = transform_check.getRotation();

            // Publish check pose
            pose_check.header = rgb->header;
            pose_check.pose.orientation.x = q_check.x();
            pose_check.pose.orientation.y = q_check.y();
            pose_check.pose.orientation.z = q_check.z();
            pose_check.pose.orientation.w = q_check.w();
            pose_check.pose.position.x = t_check.x();
            pose_check.pose.position.y = t_check.y();
            pose_check.pose.position.z = t_check.z();
            check_pose_pub.publish(pose_check);

            // Deubgging the output rotation matrix of the cam pose
            // std::cout << "Cam Rotation" << std::endl << R_cam[0][0] << " " << R_cam[0][1] << " " << R_cam[0][2] << std::endl
            //                                          << R_cam[1][0] << " " << R_cam[1][1] << " " << R_cam[1][2] << std::endl
            //                                          << R_cam[2][0] << " " << R_cam[2][1] << " " << R_cam[2][2] << std::endl;
            // std::cout << "Cam Translation" << std::endl << t_cam.x() << " " << t_cam.y() << " " << t_cam.z() << std::endl; 


            double nr_of_square =  std::max ( col_num, row_num );
            double size =  square_size * nr_of_square;

            int font = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 1.0;
            double thickness = 1.0;
            double lineType = CV_AA;
            double lineThickness = 3;

            input_bridge = cv_bridge::toCvCopy( rgb, image_encodings::BGR8 );
            ROS_INFO("Finish converting to BGR...\n");
            image_rgb = input_bridge->image;

            cv::Mat_<double> Pi0 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << 0, 0, 0, 1 );
            cv::Point2d pi0 ( Pi0 ( 0,0 ) / Pi0 ( 0,2 ), Pi0 ( 0,1 ) / Pi0 ( 0,2 ) );
            cv::circle ( image_rgb, pi0, 3, CV_RGB ( 255,255,255 ) );

            cv::Mat_<double> Pi1 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << size, 0, 0, 1 );;
            cv::Point2d pi1 ( Pi1 ( 0,0 ) / Pi1 ( 0,2 ), Pi1 ( 0,1 ) / Pi1 ( 0,2 ) );
            cv::circle ( image_rgb, pi1, 3, CV_RGB ( 255,0,0 ) );
            putText ( image_rgb, "X", pi1, font, fontScale, CV_RGB ( 255,0,0 ), thickness, CV_AA );
            cv::line ( image_rgb, pi0, pi1, CV_RGB ( 255,0,0 ), lineThickness );

            cv::Mat_<double> Pi2 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << 0, size, 0, 1 );
            cv::Point2d pi2 ( Pi2 ( 0,0 ) / Pi2 ( 0,2 ), Pi2 ( 0,1 ) / Pi2 ( 0,2 ) );
            cv::circle ( image_rgb, pi2, 3, CV_RGB ( 0,255,0 ) );
            putText ( image_rgb, "Y", pi2, font, fontScale, CV_RGB ( 0,255,0 ), thickness, CV_AA );
            cv::line ( image_rgb, pi0, pi2, CV_RGB ( 0,255,0 ), lineThickness );

            cv::Mat_<double> Pi3 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << 0, 0, size, 1 );
            cv::Point2d pi3 ( Pi3 ( 0,0 ) / Pi3 ( 0,2 ), Pi3 ( 0,1 ) / Pi3 ( 0,2 ) );
            cv::circle ( image_rgb, pi3, 3, CV_RGB ( 0,0,255 ) );
            putText ( image_rgb, "Z", pi3, font, fontScale, CV_RGB ( 0,0,255 ) , thickness, CV_AA );
            cv::line ( image_rgb, pi0, pi3, CV_RGB ( 0,0,255 ), lineThickness );

            drawChessboardCorners ( image_rgb, patternsize, Mat ( image_corners ), patternfound );

            // Publish pose estimation result
            pose_result_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rgb).toImageMsg();
            pose_result_msg->header = rgb->header;
            pose_result_pub.publish(pose_result_msg);
        }
        else
        {
            ROS_INFO("Cannot find all the corners of the chessboard...");
        }
    }
}



int main( int argc, char** argv )
{
    ros::init( argc, argv, "capture" );

    string check_pose_topic = "/capture/pose_check";
    string pose_result_topic = "/capture/pose_result";
    string rgb_topic = "/camera/rgb/image_rect_color";
    string rgb_cam_info_topic = "/camera/rgb/camera_info";
    bool use_rect = true;

    CapturePoseCheckerboard CPC(check_pose_topic,
                                pose_result_topic,
                                rgb_topic, 
                                rgb_cam_info_topic,
                                use_rect);
    
    ros::spin();
    return 0;
}
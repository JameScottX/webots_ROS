#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include "orb3_odom.hpp"

using namespace std;

static vector<double>odom_xyzrpy;

class ImageGrabber
{

public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
};



int main(int argc, char **argv)
{   

    odom_xyzrpy.resize(6);

    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/mob/camera/image", 1, &ImageGrabber::GrabImage,&igb);

    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);
    // ImageGrabber igb(&SLAM);
    // ros::NodeHandle nh;
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    // sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}



void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    static geometry_msgs::Quaternion  real_q;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame =  mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    if (!frame.empty()){
        
        Eigen::Matrix4f frame_e;
        cv::cv2eigen(frame,frame_e);
        Eigen::Matrix4f Turn;

        Turn << 0,0,-1,0,
                1,0,0,0,
                0,1,0,0,
                0,0,0,1;

        Eigen::Matrix4f new_frame = Turn*frame_e;

        
        odom_xyzrpy[0] = 2*new_frame(0,3); 
        odom_xyzrpy[1] = 2*new_frame(1,3);
        odom_xyzrpy[2] = 2*new_frame(2,3);
        ROS_INFO("%f, %f, %f ", 2*new_frame(0,3), 2*new_frame(1,3), 2*new_frame(2,3));

        tf::Matrix3x3 rotat;
        tfScalar xx = tfScalar(new_frame(0,0));
        tfScalar xy = tfScalar(new_frame(0,1));
        tfScalar xz = tfScalar(new_frame(0,2));
        tfScalar yx = tfScalar(new_frame(1,0));
        tfScalar yy = tfScalar(new_frame(1,1));
        tfScalar yz = tfScalar(new_frame(1,2));
        tfScalar zx = tfScalar(new_frame(2,0));
        tfScalar zy = tfScalar(new_frame(2,1));
        tfScalar zz = tfScalar(new_frame(2,2));
        rotat.setValue( xx,xy,xz,yx,yy,yz,zx,zy,zz);

        tfScalar roll=0., pitch=0., yaw=0.;
        rotat.getRPY(pitch, roll, yaw); //转化到 RPY

        ROS_INFO("%f, %f, %f ", roll, pitch-1.5708, -yaw+1.5708);

        tf::Quaternion q;
        q.setEulerZYX(-yaw+1.5708 , pitch-1.5708, roll);  //角度变换

        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(odom_xyzrpy[0],odom_xyzrpy[1],odom_xyzrpy[2]));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
        transform.setOrigin(tf::Vector3(0.,0.,0.));
        transform.setIdentity();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "/te/lidar"));

        transform.setOrigin(tf::Vector3(0.,0.,0.));
        transform.setIdentity();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
    }
}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    static geometry_msgs::Quaternion  real_q;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (!frame.empty()){
        
        Eigen::Matrix4f frame_e;
        cv::cv2eigen(frame,frame_e);
        Eigen::Matrix4f Turn;

        Turn << 0,0,-1,0,
                1,0,0,0,
                0,1,0,0,
                0,0,0,1;

        Eigen::Matrix4f new_frame = Turn*frame_e;
        
        odom_xyzrpy[0] = new_frame(0,3); 
        odom_xyzrpy[1] = new_frame(1,3);
        odom_xyzrpy[2] = new_frame(2,3);
        ROS_INFO("%f, %f, %f ", new_frame(0,3), new_frame(1,3), new_frame(2,3));

        tf::Matrix3x3 rotat;
        tfScalar xx = tfScalar(new_frame(0,0));
        tfScalar xy = tfScalar(new_frame(0,1));
        tfScalar xz = tfScalar(new_frame(0,2));
        tfScalar yx = tfScalar(new_frame(1,0));
        tfScalar yy = tfScalar(new_frame(1,1));
        tfScalar yz = tfScalar(new_frame(1,2));
        tfScalar zx = tfScalar(new_frame(2,0));
        tfScalar zy = tfScalar(new_frame(2,1));
        tfScalar zz = tfScalar(new_frame(2,2));
        rotat.setValue( xx,xy,xz,yx,yy,yz,zx,zy,zz);

        tfScalar roll=0., pitch=0., yaw=0.;
        rotat.getRPY(pitch, roll, yaw); //转化到 RPY

        ROS_INFO("%f, %f, %f ", roll, pitch-1.5708, -yaw+1.5708);

        tf::Quaternion q;
        q.setEulerZYX(-yaw+1.5708 , pitch-1.5708, roll);  //角度变换

        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(odom_xyzrpy[0],odom_xyzrpy[1],odom_xyzrpy[2]));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
        transform.setOrigin(tf::Vector3(0.,0.,0.));
        transform.setIdentity();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "/te/lidar"));

        transform.setOrigin(tf::Vector3(0.,0.,0.));
        transform.setIdentity();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
    }
}







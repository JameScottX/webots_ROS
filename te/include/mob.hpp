#include "ros/ros.h"
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

static double GPSValues[3] = {0, 0, 0};
static double inertialUnitValues[4] = {0, 0, 0, 0};
static geometry_msgs::Quaternion  real_q;


void __Order_Float(ros::NodeHandle *n, float val, std::string name ){

    ros::ServiceClient SC;
    webots_ros::set_float set_srv;
    SC = n->serviceClient<webots_ros::set_float>(name);
    set_srv.request.value = val;
    if (SC.call(set_srv) && set_srv.response.success)
    ;
    else
      ROS_ERROR("Failed to call service %s! ", name.c_str());
}



void __Order_Init_Sensor( ros::NodeHandle *n, int val, std::string name){

    ros::ServiceClient SC;
    webots_ros::set_int set_srv;
    SC = n->serviceClient<webots_ros::set_int>(name);
    set_srv.request.value = val;
    if (SC.call(set_srv) && set_srv.response.success) {
      ROS_INFO("%s is ok! ",name.c_str());
    } else {
      if (!set_srv.response.success)
        ROS_ERROR("Failed to enable %s. ", name.c_str());
    }
}

void broadcastTransform() {

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(-GPSValues[2], -GPSValues[0], 0));

  tf::Quaternion q(real_q.x, real_q.y,real_q.z,real_q.w);

  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  transform.setOrigin(tf::Vector3(0,0,0.));
  transform.setIdentity();
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "mob/lidar"));
}


void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values){

  GPSValues[0] = values->latitude;
  GPSValues[1] = values->altitude;
  GPSValues[2] = values->longitude;
  broadcastTransform();

  // ROS_INFO("%f, %f ", GPSValues[0],GPSValues[1]);
}

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {

  double roll=0., pitch=0., yaw=0.;

  tf::Quaternion q;
  tf::quaternionMsgToTF(values->orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // ROS_INFO("%f, %f, %f ", -pitch, -roll, yaw);
  real_q = tf::createQuaternionMsgFromRollPitchYaw(-pitch, -roll, yaw);

  broadcastTransform();

}


void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {

  double temp = scan->angle_min;
  // ROS_INFO("%f  ", scan->ranges[0]);

}


#include "std_msgs/String.h"
#include <sstream>
#include <signal.h>
#include "mob.hpp"


#define TIME_STEP 20
#define NMOTORS 4
static const char *motorNames[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};


int controllerCount =0;
ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;
static std::vector<std::string> controllerList;

void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}


void quit(int sig) {
  ROS_INFO("User stopped the 'pioneer3at' node.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
    std::string controllerName;
    ros::init(argc,argv,"mob",ros::init_options::AnonymousName);

    
    ros::NodeHandle n;

    signal(SIGINT, quit);
    //寻找模型名称
    ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
      ros::spinOnce();
      ros::spinOnce();
      ros::spinOnce();
    }
    ros::spinOnce();
    

    timeStepClient = n.serviceClient<webots_ros::set_int>("mob/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    if (controllerCount == 1)
      controllerName = controllerList[0];
    else {
      int wantedController = 0;
      std::cout << "Choose the # of the controller you want to use:\n";
      std::cin >> wantedController;
      if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
      else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
      }
    }

  ROS_INFO("Using controller: '%s'", controllerName.c_str());
  nameSub.shutdown();


  //初始化电机
    for (int i = 0; i < NMOTORS; ++i) {
      // position
      
      __Order_Float(&n,INFINITY, 
        std::string("mob/") + std::string(motorNames[i]) + std::string("/set_position"));

      __Order_Float(&n,2.0, 
        std::string("mob/") + std::string(motorNames[i]) +std::string("/set_velocity"));
    }

    ros::Subscriber sub_lid;
    __Order_Init_Sensor(&n, 200,std::string("mob/lidar/enable"));
    sub_lid = n.subscribe("mob/lidar/laser_scan/layer0", 20, lidarCallback); //Hz
    while (sub_lid.getNumPublishers() == 0) {
    }
    ROS_INFO("Lidar enabled.");


    __Order_Init_Sensor(&n, TIME_STEP,std::string("mob/camera/enable"));



    ros::Subscriber sub_imu;
    __Order_Init_Sensor(&n, TIME_STEP,std::string("mob/inertial_unit/enable"));
    sub_imu = n.subscribe("mob/inertial_unit/roll_pitch_yaw", 2, inertialUnitCallback);
    while (sub_imu.getNumPublishers() == 0) {
    }
    ROS_INFO("Inertial unit enabled.");


    ros::Subscriber sub_gps;
    __Order_Init_Sensor(&n, TIME_STEP,std::string("mob/gps/enable"));
    sub_gps = n.subscribe("mob/gps/values", 2, GPSCallback);
    while (sub_gps.getNumPublishers() == 0) {
    }
    ROS_INFO("GPS enabled.");


    int count =0;

    while(ros::ok())
    {
          if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
            ROS_ERROR("Failed to call service time_step for next step.");
            break;
          }
          ros::spinOnce(); 
    }

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);

    ros::shutdown();
    return 0;
}






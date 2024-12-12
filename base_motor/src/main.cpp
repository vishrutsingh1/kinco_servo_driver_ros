#include<base_motor/kinco_drive.h>
#include<base_motor/ros_wrapper.h>
#include<iostream>


int main(int argc , char** argv)
{


    ros::init(argc ,argv , "base_motor_node");

    ROS_WRAPPER object("/dev/left_motor","/dev/right_motor");   //set usb port name 
    ros::spin();
    return 0;
}

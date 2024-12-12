#ifndef ROS_WRAPPER_H_
#define ROS_WRAPPER_H_
#include <ros/ros.h>
#include <base_motor/kinco_drive.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <string>
#include <std_msgs/Float32.h>



class ROS_WRAPPER
{

    public:

        

        ROS_WRAPPER(std::string left_motor_port , std::string right_motor_port);

        void ms_to_rpm(const geometry_msgs::Twist::ConstPtr &vel);
    
        void initialize_subscriber();

        void init_motor_params();
    
        void ros_callback(const geometry_msgs::Twist::ConstPtr & vel);

        void publish_encoder_data();

        ~ROS_WRAPPER();

    private:
        ros::NodeHandle _node;
       
        BaseMotor _left_motor;
        BaseMotor _right_motor;
        std::string _left_motor_port;
        std::string _right_motor_port;
        uint8_t _ID;
        ros::Subscriber _vel_sub;
        ros::Subscriber _motor_param_sub;
        ros::Publisher _encoder_left_pub;
        ros::Publisher _encoder_right_pub;
        ros::Publisher _velocity_left_pub;
        ros::Publisher _velocity_right_pub;

        double _base_width = 0.610;
        double _wheel_radius = 0.09;
        double _left_wheel_rpm = 0;
        double _right_wheel_rpm = 0;
        double _gear_reduction = 9;
        



};
#endif

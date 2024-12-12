#include "base_motor/ros_wrapper.h"




ROS_WRAPPER:: ROS_WRAPPER(std::string left_motor_port , std::string right_motor_port):_left_motor_port(left_motor_port),_right_motor_port(right_motor_port)
{
    
    initialize_subscriber();
    ROS_DEBUG("!! INITIALIZED SUBSCRIBER !!");
    init_motor_params();
    ROS_DEBUG("!! INITIALIZED MOTOR PARAMS !!");

//    publish_encoder_data();
    
}


void ROS_WRAPPER::initialize_subscriber()
{
    _vel_sub = _node.subscribe("/pgv/cmd_vel", 5 ,&ROS_WRAPPER::ros_callback,this);
    _encoder_left_pub = _node.advertise<std_msgs::Int64>("/left_wheel_encoder",1);
    _encoder_right_pub = _node.advertise<std_msgs::Int64>("/right_wheel_encoder",1); 

}

void ROS_WRAPPER::init_motor_params()
{
    _left_motor.initialize(_left_motor_port);
    _right_motor.initialize(_right_motor_port);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    _left_motor.enable();
    _right_motor.enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    _left_motor.set_operation_mode();
    _right_motor.set_operation_mode();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    _left_motor.set_acc(3);
    _right_motor.set_acc(3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    _left_motor.set_dec(5);
    _right_motor.set_dec(5);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
   
   _left_motor.set_target_velocity(0.0);
  _right_motor.set_target_velocity(0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void ROS_WRAPPER::ros_callback(const geometry_msgs::Twist::ConstPtr & vel)
{
   
    ms_to_rpm(vel);
    std::cout << _left_wheel_rpm << std::endl;
    std::cout << _right_wheel_rpm << std::endl;
     _left_motor.set_target_velocity(-_left_wheel_rpm);
    _right_motor.set_target_velocity(_right_wheel_rpm);



}

void ROS_WRAPPER::ms_to_rpm(const geometry_msgs::Twist::ConstPtr &vel)
{
    double vel_x = vel->linear.x;
    double rot_z = vel->angular.z;

    double right_wheel_speed = vel_x + (rot_z * _base_width) / 2;
    
    double left_wheel_speed = vel_x - (rot_z * _base_width) / 2;
    std::cout << left_wheel_speed << std::endl;
    

    _right_wheel_rpm =  ((right_wheel_speed) * 60.0) / (2 * 3.14 * _wheel_radius) * _gear_reduction;
    _left_wheel_rpm = ((left_wheel_speed) * 60) / (2 * 3.14 * _wheel_radius) * _gear_reduction;
 
}

void ROS_WRAPPER::publish_encoder_data()
{
   
    ros::Rate rate(20);
    while(ros::ok())
    {
        std_msgs::Int64 left_encoder , right_encoder;
       
        
        left_encoder.data = -_left_motor.get_encoder_data();
        right_encoder.data = _right_motor.get_encoder_data();

        _encoder_left_pub.publish(left_encoder);
        _encoder_right_pub.publish(right_encoder);
       
        
        ros::spinOnce();
        rate.sleep();
    } 
}


ROS_WRAPPER::~ ROS_WRAPPER()
{
    // _left_motor.disable();
    // _right_motor.disable();


}



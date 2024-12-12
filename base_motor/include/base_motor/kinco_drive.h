#ifndef _KINCO_DRIVER_H_
#define _KINCO_DRIVER_H_
#include<iostream>
#include<vector>
#include<ros/ros.h>
#include<serial/serial.h>
#include<cmath>
#include<thread>





class BaseMotor
{
    public:
        BaseMotor();
        ~BaseMotor();
        void initialize(std::string port);
        void connect();
        void set_target_velocity(double rpm);
        unsigned char calculate_checksum(const std::vector<unsigned char>& data);
        int32_t convert_rpm_to_dec(double rpm);
        void enable();
        void set_operation_mode();
        int64_t get_encoder_data();
        void set_dec(int64_t dec);
        void set_acc(int64_t acc);
        





    private:
        serial::Serial ser;
        const uint8_t ID = 0x01;
        const uint8_t Target_Velocity[2] = {0xFF,0x60};
        const uint8_t Read_Encoder[2] = {0x63,0x60};
        const uint8_t Enable[2] = {0x40,0x60};
        const uint8_t Read_Velocity[2] = {0x6C,0x60};
        const uint8_t Acceleration[2] = {0x83,0x60};
        const uint8_t Deacceleration[2] = {0x84,0x60};
        const uint8_t Operation_Mode[2] = {0x60,0x60};
        const uint8_t four_bytes = 0x23;
        const uint8_t one_bytes = 0x2F;
        const uint8_t two_bytes = 0x2B;
        const uint8_t read_bytes = 0x40;
        const uint8_t sub_index = 0x00;
        const double encoder_resolution = 10000.0;
        int32_t baud_rate = 38400;
        std::string _port;  














};

#endif

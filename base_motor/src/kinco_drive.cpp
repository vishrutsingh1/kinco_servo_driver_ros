#include<base_motor/kinco_drive.h>


BaseMotor::BaseMotor()
{

    std::cout << "!! base motor initialized !!" << std::endl;
}

BaseMotor::~BaseMotor()
	{
		if (ser.isOpen())
		{
			
			ser.close();
		}
	}

void BaseMotor::initialize(std::string port)
{
    _port = port;
    connect();
}

void BaseMotor::connect()
{
    try
		{
			ser.setPort(_port);
			ser.setBaudrate(baud_rate); // get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
			;
		}
		if (ser.isOpen())
		{
			ROS_INFO_STREAM("Serial Port initialized\"");
		}
		else
		{
			ROS_INFO_STREAM("Serial Port is not open");
		}
}

void BaseMotor::enable()
{
    
    std::vector<unsigned char> command;
    command.push_back(ID);
    command.push_back(two_bytes);
    command.push_back(Enable[0]);
    command.push_back(Enable[1]);
    command.push_back(sub_index);
    command.push_back(0x2F);
    command.push_back(0x00);
    command.push_back(0x00);
    command.push_back(0x00);
    unsigned char checksum = calculate_checksum(command);
    command.push_back(checksum);

   
    uint8_t* byteArray = command.data();
    size_t byteArraySize = command.size();
    ser.write(byteArray,byteArraySize);
    ser.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}


void BaseMotor::set_operation_mode()
{
    
    std::vector<unsigned char> command;
    command.push_back(ID);
    command.push_back(one_bytes);
    command.push_back(Operation_Mode[0]);
    command.push_back(Operation_Mode[1]);
    command.push_back(sub_index);
    command.push_back(0x03);
    command.push_back(0x00);
    command.push_back(0x00);
    command.push_back(0x00);
    unsigned char checksum = calculate_checksum(command);
    command.push_back(checksum);

    

    uint8_t* byteArray = command.data();
    size_t byteArraySize = command.size();
    ser.write(byteArray,byteArraySize);
    ser.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
}


void BaseMotor::set_target_velocity(double rpm)
{
    
    int32_t rpm_dec = convert_rpm_to_dec(rpm);
    std::vector<unsigned char> command;
    command.push_back(ID);
    command.push_back(four_bytes);
    command.push_back(Target_Velocity[0]);
    command.push_back(Target_Velocity[1]);
    command.push_back(sub_index);
    command.push_back(rpm_dec & 0xFF);
    command.push_back((rpm_dec >> 8) & 0xFF);
    command.push_back((rpm_dec >> 16) & 0xFF);
    command.push_back((rpm_dec >> 24) & 0xFF);
    unsigned char checksum = calculate_checksum(command);
    command.push_back(checksum);

    uint8_t* byteArray = command.data();
    size_t byteArraySize = command.size();
    ser.write(byteArray,byteArraySize);
    ser.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

unsigned char BaseMotor::calculate_checksum(const std::vector<unsigned char>& data)
{
    unsigned char checksum = 0;
    for (unsigned char byte : data) 
    {
        checksum += byte;
    }
    checksum = (~checksum) + 1; 
    return checksum;
}


int32_t BaseMotor::convert_rpm_to_dec(double rpm)
{
  int32_t dec = ceil(rpm * 512.0 * encoder_resolution)/1875.0;                   //as provided in kinco driver datasheet
 
  return dec;
}



int64_t BaseMotor::get_encoder_data()
{
    std::vector<unsigned char> command;
    command.push_back(ID);
    command.push_back(read_bytes);
    command.push_back(Read_Encoder[0]);
    command.push_back(Read_Encoder[1]);
    command.push_back(sub_index);
    command.push_back(0x00);
    command.push_back(0x00);
    command.push_back(0x00);
    command.push_back(0x00);
    unsigned char checksum = calculate_checksum(command);
    command.push_back(checksum);
   
    uint8_t* byteArray = command.data();
    size_t byteArraySize = command.size();
    ser.write(byteArray,byteArraySize);
    ser.flush();
    ser.flushInput();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    std::vector<uint8_t> buffer(256);
    size_t bytes_size = ser.read(buffer.data(),buffer.size());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
   
    ser.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    return buffer[5] + (buffer[6] << 8) + (buffer[7] <<16) + (buffer[8]<< 24); 


}

void BaseMotor::set_acc(int64_t acc)
{
    int64_t acc_dec = (acc * 65536)/400;            //as provided by kinco datasheet
    std::vector<unsigned char> command;
    command.push_back(ID);
    command.push_back(four_bytes);
    command.push_back(Acceleration[0]);
    command.push_back(Acceleration[1]);
    command.push_back(sub_index);
    command.push_back(acc_dec & 0xFF);
    command.push_back((acc_dec >> 8) & 0xFF);
    command.push_back((acc_dec >> 16) & 0xFF);
    command.push_back((acc_dec >> 24) & 0xFF);
    unsigned char checksum = calculate_checksum(command);
    command.push_back(checksum);
    uint8_t* byteArray = command.data();
    size_t byteArraySize = command.size();
    ser.write(byteArray,byteArraySize);
    ser.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

void BaseMotor::set_dec(int64_t dec)
{
    int64_t dec_dec = (dec * 65536)/400;
    std::vector<unsigned char> command;
    command.push_back(ID);
    command.push_back(four_bytes);
    command.push_back(Deacceleration[0]);
    command.push_back(Deacceleration[1]);
    command.push_back(sub_index);
    command.push_back(dec_dec & 0xFF);
    command.push_back((dec_dec >> 8) & 0xFF);
    command.push_back((dec_dec >> 16) & 0xFF);
    command.push_back((dec_dec >> 24) & 0xFF);
    unsigned char checksum = calculate_checksum(command);
    command.push_back(checksum);
    uint8_t* byteArray = command.data();
    size_t byteArraySize = command.size();
    ser.write(byteArray,byteArraySize);
    ser.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

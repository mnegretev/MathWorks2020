#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bulk_read_2_bytes");
    ros::NodeHandle node("~");
    int baudrate;
    std::string port;
    std::vector<int> ids;
    std::vector<int> addresses;

    

    //Default baudrate and port
    node.param("baudrate", baudrate,1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    dynamixel::GroupBulkRead  groupBulkRead(portHandler, packetHandler);
    portHandler->setBaudRate(baudrate);

    groupBulkRead.addParam(1, 36, 2);
    groupBulkRead.addParam(2, 36, 2);
}
    

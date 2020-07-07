#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bulk_read_positions");
    ros::NodeHandle node("~");
    int baudrate;
    std::string port;
    std::vector<int> ids;
    std::vector<uint16_t> positions;

    for(int i=0; i < argc; i++)
        if(strcmp(argv[i], "--ids") == 0)
            for(int j=i+1; j<argc; j++)
            {
                ids.push_back(atoi(argv[j]));
                positions.push_back(0);
            }

    std::cout << "Trying to read positions from " << ids.size() << " servomotors" << std::endl;

    //Default baudrate and port
    node.param("baudrate", baudrate,1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    dynamixel::GroupBulkRead  groupBulkRead(portHandler, packetHandler);
    portHandler->setBaudRate(baudrate);
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    for(size_t i=0; i < ids.size(); i++)
        if(!groupBulkRead.addParam(ids[i], 36, 2))
        {
            std::cout << "Cannot add parameter to bulk reader for servo " << ids[i] << std::endl;
            return -1;
        }

    dxl_comm_result = groupBulkRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) std::cout << "ERROR COMMUNICATION: " << int(dxl_comm_result) << std::endl;

    for(size_t i=0; i< ids.size(); i++)
        if(!groupBulkRead.isAvailable(ids[i], 36, 2))
        {
            std::cout << "[ID:" << ids[i] << " groupBulkRead getdata failed" << std::endl;
            return -1;
        }
    for(size_t i=0; i< ids.size(); i++)
        positions[i] = groupBulkRead.getData(ids[i], 36, 2);

    for(size_t i=0; i< ids.size(); i++)
        std::cout << "Servo " << ids[i] << " position=" << positions[i] << std::endl;
}
    

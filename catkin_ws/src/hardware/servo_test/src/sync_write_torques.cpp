#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"

#define ADDR_MX_GOAL_TORQUE 71
#define ADDR_MX_TORQUE_CONTROL 70
#define LEN_MX_GOAL_POSITION 2
#define LEN_MX_TORQUE_CONTROL 1

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_write_torques");
    ros::NodeHandle node("~");
    int baudrate;
    std::string port;
    std::vector<uint8_t> ids;
    std::vector<uint16_t> torques;

    for(int i=0; i < argc; i++)
        if(strcmp(argv[i], "--ids") == 0)
            for(int j=i+1; j<argc && strcmp(argv[j], "--torques"); j++)
                ids.push_back(atoi(argv[j]));
    for(int i=0; i < argc; i++)
        if(strcmp(argv[i], "--torques") == 0)
            for(int j=i+1; j<argc && strcmp(argv[j], "--ids"); j++)
                torques.push_back(atoi(argv[j]));

    if(ids.size() != torques.size() || !ids.size())
    {
        std::cout << "Torques and ids must be non empty equally sized arrays" << std::endl;
        return 0;
    }
    std::cout << "Trying to write torques: ";
    for(int i=0; i < torques.size(); i++) std::cout << torques[i] << "  ";
    std::cout << std::endl << "To servomotors: ";
    for(int i=0; i < ids.size(); i++) std::cout << int(ids[i]) << "  ";
    std::cout << std::endl;
    
    //Default baudrate and port
    node.param("baudrate", baudrate,1000000);
    node.param<std::string>("port",port,"/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    dynamixel::GroupSyncWrite groupSyncWriteTorque      (portHandler, packetHandler, ADDR_MX_GOAL_TORQUE,    LEN_MX_GOAL_POSITION);
    dynamixel::GroupSyncWrite groupSyncWriteTorqueEnable(portHandler, packetHandler, ADDR_MX_TORQUE_CONTROL, LEN_MX_TORQUE_CONTROL);
    portHandler->setBaudRate(baudrate);
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t goal_torque_bits[2];
    uint8_t control_torque_en = 1;

    for(size_t i=0; i < ids.size(); i++)
        if(!groupSyncWriteTorqueEnable.addParam(ids[i], &control_torque_en))
        {
            std::cout << "Cannot add parameter to sync writer (torque control enable) for servo " << ids[i] << std::endl;
            return -1;
        }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteTorqueEnable.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) std::cout << "ERROR COMMUNICATION: " << int(dxl_comm_result) << std::endl;

    // Clear syncwrite parameter storage
    groupSyncWriteTorque.clearParam();

    for(size_t i=0; i < ids.size(); i++)
    {
        goal_torque_bits[0] = DXL_LOBYTE(torques[i]);
        goal_torque_bits[1] = DXL_HIBYTE(torques[i]);
        if(!groupSyncWriteTorque.addParam(ids[i], goal_torque_bits))
        {
            std::cout << "Cannot add parameter to sync writer (torque) for servo " << ids[i] << std::endl;
            return -1;
        }
    }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteTorque.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) std::cout << "ERROR COMMUNICATION: " << int(dxl_comm_result) << std::endl;

    // Clear syncwrite parameter storage
    groupSyncWriteTorque.clearParam();
    
}
    

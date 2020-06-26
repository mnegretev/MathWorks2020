#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

void callback_la_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bulk_read_2_bytes");
    ros::NodeHandle n("~");
    ros::Publisher  pub_current_pose = n.advertise<std_msgs::Float32MultiArray>("/hardware/la_current_pose", 1);
    ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber sub_goal_pose    = n.subscribe("/hardware/la_goal_pose", 1, callback_la_goal_pose);
    ros::Rate loop(20);
    
    int baudrate = 1000000;
    std::string port    = "/dev/ttyUSB0";
    std::string str_ids = "1 2 3 4 5 6 7 8 9";
    if(ros::param::has("~port"))
        ros::param::get("~port", port);
    if(ros::param::has("~baudrate"))
        ros::param::get("~baudrate", baudrate);
    if(ros::param::has("~servo_ids"))
        ros::param::get("~servo_ids", str_ids);
    
    std::vector<int>      servo_IDs;
    std::vector<uint16_t> positions;
    std::istringstream iss(str_ids);
    for(std::string s; iss >> s;)
    {
        servo_IDs.push_back(atoi(s.c_str()));
        positions.push_back(0);
    }

    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    dynamixel::GroupBulkRead  groupBulkRead(portHandler, packetHandler);
    portHandler->setBaudRate(baudrate);
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;             
    for(size_t i=0; i < servo_IDs.size(); i++)
        if(!groupBulkRead.addParam(servo_IDs[i], 36, 2))
        {
            std::cout << "Cannot add parameter to bulk reader for servo " << servo_IDs[i] << std::endl;
            return -1;
        }
    
    while(ros::ok())
    {
        dxl_comm_result = groupBulkRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) std::cout << "ERROR COMMUNICATION: " << int(dxl_comm_result) << std::endl;
        
        for(size_t i=0; i< servo_IDs.size(); i++)
            if(!groupBulkRead.isAvailable(servo_IDs[i], 36, 2))
                std::cout << "[ID:" << servo_IDs[i] << " groupBulkRead getdata failed" << std::endl;
        for(size_t i=0; i< servo_IDs.size(); i++)
            positions[i] = groupBulkRead.getData(servo_IDs[i], 36, 2);
        
        loop.sleep();
    }
}


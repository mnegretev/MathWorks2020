#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

std::vector<double> goal_torque;
void callback_la_torque(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    goal_positions = msg->data;
}

int main(int argc, char **argv)
{
    /*
     * Initializing ROS stuff: node, publishers and subscribers.
     */
    std::cout << "INITIALIZING LEFT_ARM_NODE ..." << std::endl;
    ros::init(argc, argv, "bulk_read_2_bytes");
    ros::NodeHandle n("~");
    ros::Publisher  pub_current_pose = n.advertise<std_msgs::Float64MultiArray>("/hardware/la_current_pose", 1);
    ros::Publisher  pub_joint_states = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber sub_goal_pose    = n.subscribe("/hardware/la_torque", 1, callback_la_torque);
    ros::Rate loop(400);


    /*
     * Reading and parsing parameters.
     */
    int baudrate = 1000000;
    std::string port    = "/dev/ttyUSB0";
    std::string str_ids = "1 2 3 4 5 6";
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

    std::cout << "LeftArm.->Using IDs: ";
    for(int i=0; i < servo_IDs.size(); i++) std::cout << servo_IDs[i] << "  ";
    std::cout << std::endl;
    std::cout << "LeftArm.->Using port: " << port << "  at " << baudrate << std::endl;

    
    /*
     * Initializing variables for reading and writing through serial port
     */
    std::string latencia = "echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer";
    char const *late = latencia.data();
    system(late);     
    
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

    /*
     * Variables for getting positions and publishing their values
     */
    std_msgs::Float64MultiArray msg_current_pose;
    sensor_msgs::JointState joint_states;
    msg_current_pose.data.resize(7);
    int zero_arm[7] = {1543, 1600, 1800, 2100, 2048, 1800, 1050};
    int zero_gripper[2] = {2440, 2680};
    float bitsPerRadian = 4095.0/360.0*180.0/M_PI;
    std::string names[9] = {"la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint", "la_grip_left", "la_grip_right"};
    joint_states.name.insert(joint_states.name.begin(), names, names + 9);
    int rate_downsampling = 0;
    
    while(ros::ok())
    {
        dxl_comm_result = groupBulkRead.txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) std::cout << "ERROR COMMUNICATION: " << int(dxl_comm_result) << std::endl;
        
        for(size_t i=0; i< servo_IDs.size(); i++)
            if(!groupBulkRead.isAvailable(servo_IDs[i], 36, 2))
                std::cout << "[ID:" << servo_IDs[i] << " groupBulkRead getdata failed" << std::endl;
        for(size_t i=0; i< servo_IDs.size(); i++)
            positions[i] = groupBulkRead.getData(servo_IDs[i], 36, 2);

        msg_current_pose.data[0] = -((float) (zero_arm[0]-positions[0]))/bitsPerRadian;
        msg_current_pose.data[1] = -((float) (zero_arm[1]-positions[1]))/bitsPerRadian;
        msg_current_pose.data[2] = -((float) (zero_arm[2]-positions[2]))/bitsPerRadian;
        msg_current_pose.data[3] =  ((float) (zero_arm[3]-positions[3]))/bitsPerRadian;
        msg_current_pose.data[4] = -((float) (zero_arm[4]-positions[4]))/bitsPerRadian;
        msg_current_pose.data[5] = -((float) (zero_arm[5]-positions[5]))/bitsPerRadian;
        msg_current_pose.data[6] = -((float) (zero_arm[6]-positions[6]))/bitsPerRadian;
        pub_current_pose.publish(msg_current_pose);

        if(++rate_downsampling > 20)
        {
            rate_downsampling = 0;
            joint_states.position = msg_current_pose.data;
            joint_states.header.stamp = ros::Time::now();
            pub_joint_states.publish(joint_states);
        }
        loop.sleep();
    }
}


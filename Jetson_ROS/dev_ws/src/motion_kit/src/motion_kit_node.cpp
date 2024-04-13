#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

serial::Serial serial_port;


int motion_kit_init(std::string portName)
{

    try
    {
        serial_port.setPort(portName);//"/dev/ttyTHS1"
        serial_port.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(to);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(serial_port.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
}

uint8_t RawControlFrame[6] = {0x23,0x01,0x80,0x00,0x80,0x00};

void createControlFrame_joy(const sensor_msgs::JoyConstPtr& msg){
    RawControlFrame[2] = 0x80 + int(msg->axes[1]*127);
    RawControlFrame[4] = 0x80 + int(msg->axes[0]*127);
    serial_port.write(RawControlFrame,6);
    // ROS_INFO_STREAM("DONE");
    ROS_INFO_STREAM("RAW: " << RawControlFrame);
}

void createControlFrame_manual(const geometry_msgs::Twist::ConstPtr& msg){
    RawControlFrame[2] = 0x80 + int(msg->linear.y);
    RawControlFrame[4] = 0x80 + int(msg->linear.x);
    serial_port.write(RawControlFrame,6);
    // ROS_INFO_STREAM("DONE");
    ROS_INFO_STREAM("RAW: " << RawControlFrame);
}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::Subscriber joyControl = nh.subscribe("joy", 1000, createControlFrame_joy);
    
    ros::Publisher manualControlPub = nh.advertise<geometry_msgs::Twist>("control",1000);
    ros::Subscriber manualControl = nh.subscribe("control", 1000, createControlFrame_manual);

    motion_kit_init("/dev/ttyTHS1");

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
    }
}

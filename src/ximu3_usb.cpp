/*
* Author : Sanket Lokhande
* Date Created : 21 Feb 2024
* Last Modified : 9 March 2024
* Description: This ros driver was created to connect 3 XIMU3 devices and broadcast the data simultaneously to ROS network
* Usage: $rosrun ximu3_ros_driver ximu3_node 0  #0 /1 /2 for devices as indicated in connection.hpp file
*/



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "ximu3_ros_driver/connection.hpp"


#include "ximu3_ros_driver/Accel.h"
#include "ximu3_ros_driver/Gyro.h"
#include "ximu3_ros_driver/Quaternion.h"
#include "ximu3_ros_driver/Euler.h"
#include "ximu3_ros_driver/LinearAccel.h"
#include "ximu3_ros_driver/Mag.h"
#include "ximu3_ros_driver/RotMatrix.h"


int main(int argc, char **argv)
{

    XIMU3_SensorMessage sensorMessage;
    memset(&sensorMessage, 0, sizeof(XIMU3_SensorMessage));

    sensor_msgs::Imu imuSensorMessage;
    connection connection(connection_type::usbconnection, sensorMessage, 0U);

    std::string nodeName = "ximu3_usb_node";
    ros::init(argc, argv, nodeName);

    ros::NodeHandle n;

    std::string topicName = "/ximu_sensor/"; 
    ros::Publisher imuSensorMessage_pub = n.advertise<sensor_msgs::Imu>(topicName ,1000);

    ros::Rate loop_rate(200);

    imuSensorMessage.header.frame_id = "0";
    imuSensorMessage.header.seq = 0;

    while (ros::ok())
    {
      imuSensorMessage.header.stamp = ros::Time::now();
      imuSensorMessage.header.seq++;
      imuSensorMessage.linear_acceleration.x = (sensorMessage.inertialMessage.accelerometer_x * 9.8000);
      imuSensorMessage.linear_acceleration.y = (sensorMessage.inertialMessage.accelerometer_y * 9.8000);
      imuSensorMessage.linear_acceleration.z = (sensorMessage.inertialMessage.accelerometer_z * 9.8000);

      imuSensorMessage.angular_velocity.x = (sensorMessage.inertialMessage.gyroscope_x * 0.0174533);
      imuSensorMessage.angular_velocity.y = (sensorMessage.inertialMessage.gyroscope_y * 0.0174533);
      imuSensorMessage.angular_velocity.z = (sensorMessage.inertialMessage.gyroscope_z * 0.0174533);

      imuSensorMessage.orientation.x = sensorMessage.quaternionMessage.x;
      imuSensorMessage.orientation.y = sensorMessage.quaternionMessage.y;
      imuSensorMessage.orientation.z = sensorMessage.quaternionMessage.z;
      imuSensorMessage.orientation.w = sensorMessage.quaternionMessage.w;

      imuSensorMessage_pub.publish(imuSensorMessage);
      ros::spinOnce();

      loop_rate.sleep();

    }

    return 0;
}


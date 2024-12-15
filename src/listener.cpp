/*
* Author : Sanket Lokhande
* Date Created : 21 Feb 2024
* Last Modified : 9 March 2024
* Description : This ros listener will listen to accelerometer values broadcasted by topic sensor0/accel. modify to see rest of the topics
* Usage: $rosrun ximu3_ros_driver listener
*/


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ximu3_ros_driver/Accel.h"
#include "ximu3_ros_driver/Gyro.h"
#include "ximu3_ros_driver/RotMatrix.h"
#include "geometry_msgs/PoseStamped.h"


typedef struct sensorMessage
{
  uint64_t timestamp;
  double accel_x;
  double accel_y;
  double accel_z;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double rot_xx;
  double rot_xy;
  double rot_xz;
  double rot_yx;
  double rot_yy;
  double rot_yz;
  double rot_zx;
  double rot_zy;
  double rot_zz;
}sensorMessage;

typedef struct dogpose
{
  double posx;
  double posy;
  double posw;
};

typedef struct allSenMsg
{
  sensorMessage sensor0;
  sensorMessage sensor1;
  sensorMessage sensor2;
  dogpose       dogpos;
}allSenMsg;

allSenMsg populatedMsg;

void dogPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  populatedMsg.dogpos.posx = msg->pose.position.x;
  populatedMsg.dogpos.posy = msg->pose.position.y;
  populatedMsg.dogpos.posw = msg->pose.orientation.w;

  printf("%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",   \
          populatedMsg.sensor0.timestamp, \
          populatedMsg.sensor0.accel_x, populatedMsg.sensor0.accel_y, populatedMsg.sensor0.accel_z,\
          populatedMsg.sensor0.gyro_x, populatedMsg.sensor0.gyro_y, populatedMsg.sensor0.gyro_z,\
          populatedMsg.sensor0.rot_xx, populatedMsg.sensor0.rot_xy, populatedMsg.sensor0.rot_xz, populatedMsg.sensor0.rot_yx, populatedMsg.sensor0.rot_yy, populatedMsg.sensor0.rot_yz, populatedMsg.sensor0.rot_zx, populatedMsg.sensor0.rot_zy, populatedMsg.sensor0.rot_zz \
        );
  printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",   \
          populatedMsg.sensor1.accel_x, populatedMsg.sensor1.accel_y, populatedMsg.sensor1.accel_z,\
          populatedMsg.sensor1.gyro_x, populatedMsg.sensor1.gyro_y, populatedMsg.sensor1.gyro_z,\
          populatedMsg.sensor1.rot_xx, populatedMsg.sensor1.rot_xy, populatedMsg.sensor1.rot_xz, populatedMsg.sensor1.rot_yx, populatedMsg.sensor1.rot_yy, populatedMsg.sensor1.rot_yz, populatedMsg.sensor1.rot_zx, populatedMsg.sensor1.rot_zy, populatedMsg.sensor1.rot_zz \
        );

  printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",   \
          populatedMsg.sensor2.accel_x, populatedMsg.sensor2.accel_y, populatedMsg.sensor2.accel_z,\
          populatedMsg.sensor2.gyro_x, populatedMsg.sensor2.gyro_y, populatedMsg.sensor2.gyro_z,\
          populatedMsg.sensor2.rot_xx, populatedMsg.sensor2.rot_xy, populatedMsg.sensor2.rot_xz, populatedMsg.sensor2.rot_yx, populatedMsg.sensor2.rot_yy, populatedMsg.sensor2.rot_yz, populatedMsg.sensor2.rot_zx, populatedMsg.sensor2.rot_zy, populatedMsg.sensor2.rot_zz \
        );


  printf("%f,%f,%f\n",populatedMsg.dogpos.posx, populatedMsg.dogpos.posy, populatedMsg.dogpos.posw);
}

void accelCallback0(const ximu3_ros_driver::Accel & msg)
{
  populatedMsg.sensor0.timestamp = msg.timestamp ;
  populatedMsg.sensor0.accel_x = msg.accel_x;
  populatedMsg.sensor0.accel_y = msg.accel_y;
  populatedMsg.sensor0.accel_z = msg.accel_z;
}

void accelCallback1(const ximu3_ros_driver::Accel & msg)
{
  populatedMsg.sensor1.timestamp = msg.timestamp ;
  populatedMsg.sensor1.accel_x = msg.accel_x;
  populatedMsg.sensor1.accel_y = msg.accel_y;
  populatedMsg.sensor1.accel_z = msg.accel_z;
}

void accelCallback2(const ximu3_ros_driver::Accel & msg)
{
  populatedMsg.sensor2.timestamp = msg.timestamp ;
  populatedMsg.sensor2.accel_x = msg.accel_x;
  populatedMsg.sensor2.accel_y = msg.accel_y;
  populatedMsg.sensor2.accel_z = msg.accel_z;
}

void gyroCallback0(const ximu3_ros_driver::Gyro & msg)
{
  populatedMsg.sensor0.timestamp = msg.timestamp ;
  populatedMsg.sensor0.gyro_x = msg.gyro_x;
  populatedMsg.sensor0.gyro_y = msg.gyro_y;
  populatedMsg.sensor0.gyro_z = msg.gyro_z;
}

void gyroCallback1(const ximu3_ros_driver::Gyro & msg)
{
  populatedMsg.sensor1.timestamp = msg.timestamp ;
  populatedMsg.sensor1.gyro_x = msg.gyro_x;
  populatedMsg.sensor1.gyro_y = msg.gyro_y;
  populatedMsg.sensor1.gyro_z = msg.gyro_z;
}

void gyroCallback2(const ximu3_ros_driver::Gyro & msg)
{
  populatedMsg.sensor2.timestamp = msg.timestamp ;
  populatedMsg.sensor2.gyro_x = msg.gyro_x;
  populatedMsg.sensor2.gyro_y = msg.gyro_y;
  populatedMsg.sensor2.gyro_z = msg.gyro_z;
}

void rotCallback0(const ximu3_ros_driver::RotMatrix & msg)
{
  populatedMsg.sensor0.timestamp = msg.timestamp;
  populatedMsg.sensor0.rot_xx = msg.rot_xx;
  populatedMsg.sensor0.rot_xy = msg.rot_xy;
  populatedMsg.sensor0.rot_xz = msg.rot_xz;
  populatedMsg.sensor0.rot_yx = msg.rot_yx;
  populatedMsg.sensor0.rot_yy = msg.rot_yy;
  populatedMsg.sensor0.rot_yz = msg.rot_yz;
  populatedMsg.sensor0.rot_zx = msg.rot_zx;
  populatedMsg.sensor0.rot_zy = msg.rot_zy;
  populatedMsg.sensor0.rot_zz = msg.rot_zz;
}

void rotCallback1(const ximu3_ros_driver::RotMatrix & msg)
{
  populatedMsg.sensor1.timestamp = msg.timestamp;
  populatedMsg.sensor1.rot_xx = msg.rot_xx;
  populatedMsg.sensor1.rot_xy = msg.rot_xy;
  populatedMsg.sensor1.rot_xz = msg.rot_xz;
  populatedMsg.sensor1.rot_yx = msg.rot_yx;
  populatedMsg.sensor1.rot_yy = msg.rot_yy;
  populatedMsg.sensor1.rot_yz = msg.rot_yz;
  populatedMsg.sensor1.rot_zx = msg.rot_zx;
  populatedMsg.sensor1.rot_zy = msg.rot_zy;
  populatedMsg.sensor1.rot_zz = msg.rot_zz;
}

void rotCallback2(const ximu3_ros_driver::RotMatrix & msg)
{
  populatedMsg.sensor2.timestamp = msg.timestamp;
  populatedMsg.sensor2.rot_xx = msg.rot_xx;
  populatedMsg.sensor2.rot_xy = msg.rot_xy;
  populatedMsg.sensor2.rot_xz = msg.rot_xz;
  populatedMsg.sensor2.rot_yx = msg.rot_yx;
  populatedMsg.sensor2.rot_yy = msg.rot_yy;
  populatedMsg.sensor2.rot_yz = msg.rot_yz;
  populatedMsg.sensor2.rot_zx = msg.rot_zx;
  populatedMsg.sensor2.rot_zy = msg.rot_zy;
  populatedMsg.sensor2.rot_zz = msg.rot_zz;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;


  ros::Subscriber suba0 = n.subscribe("sensor0/accel", 1000, accelCallback0);
  ros::Subscriber suba1 = n.subscribe("sensor1/accel", 1000, accelCallback1);
  ros::Subscriber suba2 = n.subscribe("sensor2/accel", 1000, accelCallback2);
  ros::Subscriber subg0 = n.subscribe("sensor0/gyro", 1000, gyroCallback0);
  ros::Subscriber subg1 = n.subscribe("sensor1/gyro", 1000, gyroCallback1);
  ros::Subscriber subg2 = n.subscribe("sensor2/gyro", 1000, gyroCallback2);
  ros::Subscriber subr0 = n.subscribe("sensor0/rot", 1000, rotCallback0);
  ros::Subscriber subr1 = n.subscribe("sensor1/rot", 1000, rotCallback1);
  ros::Subscriber subr2 = n.subscribe("sensor2/rot", 1000, rotCallback2);
  ros::Subscriber subp = n.subscribe("dog/pose", 1000, dogPoseCallback);



  ros::spin();


  return 0;
}

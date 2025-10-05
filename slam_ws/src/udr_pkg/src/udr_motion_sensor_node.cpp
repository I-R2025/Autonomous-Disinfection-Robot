#ifndef MMC_SUCCESS
    #define MMC_SUCCESS 1
#endif

#ifndef MMC_FAILED
    #define MMC_FAILED 0
#endif

#include "ros/ros.h"
#include <ros/time.h>
#include <std_srvs/Trigger.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <udr_pkg/Definitions.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h> // Include this for geometry_msgs::Quaternion

class Control
{
 public:
 
  ros::NodeHandle n;

  // initializations
  typedef void* HANDLE;
  typedef int BOOL;

  // Variables:
  bool deviceOpenedCheckStatus;
  bool homingCompletedStatus;
  void* g_pKeyHandle;
  unsigned short g_usNodeId;
  int g_baudrate; 
  std::string g_deviceName;
  std::string g_protocolStackName; 
  std::string g_interfaceName;
  std::string g_portName;
  const std::string g_programName;
  int lStartOfSelection;
  int lMaxStrSize;
  char* pPortNameSel;
  int lEndOfSelection;
  unsigned int ulErrorCode;
  signed int pPositionIsCounts_x;
  signed int pPositionIsCounts_y;
  signed int pPositionIsCounts_z;
  signed int pVelocityIsRpm_x;
  signed int pVelocityIsRpm_y;
  int pVelocityIsCounts;
  int pTargetReached;
  unsigned int profile_velocity;
  unsigned int profile_acceleration;
  unsigned int profile_deceleration;
  int oIsFault;
  bool homingflag;
  unsigned short pInputs;
 
  // Odometry publisher and transform broadcaster
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;

  // Odometry tracking variables
  double x, y, th; // Position and orientation
  double vx, vy, vth; // Velocities
  double yaw;
  ros::Subscriber twist_subscriber;

  // class constructor 
  Control()
  {
    deviceOpenedCheckStatus = MMC_FAILED;
    homingCompletedStatus = MMC_FAILED;
    g_usNodeId = 1;
    g_baudrate = 1000000; 
    g_deviceName = "EPOS4";
    g_protocolStackName = "MAXON SERIAL V2"; 
    g_interfaceName = "USB";
    lStartOfSelection = 1;
    lMaxStrSize = 255;
    pPortNameSel = new char[lMaxStrSize];
    lEndOfSelection = 0;
    ulErrorCode = 0;
    pPositionIsCounts_x;
    pPositionIsCounts_y;
    pPositionIsCounts_z;
    pVelocityIsCounts;
    pTargetReached;
    profile_velocity = 1200; //default: 300
    profile_acceleration = 1000; //default: 50
    profile_deceleration = 1000; //default: 50
    homingflag = false;

    // Initialize publisher for odometry
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    twist_subscriber = n.subscribe("/cmd_vel", 10, &Control::subscribercallback, this);

    // Initialize odometry variables
    x = 0.0;
    y = 0.0;
    th = 0.0;
    vx = 0.0;
    vy = 0.0;
    vth = 0.0;
    yaw = 0;
  }

  void subscribercallback(const geometry_msgs::Twist::ConstPtr& msg) // Define
  {
    nav_msgs::Odometry odom_msg;
  	if(!VCS_ActivateProfileVelocityMode(g_pKeyHandle,1,&ulErrorCode))
    {
       ROS_INFO_STREAM("E: SCBO1"); return;
    }
    if(!VCS_SetEnableState(g_pKeyHandle,1,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO3 "<<ulErrorCode);return;
    } 
    if(!VCS_SetVelocityProfile(g_pKeyHandle,1,2000,2000,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO2");return;
    }
     

    if(!VCS_ActivateProfileVelocityMode(g_pKeyHandle,2,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO4");return;
    }
    if(!VCS_SetEnableState(g_pKeyHandle,2,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO6");return;
    }
    if(!VCS_SetVelocityProfile(g_pKeyHandle,2,2000,2000,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO5");return;
    }
    
    int pVelocityIsAveraged_r;
    int pVelocityIsAveraged_l;
    if(!VCS_GetVelocityIs(g_pKeyHandle, 1, &pVelocityIsAveraged_r, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO9");return;
    }
    if(!VCS_GetVelocityIs(g_pKeyHandle, 2, &pVelocityIsAveraged_l, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCB10");return;
    }
    
    // Motor commands to wheel velocities
    float l_motor = -(((msg->linear.x)/2 + (msg->angular.z)/2)*285*60)/(2*3.14*0.124);
    float r_motor = ((((msg->linear.x)/2 - (msg->angular.z)/2)*285*60)/(2*3.14*0.124));

    // Left and right motor velocity
    float v_left = (pVelocityIsAveraged_l * 2 * 3.14 * 0.124) /(285*60);
    float v_right = (pVelocityIsAveraged_r * 2 * 3.14 * 0.124) /(285*60);
    float wheel_base = 0.508; // Wheelbase distance in meters

    // Time step (dt) for odometry update
    double dt = 0.1;

    // Calculate linear and angular velocities
    double v_linear = (v_left + v_right) / 2.0;
    double v_angular = (v_right - v_left) / wheel_base;
    
    
    // Update yaw (orientation)
    double delta_yaw = v_angular * dt;  // Change in yaw
    yaw = yaw  +  delta_yaw;  // Update the current yaw

    // Normalize yaw to be between -pi and pi
    if (yaw > M_PI) yaw -= 2 * M_PI;
    if (yaw < -M_PI) yaw += 2 * M_PI;

    // Update position
    double delta_x = v_linear * cos(yaw) * dt;  // Change in X
    double delta_y = v_linear * sin(yaw) * dt;  // Change in Y

    x += delta_x;  // Update X position
    y += delta_y;  // Update Y position

    // Fill the Odometry message with updated values
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;  // 2D robot, z is zero

    // Convert yaw to quaternion for orientation
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);  // Convert yaw to quaternion
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);  // Convert tf quaternion to ROS message

    odom_msg.pose.pose.orientation = odom_quat;  // Set orientation in odometry

    // Set linear and angular velocity in the odometry message
    odom_msg.twist.twist.linear.x = v_linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = v_angular;

    // Publish the odometry message
    odom_pub.publish(odom_msg);
    
   
    // Broadcast the transform (for tf)
    odom_broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, sin(yaw / 2), cos(yaw / 2)), tf::Vector3(x, y, 0)),
            ros::Time::now(), "odom", "base_link"));

    // Command motors to move
    if (!VCS_MoveWithVelocity(g_pKeyHandle, 1, r_motor, &ulErrorCode)) {
      ROS_INFO_STREAM("E: SCBO7");
      return;
    }
    if (!VCS_MoveWithVelocity(g_pKeyHandle, 2, l_motor, &ulErrorCode)) {
      ROS_INFO_STREAM("E: SCBO8");
      return;
    }

    ROS_INFO_STREAM("Right motor velocity: " << r_motor);
    ROS_INFO_STREAM("Left motor velocity: " << l_motor);
    ROS_INFO_STREAM("pVelocityIsAveraged_r: " << pVelocityIsAveraged_r);
    ROS_INFO_STREAM("pVelocityIsAveraged_l: " << pVelocityIsAveraged_l);
  }

  bool start_communication()
  {
    if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), (char*)g_interfaceName.c_str(), lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
    {
        ROS_INFO_STREAM("E: SCO1");
        return false;
    }
    
    BOOL oIsFault = 0; //0 is not in fault state

    g_portName = pPortNameSel;
    ROS_INFO_STREAM("Port Name: " << g_portName);

    if(g_pKeyHandle = VCS_OpenDevice((char*)"EPOS4", (char*)"MAXON SERIAL V2",(char*)"USB", (char*)"USB0", &ulErrorCode))
    {
        ROS_INFO_STREAM("Device successfully opened" << g_pKeyHandle);
    }
    else ROS_INFO_STREAM("E: SCO2");

    return true;
  }

  bool sensor_settings()
  {
    if(!VCS_SetSensorType(g_pKeyHandle, 1, 1, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: SSO1");
      return false;
    }
    ROS_INFO_STREAM("Sensors successfully reset");
    return true;
  }

  bool configure_IOs()
  {
    if(!VCS_DigitalInputConfiguration(g_pKeyHandle, 1, 1, 15, 1, 0, 0, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: CIO1");
      return false;
    }

    ROS_INFO_STREAM("Digital inputs and outputs successfully configured");
    return true;
  }

};

int main(int argc, char**argv)
{
  ros::init(argc, argv, "control_pub");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  Control program;
  program.start_communication();
  program.sensor_settings();
  ros::spin();

  return 0;
}


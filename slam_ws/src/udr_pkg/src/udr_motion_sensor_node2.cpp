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
#include <std_msgs/Int64.h>

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
  int pPositionIs_r;
  int pPositionIs_l;
  float l_motor;
  float r_motor;

  ros::Publisher pubForRightCounts;
  ros::Publisher pubForLeftCounts;
  

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
    pPositionIs_r = 0;
    pPositionIs_l = 0;
    l_motor=0;
    r_motor=0;
    // Initialize publishers and subscribers
    pubForRightCounts=n.advertise<std_msgs::Int64>("rightWheel", 1000);
    pubForLeftCounts=n.advertise<std_msgs::Int64>("leftWheel", 1000);
    twist_subscriber = n.subscribe("/cmd_vel", 10, &Control::subscribercallback, this);

   
  }

  void subscribercallback(const geometry_msgs::Twist::ConstPtr& msg) // Define
  {
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
    
    // Motor commands to wheel velocities
    
    if(msg->linear.x <(-0.05) || msg->linear.x>0.05 || msg->angular.z<(-0.005) || msg->angular.z>0.005)
    {
	    l_motor = -((((msg->linear.x)/2 + (msg->angular.z)/2)*285*60)/(2*3.14*0.124))/8; // lin: max 0.25, 5489, min -0.25, -5489
	    r_motor = ((((msg->linear.x)/2 - (msg->angular.z)/2)*285*60)/(2*3.14*0.124))/8; // lin: max 0.25, -5489, min -0.25, 5489
    }
    else
    {
    	 l_motor =0;r_motor=0;
    	 if(!VCS_SetQuickStopState(g_pKeyHandle, 1, &ulErrorCode))
    	{	
        ROS_INFO_STREAM("1st node quickstoped");
    	}
    	if(!VCS_SetQuickStopState(g_pKeyHandle, 2, &ulErrorCode))
    	{
        ROS_INFO_STREAM("2nd node quickstoped");
   	}
    }
    
    
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
    if(!VCS_ClearFault(g_pKeyHandle, 1, &ulErrorCode))
    {
        ROS_INFO_STREAM("1st node fault not cleared");
    }
    if(!VCS_ClearFault(g_pKeyHandle, 2, &ulErrorCode))
    {
        ROS_INFO_STREAM("2nd node fault not cleared");
    }
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
  
  bool pub_counts()
  {
    std_msgs::Int64 r_counts;
    std_msgs::Int64 l_counts;
    
    if(!VCS_GetPositionIs(g_pKeyHandle, 1, &pPositionIs_r, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: PCO1");
      return false;
    }
    if(!VCS_GetPositionIs(g_pKeyHandle, 2, &pPositionIs_l, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: PCO2");
      return false;
    }
    
    r_counts.data = -pPositionIs_r;
    l_counts.data = pPositionIs_l;
    
    pubForRightCounts.publish(r_counts);
    pubForLeftCounts.publish(l_counts);
    return true;
  }

};

int main(int argc, char**argv)
{
  ros::init(argc, argv, "control_pub");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  Control program;
  program.start_communication();
  program.sensor_settings();
 
    while(ros::ok())
    {
        ros::spinOnce();
        program.pub_counts();
        loop_rate.sleep();
    }
    //ros::spin();
  return 0;
}


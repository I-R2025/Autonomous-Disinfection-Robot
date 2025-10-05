//Define some parameters:
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


class Control
{
 public:
 
  ros::NodeHandle n;

  // initializations

  typedef void* HANDLE;
  typedef int BOOL;

  //Variables:
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
 
  // iniializing publisher

  ros::Publisher ultrasonic_publisher;
  std_msgs::Float64 output;
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
 //   g_programName = "EPOS2 Controller";
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
   
  //  ultrasonic_publisher = n.advertise<std_msgs::Float64>("ultrasonic_trans",10);
    twist_subscriber = n.subscribe("/cmd_vel",10,&Control::subscribercallback,this);

  }

  void subscribercallback(const geometry_msgs::Twist::ConstPtr& msg) // Define
  {
   /* if(!VCS_GetAllDigitalInputs(g_pKeyHandle,1,&pInputs,&ulErrorCode))
    {
      return;
    }
    ROS_INFO_STREAM("PIR Sensor Value"<<pInputs);*/

    float l_motor=24000*((0.5*msg->linear.x) - (0.5*msg->angular.z));
    float r_motor=-24000*((0.5*msg->linear.x) + (0.5*msg->angular.z));

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
    
    if(pInputs == 32768)
    {
      r_motor = 0;
      l_motor = 0;
    }   
    if(!VCS_MoveWithVelocity(g_pKeyHandle,1,r_motor,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO7");return;
    }
    if(!VCS_MoveWithVelocity(g_pKeyHandle,2,l_motor,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: SCBO8");return;
    }
    if(abs(r_motor)>4000) r_motor = 4000;
    if(abs(l_motor)>4000) l_motor = 4000;

    ROS_INFO_STREAM("right motor velocity"<<r_motor);
    ROS_INFO_STREAM("left motor velocity"<<l_motor);

    ROS_INFO_STREAM("linear x velocity"<<msg->linear.x);
    ROS_INFO_STREAM("angular z velocity"<<msg->angular.z);
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
      /*  if(!VCS_SendNMTService(g_pKeyHandle,1,129,&ulErrorCode))
        {
          ROS_INFO_STREAM("E: SCO3");
          return false;
        }
   */
        ROS_INFO_STREAM("Device successfully opened"<< g_pKeyHandle);
        unsigned int lBaudrate = 0;
        unsigned int lTimeout = 0;

        if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, &ulErrorCode))
        {
          if(VCS_SetProtocolStackSettings(g_pKeyHandle, lBaudrate, lTimeout, &ulErrorCode))
          {
            if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, &ulErrorCode))
            {
              if(g_baudrate==(int)lBaudrate)
              {
                ROS_INFO_STREAM("Device successfully opened with key handle: "  << g_pKeyHandle);
              } 
            } 
          } 
        } 
    }
    else ROS_INFO_STREAM("E: SCO2");

    if(!VCS_ClearFault(g_pKeyHandle,1,&ulErrorCode))
    {
        ROS_INFO_STREAM("E: SCO5");
        return false;
    }
    if(!VCS_ClearFault(g_pKeyHandle,2,&ulErrorCode))
    {
        ROS_INFO_STREAM("E: SCO7");
        return false;
    }
    if(!VCS_GetFaultState(g_pKeyHandle, 1, &oIsFault, &ulErrorCode ))
    {
        ROS_INFO_STREAM("E: SCO4"<<ulErrorCode);
        return false;
    }
    
    ROS_INFO_STREAM("Debug 1: FaultState:" << oIsFault);

    if(oIsFault)
    {
      if(!VCS_ClearFault(g_pKeyHandle,1,&ulErrorCode))
      {
        ROS_INFO_STREAM("E: SCO5");
        return false;
      }
    }
  
    if(!VCS_GetFaultState(g_pKeyHandle, 2, &oIsFault, &ulErrorCode ))
    {
        ROS_INFO_STREAM("E: SCO6");
        return false;
    }

    ROS_INFO_STREAM("Debug 2: FaultState:" << oIsFault);

    if(oIsFault)
    {
      if(!VCS_ClearFault(g_pKeyHandle,2,&ulErrorCode))
      {
        ROS_INFO_STREAM("E: SCO7");
        return false;
      }
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

    if(!VCS_SetIncEncoderParameter(g_pKeyHandle, 1, 500, 0, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: SSO2");return false;
    }

    if(!VCS_SetSensorType(g_pKeyHandle, 2, 1, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: SSO3");return false;
    }

    if(!VCS_SetIncEncoderParameter(g_pKeyHandle, 2, 500, 0, &ulErrorCode))
    {
      ROS_INFO_STREAM("E: SSO4");return false;
    }

    ROS_INFO_STREAM("Sensors successfully reset");
    return true;
    
  }

  bool configure_IOs()
  {
    
    // PIR sensor configuration

    if(!VCS_DigitalInputConfiguration(g_pKeyHandle,1,1,15,1,0,0,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: CIO1");
      return false;
    }

    
    ROS_INFO_STREAM("Digital inputs and outputs successfully configured");
    return true;
  }

  bool ultrasonic_transmitter()
  {
   /*  if(!VCS_DigitalOutputConfiguration(g_pKeyHandle,1,1,15,1,0,0,&ulErrorCode))
     {
        ROS_INFO_STREAM("E: CIO2");
        return false;
     }
     ros::Time pulse_start = ros::Time::now();
     ros::Duration(0.001).sleep();

    if(!VCS_DigitalOutputConfiguration(g_pKeyHandle,1,1,15,0,0,0,&ulErrorCode))
    {
       ROS_INFO_STREAM("E: CIO2");
      return false;
    }

  //  ros::Duration(0.001).sleep();
     
    ros::Time pulse_end = ros::Time::now(); 
  
    while(1)
    {
      if(!VCS_DigitalInputConfiguration(g_pKeyHandle,1,1,15,1,0,0,&ulErrorCode))
      {
        ROS_INFO_STREAM("E: CIO3");return false;
      }

      unsigned short pInputs;

      if(!VCS_GetAllDigitalInputs(g_pKeyHandle,1,&pInputs,&ulErrorCode))
      {
        ROS_INFO_STREAM("E: CIO4");return false;
      }

      if(pInputs == 32768)
      {
        pulse_end = ros::Time::now();
        ROS_INFO_STREAM("pulse start time"<< pulse_start);
        ROS_INFO_STREAM("pulse end time"<< pulse_end);
        
      }
      
      ROS_INFO_STREAM("in loop");
    } */
      return true;
  }
  

  bool ultrasonic_receiver()
  {
    if(!VCS_DigitalOutputConfiguration(g_pKeyHandle,1,1,15,1,0,0,&ulErrorCode))
     {
        ROS_INFO_STREAM("E: CIO2");
        return false;
     }

     ros::Duration(0.001).sleep();

    if(!VCS_DigitalOutputConfiguration(g_pKeyHandle,1,1,15,0,0,0,&ulErrorCode))
    {
       ROS_INFO_STREAM("E: CIO2");
      return false;
    }

    ros::Duration(0.001).sleep();

    if(!VCS_DigitalInputConfiguration(g_pKeyHandle,1,1,15,1,0,0,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: CIO3");return false;
    }

    unsigned short pInputs;

    if(!VCS_GetAllDigitalInputs(g_pKeyHandle,1,&pInputs,&ulErrorCode))
    {
      ROS_INFO_STREAM("E: CIO4");return false;
    }

    ROS_INFO_STREAM("DigitalInput"<<pInputs);
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
 // program.configure_IOs();
 // program.ultrasonic_transmitter();

 /* while (ros::ok())
  {
	//  program.ultrasonic_receiver();
	  ros::spinOnce();
	  loop_rate.sleep();
  }*/
  ros::spin();

  return 0;
}

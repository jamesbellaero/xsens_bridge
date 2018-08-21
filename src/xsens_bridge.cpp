/* 
Author: James Bell
Email : jtb2013@gmail.com
Like, don't copy this I guess. Not because it's copyrighted, but because it's highly suspicious in terms of quality.

TODO:
Make imu port configurable from an external file
Make networking port configurable from an external file
*/

#include <ros/ros.h>

#include "xsens_bridge/Imu.h"

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xssdidata.h>
#include <xsens/xstime.h>
#include <xsens/xstimestamp.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ctime>
#include <stdlib.h>
#include <unistd.h>



bool ready = false;
void readyCallback(bool r){
  ready=r;
}
ros::Publisher pub;
int main(int argc, char** argv){ 
  DeviceClass device;
  //Check command line options
  int c;
  char* cval = NULL;
  int portOut = 26015;      //First port to send data to. Rest of data is sent to subsequent ports.
  int portReady = 26100;    //Port to which program listens for ready command
  bool quat = false;        //When true, output delta-quaternion and delta-acceleration
  int rate = 100;           //IMU update rate
  bool waitReady = false;   //When false, will start sending data to port automatically
  bool logOutputs = false;  //When true, will log all data sent to ports to ~/imu_data/data_imu.txt
  while((c = getopt(argc,argv,"p:P:qr:wl"))!=-1){
  switch(c){
    case 'p'://Set portOut
      portOut = atoi(optarg);
      break;
    case 'P'://Set portReady
      portReady = atoi(optarg);
      break;
    case 'q'://Quaternion
      quat = true;
      break;
    case 'r'://set IMU update rate
      rate = std::min(atoi(optarg),100);
      break;
    case 'w'://Wait ready
      waitReady = true;
      break;
    case 'l'://Logging
      logOutputs = true;
      break;
    }
  }
  std::string portName = "/dev/ttyUSB0";
  int baudRate = 115200;
  
  // Use the device at port portName
  XsPortInfo mtPort(portName,XsBaud::numericToRate(baudRate));

  
  device.openPort(mtPort);

  bool setConfigMode = false;
  for(int i=0;i<20 && !setConfigMode;i++){
    setConfigMode = device.gotoConfig();
  }
  // Request the device Id to check the device type
  mtPort.setDeviceId(device.getDeviceId());
  // Check if we have an MTi / MTx / MTmk4 device

  XsOutputConfiguration accConfig;
  XsOutputConfiguration quatConfig;
  if(!quat){
    accConfig = XsOutputConfiguration(XDI_Acceleration, rate);
    quatConfig = XsOutputConfiguration(XDI_RateOfTurn, rate);
  }else{
    accConfig = XsOutputConfiguration(XDI_DeltaV, rate);
    quatConfig = XsOutputConfiguration(XDI_DeltaQ, rate);
  }
  XsOutputConfiguration sampleTimeConfig(XDI_SampleTimeFine, rate);
  XsOutputConfigurationArray configArray;
  configArray.push_back(accConfig);
  configArray.push_back(quatConfig);
  configArray.push_back(sampleTimeConfig);
  device.setOutputConfiguration(configArray);

  // Put the device in measurement mode
  device.gotoMeasurement();

  //std::cout << std::string(79, '-') << std::endl;

  XsByteArray data;
  XsMessageArray msgs;

  string robotName;
	ros::NodeHandle nh;
	nh.getParam("RobotName", robotName);
  ros::init(argc,argv,strcat(robotName,"/imu"));
  pub=nh.advertise<xsens_bridge::Imu>(strcat(robotName,"/imu"),1000);
  ros::Rate loop_rate(rate);

  while (ros::ok()){
    XsTimeStamp ts = XsTime::timeStampNow();
    device.readDataToBuffer(data);
    device.processBufferedData(data, msgs);
    for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it){
      // Retrieve a packet
      XsDataPacket packet;
      if ((*it).getMessageId() == XMID_MtData2) {
        packet.setMessage((*it));
        packet.setDeviceId(mtPort.deviceId());
      }

      // Get the packet data
      //int sampleTime = packet.sampleTimeFine();
      double sampleTime = ts.msTime()/1000.0;
      toPub.timeStamp = sampleTime;
      xsens_bridge::Imu toPub;
      if(quat){
        XsSdiData sdi = packet.sdiData();
        XsVector3 acc = sdi.velocityIncrement();
        XsQuaternion quat = sdi.orientationIncrement();
        toPub.acc[0] = acc.at(0);
        toPub.acc[1] = acc.at(1);
        toPub.acc[2] = acc.at(2);
        toPub.gyr[3] = quat.x();
        toPub.gyr[4] = quat.y();
        toPub.gyr[5] = quat.z();
      }else{
        XsVector3 acc = packet.calibratedAcceleration();
        XsVector3 rotRate = packet.calibratedGyroscopeData();
        toPub.acc[0] = acc.at(0);
        toPub.acc[1] = acc.at(1);
        toPub.acc[2] = acc.at(2);
        toPub.gyr[3] = rotRate.at(0);
        toPub.gyr[4] = rotRate.at(1);
        toPub.gyr[5] = rotRate.at(2);
      }
      pub.publish(toPub);
    }
    msgs.clear();
    ros::spinOnce();
    loop_rate.sleep();	
  }
}

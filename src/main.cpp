#include <iostream>
#include "bms_modbus.h"
#include <ros/ros.h>

using std::string;
using std::endl;
using std::cout;

int main(int argc, char **argv) {
  ros::init(argc, argv, "bms_node");
  ros::NodeHandle nh_priv("~");

  BMS_Modbus bms_node;
  bms_node.LoadLaunchConfig(nh_priv);
  bms_node.setup_modbus();

  ros::Rate loopRate(10);
  while (ros::ok()) {
    // inquiry battery status
    bms_node.SendReqToGetData();
    bms_node.HandleReceiveData();
    ros::spinOnce();
    loopRate.sleep();
  }
  bms_node.ModbusClose();
  return 0;
}


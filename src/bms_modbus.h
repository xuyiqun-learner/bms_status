#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <modbus/modbus.h>

using std::string;

struct rosSerialParameter {
  string port;
  string frame_id;
  string bms_topic;
  int baudrate;
  char parity;
};

class BMS_Modbus {
 public:
    BMS_Modbus() = default;
    void setup_modbus();
    void LoadLaunchConfig(ros::NodeHandle&);
    void HandleReceiveData();
    void ModbusClose();
    void SendReqToGetData();

 private:
    modbus_t *mb;
    uint8_t receive_data[64];
    int is_receive_success;
    uint8_t count_fail;
   //  uint8_t receive_data[MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t simulate_variable_req[8] =
       { 0x01, 0x03, 0x00, 0x00, 0x00, 0x1D, 0x85, 0xC3};
    uint8_t switch_variable_req[8] =
      { 0x01, 0x01, 0x00, 0x00, 0x00, 0x34, 0x3D, 0xDD };
    ros::Publisher bms_data_pub;
    rosSerialParameter bms_config;
};
#include "bms_modbus.h"

#include <sensor_msgs/BatteryState.h>
#include  <cstdlib>

void BMS_Modbus::LoadLaunchConfig(ros::NodeHandle& nh_ptr) {
  int parity;
  nh_ptr.param<string>("port", bms_config.port, "/dev/ttyUSB0");
  nh_ptr.param<string>("bms_topic", bms_config.bms_topic, "/bms/battery_data");
  nh_ptr.param("baudrate", bms_config.baudrate, 9600);
  nh_ptr.param("parity", parity, 0);
  nh_ptr.param<string>("frame_id", bms_config.frame_id, "/bms/batterystate");

  switch (parity) {
  case 0:
    bms_config.parity = 'N';  // 无校验
    break;
  case 1:
    bms_config.parity = 'O';  // 奇校验
    break;
  case 2:
    bms_config.parity = 'E';  // 偶校验
    break;
  default:
    ROS_ERROR_STREAM("invalid serial parity setting." << parity);
    break;
  }
  bms_data_pub =
      nh_ptr.advertise<sensor_msgs::BatteryState>(bms_config.bms_topic, 1);
  std::cout << "load success !!" << std::endl;
}

void BMS_Modbus::setup_modbus() {
  struct timeval response_timeout;
  struct timeval orig_response_timeout;
  // response_timeout.tv_sec = 1;
  // response_timeout.tv_usec = 0.5;
  int slaveAdr = 1;
  const int req_length = 8;
  std::cout <<  bms_config.parity << std::endl;
  mb = modbus_new_rtu(bms_config.port.c_str(),
      bms_config.baudrate, bms_config.parity, 8, 1);
  if (mb == NULL) {
    ROS_ERROR_STREAM("Unable to create the libmodbus context");
    return;
  }
  modbus_get_response_timeout(mb, &orig_response_timeout);
  std::cout << "modbus_get_response_timeout: " << orig_response_timeout.tv_sec
            << " " << orig_response_timeout.tv_usec << std::endl;
  // modbus_set_response_timeout(mb, &response_timeout);
  // modbus_set_debug(mb, TRUE); // only for debug
  modbus_set_error_recovery(mb, MODBUS_ERROR_RECOVERY_PROTOCOL);

  if (modbus_set_slave(mb, slaveAdr) != 0) {
    ROS_ERROR("modbus_set_slave fail !");
  }

  if (modbus_connect(mb) == -1) {
    ROS_ERROR_STREAM("Connection failed!!!");
    modbus_free(mb);
    return;
  }
}

void BMS_Modbus::HandleReceiveData() {
  static sensor_msgs::BatteryState bms_data;
  static sensor_msgs::BatteryState last_bms_data;
  bms_data.header.frame_id = bms_config.frame_id;
  bms_data.header.stamp.sec = ros::Time::now().toSec();
  last_bms_data.header.frame_id = bms_data.header.frame_id;

  if (receive_data[0] == 0x01) {
    if (receive_data[1] == 0x03) {
      bms_data.voltage =
          static_cast<float>(((receive_data[3] << 8) + receive_data[4]) * 0.01);
      last_bms_data.voltage = bms_data.voltage;
      bms_data.percentage =
          static_cast<float>((receive_data[7] << 8) + receive_data[8]);
      last_bms_data.percentage = bms_data.percentage;
      bms_data.capacity =
          static_cast<float>(((receive_data[9] << 8) + receive_data[10]) * 0.01);
      last_bms_data.capacity = bms_data.capacity;
      bms_data.current =
          static_cast<float>(((receive_data[11] << 8) + receive_data[12]) * 0.01);
      last_bms_data.current = bms_data.current;
      bms_data.charge = 0;
      // (static_cast<float>(((receive_data[13] << 8) + receive_data[14]) * 0.01)) * bms_data.voltage;
      last_bms_data.charge = bms_data.charge;
      bms_data.present = true;
      if (bms_data.charge > 0) {
        bms_data.power_supply_status = 1;
      }
      bms_data.power_supply_status = 3;
      last_bms_data.power_supply_status = bms_data.power_supply_status;
    } else if (receive_data[1] == true) {
      bms_data.power_supply_health = 1;
      if ((receive_data[3]& 0x01) == 0x01) {
        bms_data.power_supply_health = 3;
      } else if (((receive_data[3] & 0x40) == true) || ((receive_data[3] & 0x20) == true)) {
        bms_data.power_supply_health = 2;
      } else if (((receive_data[3] & 0x80) == true) || ((receive_data[4] & 0x01) == true)) {
        bms_data.power_supply_health = 6;
      } else if ((receive_data[4] & 0x10) == true) {
        bms_data.power_supply_health = 4;
      }
      last_bms_data.power_supply_health = bms_data.power_supply_health;
    }
    bms_data_pub.publish(bms_data);
  } else {
    if (count_fail > 8) {
        ROS_ERROR_STREAM("HandleReceiveData fail !!");
        return;
    }
    last_bms_data.header.stamp.sec = ros::Time::now().toSec();
    bms_data_pub.publish(last_bms_data);
  }
}

void BMS_Modbus::SendReqToGetData() {
  static bool inquiry_switch_flag = true;
  // static uint8_t inquiry_switch_flag = 0;
  if (inquiry_switch_flag) {
    int sr = modbus_send_raw_request(mb, simulate_variable_req, 8);
    if (sr == -1) {
      ROS_ERROR("send raw simulate request error !");
    }
    inquiry_switch_flag = false;
  } else {
    int vr = modbus_send_raw_request(mb, switch_variable_req, 8);
    if (vr == -1) {
      ROS_ERROR("send raw switch request error !");
    }
    inquiry_switch_flag = true;
  }
  // int sr = modbus_send_raw_request(mb, simulate_variable_req, 8);
  // if (sr == -1) {
  //   ROS_ERROR("send raw simulate request error !");
  // }
  std::memset(receive_data, 0, sizeof(receive_data));
  is_receive_success = modbus_receive_confirmation(mb, receive_data);
  if (is_receive_success == -1) {
    ROS_ERROR("receive_confirmation failed!");
    count_fail++;
  } else {
    count_fail = 0;
  }
}

void BMS_Modbus::ModbusClose() {
  modbus_close(mb);
  modbus_free(mb);
}
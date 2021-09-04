## Introduction

BMS driver of modbus protocol by serial， 

## Protocal Specific
(Baud: 9600bit/s, 8 data bytes, No check digit, 1 stop bit, rtu format)(specific: 485通讯协议.pdf）

### Publish
/bms/battery_data
 
### Dependency
sudo apt install libmodbus-dev 
sudo apt install ros-<ros-distro>-sensor-msgs

## Usage
```
roslaunch bms_status rion_modbus_protocol.launch
```
## Versioning


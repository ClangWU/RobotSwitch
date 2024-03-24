#include "imu_mcu/imu_mcu_control.h"
void IMUControl::set_port(int baud_rate, const std::string& _name){
    IMU_port.set_port_info(baud_rate, _name);
}
IMUControl::IMUControl(){}                    

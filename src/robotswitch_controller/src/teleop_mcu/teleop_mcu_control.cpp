#include "teleop_mcu/teleop_mcu_control.h"
void TeleopControl::set_port(int baud_rate, const std::string& _name){
    teleop_port.set_port_info(baud_rate, _name);
}
TeleopControl::TeleopControl(){}                    

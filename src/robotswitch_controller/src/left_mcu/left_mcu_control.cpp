#include "left_mcu/left_mcu_control.h"
void LeftControl::set_port(int baud_rate, const std::string& _name){
    left_port.set_port_info(baud_rate, _name);
}
LeftControl::LeftControl(){}                    

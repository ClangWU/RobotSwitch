#include <right_mcu/right_mcu_control.h>
void RightControl::set_port(int baud_rate, const std::string& _name){
    port.set_port_info(baud_rate, _name);
}

RightControl::RightControl(){}                    

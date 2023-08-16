#include "force_mcu/force_mcu_control.h"
void ForceControl::set_port(int baud_rate, const std::string& _name){
    force_port.set_port_info(baud_rate, _name);
}
ForceControl::ForceControl(){}                    

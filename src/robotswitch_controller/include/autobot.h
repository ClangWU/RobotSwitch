#pragma once
#include "ros/ros.h"

enum class State {
    CHOP,
    COLLIDE,
    RETRACT,
    ROTATE,
    FINAL,
    IDLE // 新增一个闲置状态，用于演示状态变化
};

class Autocut {
public:
    float pos_yt = 0;
    float pos_zt = 0;
    float pos_yt_1 = 0;
    float pos_yt_2 = 0;
    float pos_zt_1 = 0;
    float pos_zt_2 = 0;

    float posd_yt = 0;
    float posd_zt = 0;
    float posd_yt_1 = 0;
    float posd_zt_1 = 0;

    float fy_t = 0;
    float fy_t_1 = 0;
    float fy_t_2 = 0;

    float fz_t = 0;
    float fz_t_1 = 0;
    float fz_t_2 = 0;

    float theta = 0;
    float thetad = 0;    
    // move delta
    float move_delta_y = 0.00001;
    float move_delta_z = 0.00005;
    float epsilon = 0.00001;
    float posz_maxgap = 0.008;
    float posz_mingap = 0.0002; 
    float fz_gap = 8.0;   
    float fy_gap = 5.0;


    short posy_add_times = 0;
    short rotate_count = 0;
    short collide_count = 0;
    short collide_during_time = 0;
    bool fz_gap_flag = false;      // fz reach high value
    bool fz_keep_flag = false;     // fz keep same
    bool fy_gap_flag = false;      // fy reach high value
    bool fy_keep_flag = false;     // fy keep same

    bool posz_stuck_flag = false;  //curr z keep same
    bool posz_mingap_flag = false; // delta z reach low value    
    bool posz_maxgap_flag = false; // delta z reach high value
    bool posz_retract_flag = false; // z retract
    bool posy_retract_flag = false; // y retract
    bool rotate_done_flag = false; // rotate flag
    Autocut() : currentState(State::CHOP) {}
// obs - posy  posz   fy  fz theta
// act - posdy posdz theta
    void update(const std::vector<float>& obs, std::vector<float>& act)  {

        theta = obs[2];
        switch (currentState) {
            case State::CHOP:
                if (pos_zt - posd_zt > posz_maxgap) 
                    posz_maxgap_flag = true; // z gap reach max    
                
                if ((pos_zt_1 - pos_zt < posz_mingap) && ( pos_zt_2 - pos_zt < posz_mingap))
                    posz_stuck_flag = true; //curr z keep same
            
                if (fz_t > fz_gap && fz_t_1 > fz_gap && fz_t_2 > fz_gap)
                    fz_gap_flag = true; // fz reach high value

                // if (fy_t > fy_gap && fy_t_1 > fy_gap && fy_t_2 > fy_gap)
                

                if (posz_maxgap_flag && posz_stuck_flag && fz_gap_flag){
                    posz_maxgap_flag = false; 
                    posz_stuck_flag = false; 
                    fz_gap_flag = false;
                    currentState = State::COLLIDE; // change COLLIDE state
                }
                break;
            case State::COLLIDE:
                if (pos_zt - posd_zt > posz_maxgap) 
                    posz_maxgap_flag = true; // z gap reach max   

                if (fz_t > fz_gap && fz_t_1 > fz_gap && fz_t_2 > fz_gap){
                    collide_during_time++;
                    fz_gap_flag = true; // fz reach high value
                }
                if (collide_during_time > 10){
                    fz_keep_flag = true; // fz keep same
                }
                
                if (fz_keep_flag && posz_maxgap_flag)
                    currentState = State::RETRACT; // change RETRACT state

                break;                    
            case State::RETRACT:
                if (abs(pos_zt - posd_zt) < posz_mingap) 
                    posz_mingap_flag = true; // z gap reach min

                if (posz_mingap_flag && posz_retract_flag && posy_retract_flag){
                    rotate_count++;
                    currentState = State::ROTATE;
                }
                break;          
            case State::ROTATE:
                if (rotate_count == 1 && theta > 5.0)
                        rotate_done_flag = true; // rotate done
                else if (rotate_count == 2 && theta > 10.0)
                        rotate_done_flag = true; // rotate done
                else if (rotate_count == 3 && theta > 15.0)
                        rotate_done_flag = true; // rotate done
                
                if (rotate_done_flag){
                    rotate_done_flag = false;
                    currentState = State::CHOP;
                }
                break;
            }        

        performAction(act); // 基于当前状态执行动作
    }

private:
    State currentState;
    State nextState;
    void performAction(std::vector<float>& act) {
        switch (currentState) {
            case State::CHOP:
                chopAction(act);
                break;
            case State::COLLIDE:
                collideAction(act);
                break;
            case State::ROTATE:
                rotateAction(act);
                break;
            case State::RETRACT:
                retractAction(act);
                break;
            case State::FINAL:
                std::cout << "Reached final state." << std::endl;
                break;
            case State::IDLE:
                // 当对象处于闲置状态时不执行任何操作
                break;
        }
    }
// act - posdy posdz theta

    void chopAction(std::vector<float>& act) {
        std::cout << "\033[1;32mPerforming chop action\033[0m" << std::endl;
        if (currentState == State::FINAL){
            posd_yt = posd_yt;
            posd_zt = posd_zt;
            thetad = 5.0;
            act[0] = posd_yt;
            act[1] = posd_zt;
            act[2] = thetad; // first time try 5.0
        }
        else{
                if (rotate_count == 0){
                posd_yt = 0;
                posd_zt = posd_zt + move_delta_z;
                thetad = 0.0;
                act[0] = posd_yt;
                act[1] = posd_zt;
                act[2] = thetad;
            } else if (rotate_count == 1){
                posd_yt = posd_yt + move_delta_y;
                posd_zt = posd_zt - move_delta_z;
                thetad = 5.0 + 1.0; // 1.0 to compensate the command
                act[0] = posd_yt;
                act[1] = posd_zt;
                act[2] = thetad;
            } else if (rotate_count == 2){
                posd_yt = posd_yt + move_delta_y;
                posd_zt = posd_zt - move_delta_z;
                thetad = 10.0 +1.0; // 1.0 to compensate the command
                act[0] = posd_yt;
                act[1] = posd_zt;
                act[2] = thetad;
            }else if (rotate_count == 3){
                posd_yt = posd_yt + move_delta_y;
                posd_zt = posd_zt - move_delta_z;
                thetad = 15.0 + 1.0; // 1.0 to compensate the command
                act[0] = posd_yt;
                act[1] = posd_zt;
                act[2] = thetad;
            }
        }
    }

    void retractAction(std::vector<float>& act) {
        std::cout << "\033[1;34mPerforming retract action\033[0m" << std::endl;
        if (!posz_retract_flag)
        {
            if (posd_zt < pos_zt){
                posd_zt = posd_zt + move_delta_z;
            }else{
                posd_zt = posd_zt + 4 * move_delta_z; // retract more to avoid collision
                posz_retract_flag = true;
            }
        }else{
            posd_zt = posd_zt;
        }
        
        if (!posy_retract_flag)
        {
            if (posd_yt > pos_yt){
                posd_yt = posd_yt - move_delta_y;
            }else{
                posd_yt = posd_yt - 4 * move_delta_y; // retract more to avoid collision
                posy_retract_flag = true;
            }        
        }else{
            posd_yt = posd_yt;
        }
        act[0] = posd_yt;
        act[1] = posd_zt;
        act[2] = thetad;
    }

    void rotateAction(std::vector<float>& act) {
        std::cout << "\033[1;33mPerforming rotate action\033[0m" << std::endl;
        thetad = thetad + 0.1;
        act[0] = posd_yt;
        act[1] = posd_zt;
        act[2] = thetad;
    }

    void collideAction(std::vector<float>& act) {
        std::cout << "\033[1;31mPerforming collide action\033[0m" << std::endl;
        act[0] = posd_yt;
        act[1] = posd_zt;
        act[2] = thetad;
    }
};
#pragma once
#include "ros/ros.h"

enum class State {
    CHOP,
    COLLIDE,
    RETRACT,
    ROTATE,
    TILT_COLLIDE,
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

    float posy_retract_point = 101.0;
    float posz_retract_point = 101.0;
    // move delta
    float move_delta_y = 0.00002;
    float move_delta_z = 0.00008;
    float epsilon = 0.00002;
    float posz_maxgap = 0.008;
    float posz_mingap = 0.0002; 
    float posy_retract_distance = 0.005;
    float posz_retract_distance = 0.01;

    float fy_before_collide = 101.0;
    float fz_before_collide = 101.0;
    float theta_before_rotate = 101.0;
    float posy_before_chop = 101.0;
    float posz_before_chop = 101.0;
    float posy_track = 0;
    float rotate_delta = 0.1;
    float fz_max = 8.0;            
    float fy_gap = 4.0;                    

    short posy_add_times = 0;
    short rotate_count = 0;
    short collide_count = 0;
    short collide_during_time = 0;
    short collide_during_time_max = 2;
    bool posz_keep_going_flag = false;
    bool fz_max_flag = false;      // fz reach high value
    bool fz_keep_flag = false;     // fz keep same
    bool fy_gap_flag = false;      // fy reach high value
    bool fy_keep_flag = false;     // fy keep same
    bool tilt_collide_flag = false; // tilt collide flag
    bool chop_anyway_flag = false;

    bool posz_stuck_flag = false;  //curr z keep same
    bool posz_mingap_flag = false; // delta z reach low value    
    bool posz_maxgap_flag = false; // delta z reach high value
    bool posz_retract_flag = false; // z retract
    bool posy_retract_flag = false; // y retract
    bool rotate_done_flag = false; // rotate flag
    Autocut() : currentState(State::CHOP) {}
// obs - posy  posz  fy  fz theta
// act - posdy posdz theta
    void update(const std::vector<float>& obs, std::vector<float>& act)  {
        if (obs.size() < 5) {
            std::cout << "obs size" << obs.size() << std::endl;
            std::cerr << "Invalid observation data." << std::endl;
            return; // 或者其他错误处理
        }else{
            pos_yt_2 = pos_yt_1;
            pos_yt_1 = pos_yt;
            pos_yt = obs[0];
            pos_zt_2 = pos_zt_1;
            pos_zt_1 = pos_zt;
            pos_zt = obs[1];
            fy_t_2 = fy_t_1;
            fy_t_1 = fy_t;
            fy_t = obs[2];
            fz_t_2 = fz_t_1;
            fz_t_1 = fz_t;
            fz_t = obs[3];
            theta = obs[4];
        }
        switch (currentState) {
            case State::CHOP:
                if (fy_before_collide > 100.0 ){
                    fy_before_collide = fy_t;
                }
                if (posz_before_chop > 100.0){
                    posz_before_chop = pos_zt;
                }
                if (posz_before_chop - pos_zt > 0.015)
                    posz_keep_going_flag = true; // z keep going down

                if (pos_zt - posd_zt > posz_maxgap) 
                    posz_maxgap_flag = true; // z gap reach max    
                
                if ((pos_zt_1 - pos_zt < posz_mingap) && ( pos_zt_2 - pos_zt < posz_mingap))
                    posz_stuck_flag = true; //curr z keep same
            
                if (fz_t > fz_max && fz_t_1 > fz_max && fz_t_2 > fz_max)
                    fz_max_flag = true; // fz reach high value

                // if (fy_t > fy_gap && fy_t_1 > fy_gap && fy_t_2 > fy_gap)
                
                if (fy_t > fy_before_collide + fy_gap){
                    fy_gap_flag = true; // fy reach high value
                }
                if (posz_maxgap_flag && posz_stuck_flag && fz_max_flag){
                    posz_maxgap_flag = false; 
                    posz_stuck_flag = false; 
                    fz_max_flag = false;
                    theta_before_rotate = 101.0;
                    posz_before_chop = 101.0;
                    fy_before_collide = 101.0;
                    currentState = State::COLLIDE; // change COLLIDE state
                }

                if (fy_gap_flag){
                    fy_gap_flag = false;
                    fy_before_collide = 101.0;
                    theta_before_rotate = 101.0;
                    posz_before_chop = 101.0;
                    currentState = State::TILT_COLLIDE; // change TILT_COLLIDE state
                }

                if (tilt_collide_flag && chop_anyway_flag)
                {
                    tilt_collide_flag = false;
                    currentState = State::FINAL; // change TILT_COLLIDE state
                }
                
                if (rotate_count != 0 && posz_keep_going_flag)
                {
                    currentState = State::FINAL; // change TILT_COLLIDE state
                }
                
                break;

            case State::TILT_COLLIDE:
                currentState = State::RETRACT;
                tilt_collide_flag = true;
                break;
            
            case State::COLLIDE:
                if (pos_zt - posd_zt > posz_maxgap) 
                    posz_maxgap_flag = true; // z gap reach max   

                if (fz_t > fz_max && fz_t_1 > fz_max && fz_t_2 > fz_max){
                    collide_during_time++;
                    fz_max_flag = true; // fz reach high value
                }

                if (collide_during_time > collide_during_time_max){
                    fz_keep_flag = true; // fz keep same
                }
                
                if (fz_keep_flag && posz_maxgap_flag && fz_max_flag)
                {
                    posz_maxgap_flag = false;
                    fz_keep_flag = false;
                    fz_max_flag = false;
                    currentState = State::RETRACT; // change RETRACT state
                }
                break;                    
            case State::RETRACT:

                if (posz_retract_flag && posy_retract_flag){
                    rotate_count++;
                    posz_retract_flag = false;
                    posy_retract_flag = false;
                    posy_retract_point = 101.0;
                    posz_retract_point = 101.0;
                    currentState = State::ROTATE;
                }
                break;          
            case State::ROTATE:
                if (!tilt_collide_flag)
                {
                    if (rotate_count == 1 && theta > 5.0)
                            rotate_done_flag = true; // rotate done
                    else if (rotate_count == 2 && theta > 10.0)
                            rotate_done_flag = true; // rotate done
                    else if (rotate_count == 3 && theta > 15.0)
                            rotate_done_flag = true; // rotate done
                }else{
                    if(theta < 2.0)
                        rotate_done_flag = true;
                }

                if (rotate_done_flag){
                    rotate_done_flag = false;
                    // tilt_collide_flag =false;
                    theta_before_rotate = 101.0;
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
            case State::TILT_COLLIDE:
                tiltCollideAction(act);
                break;
            case State::ROTATE:
                rotateAction(act);
                break;
            case State::RETRACT:
                retractAction(act);
                break;
            case State::FINAL:
                std::cout << "Reached final state." << std::endl;
                chopAction(act);
                break;
            case State::IDLE:
                // 当对象处于闲置状态时不执行任何操作
                break;
        }
    }
// act - posdy posdz theta

    void chopAction(std::vector<float>& act) {
        std::cout << "\033[1;32mPerforming chop action\033[0m" << std::endl;
        if (posy_before_chop > 100)
            posy_before_chop = pos_yt;
        
        if (currentState == State::FINAL){
            posd_zt = posd_zt - move_delta_z;
                if (theta > 0.0){
                    thetad = thetad - rotate_delta; 
                }else{
                    if (thetad < -1.0)
                    {
                        thetad = + rotate_delta;
                    } 
                }  
        }
        else{
            if (tilt_collide_flag){
                if (pos_yt < posy_before_chop + 0.01){
                    posd_yt = posd_yt + move_delta_y;
                    posy_track = pos_yt;
                }
                else{
                    posd_yt = posy_track + 0.012;
                    chop_anyway_flag = true;
                }

                posd_zt = posd_zt - move_delta_z;
                thetad = 0.0;
            }else{
                switch (rotate_count) {
                    case 0:
                        posd_yt = 0;
                        posd_zt = posd_zt - move_delta_z;
                        thetad = 0.0;
                        break;
                    case 1:
                        posd_yt = posd_yt + move_delta_y;
                        posd_zt = posd_zt - move_delta_z;
                        if (thetad > theta + 1.0)
                        {
                            thetad = thetad - 0.1; // 1.0 to compensate the command
                        }
                        break;
                    case 2:
                        posd_yt = posd_yt + move_delta_y;
                        posd_zt = posd_zt - move_delta_z;
                        if (thetad > theta + 1.0)
                        {
                            thetad = thetad - 0.1; // 1.0 to compensate the command
                        }                        break;
                    case 3:
                        posd_yt = posd_yt + move_delta_y;
                        posd_zt = posd_zt - move_delta_z;
                        if (thetad > theta + 1.0)
                        {
                            thetad = thetad - 0.1; // 1.0 to compensate the command
                        }                        break;
                    default:
                        // 可以添加一些处理未知rotate_count值的代码
                        break;
                }
            }
        }
            act[0] = posd_yt; 
            act[1] = posd_zt;
            act[2] = thetad; 
    }

    void retractAction(std::vector<float>& act) {
        std::cout << "\033[1;34mPerforming retract action\033[0m" << std::endl;
        if (posy_retract_point > 100){
            posy_retract_point = pos_yt;
        }
        if (posz_retract_point > 100){
            posz_retract_point = pos_zt;
            std::cout << posz_retract_point << std::endl;
        }
        
        if (!posz_retract_flag)
        {
            if (posd_zt < posz_retract_point + posz_retract_distance){
                posd_zt = posd_zt + move_delta_z;
                // std::cout << posz_retract_point + posz_retract_distance << std::endl;
            }else
                posz_retract_flag = true;
        }
        
        if (!posy_retract_flag)
        {
            if (posd_yt < posy_retract_point + posy_retract_distance)
                posd_yt = posd_yt + move_delta_y;
            else
                posy_retract_flag = true;
        }
        act[0] = posd_yt;
        act[1] = posd_zt;
        act[2] = thetad;
    }

    void rotateAction(std::vector<float>& act) {
        std::cout << "\033[1;33mPerforming rotate action\033[0m" << std::endl;
        if(theta_before_rotate > 100.0){
            theta_before_rotate = theta;
        }
        if (tilt_collide_flag){
            if (theta > 0.0)
                    thetad = thetad - rotate_delta;                
        }
        else{
                thetad = thetad + rotate_delta;  
        }

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

    void tiltCollideAction(std::vector<float>& act){
        std::cout << "\033[1;35mPerforming tilt collide action\033[0m" << std::endl;
        act[0] = posd_yt;
        act[1] = posd_zt;
        act[2] = thetad;
    }
};
#pragma once
#include "ros/ros.h"
#include <fstream>

enum class State {
    CHOP,
    RETRACT,
    ROTATE,
    FINAL,
    COLLIDE,
    TILT_COLLIDE,
    IDLE 
};

class Autocut {
public:
    std::string state_path;
    std::ofstream state_file;
    std::string act_path;
    std::ofstream act_file;
    std::string epi_path;
    std::ofstream epi_file;

    std::string _state_path;
    std::ofstream _state_file;
    std::string _act_path;
    std::ofstream _act_file;
    std::string _epi_path;
    std::ofstream _epi_file;
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

    float abs_posz = 0.0;
    float posy_retract_point = 101.0;
    float posz_retract_point = 101.0;
    // move delta
    float move_delta_y = 0.00002;
    float move_delta_z = 0.00008;
    float epsilon = 0.00002;
    float posz_maxgap = 0.012;   
    float posz_mingap = 0.0002; 
    float posy_retract_distance = 0.004;
    float posz_retract_distance = 0.008;

    float fy_before_collide = 101.0;
    float fz_before_collide = 101.0;
    float theta_before_rotate = 101.0;
    float posz_keep_going_threshold = 0.018;
    float posy_before_chop = 101.0;
    float posz_before_chop = 101.0;
    float posy_track = 0;
    float rotate_delta = 0.2;
    float fz_max = 12.0;            
    float fy_gap = 3.0;                    
    float fz_gap = 3.0;

    float rotate_angle1 = 5.0;
    float rotate_angle2 = 10.0;
    float rotate_angle3 = 15.0;

    short action_steps_10 = 0;
    short action_steps = 0;
    short posy_add_times = 0;
    short rotate_count = 0;
    short collide_count = 0;
    short collide_during_time = 0;
    short collide_during_time_max = 2;
    bool cut_over_flag = false;
    bool from_chop_flag = false;
    bool posz_keep_going_flag = false;
    bool fz_max_flag = false;      // fz reach high value
    bool fz_keep_flag = false;     // fz keep same
    bool fy_gap_flag = false;      // fy reach high value
    bool fz_gap_flag = false;      // fz reach high value

    bool fy_keep_flag = false;     // fy keep same
    bool tilt_collide_flag = false; // tilt collide flag
    bool tilt_action_posdy_flag = false;
    bool tilt_action_therad_flag = false;
    bool print_cut_finish_flag = false;
    bool posz_stuck_flag = false;  //curr z keep same 
    bool posz_mingap_flag = false; // delta z reach low value    
    bool posz_maxgap_flag = false; // delta z reach high value
    bool posz_retract_flag = false; // z retract
    bool posy_retract_flag = false; // y retract
    bool rotate_done_flag = false; // rotate flag
    Autocut() : currentState(State::CHOP) {
            std::string state_path = "/home/yzc/project/robotswitch/src/robotswitch_controller/data/50hz_obs_t26.csv";
            state_file = std::ofstream(state_path, std::ios::out);
            state_file << "observation" << std::endl;
            std::string act_path = "/home/yzc/project/robotswitch/src/robotswitch_controller/data/50hz_act_t26.csv";
            act_file = std::ofstream(act_path, std::ios::out);
            act_file << "action" << std::endl;
            std::string epi_path = "/home/yzc/project/robotswitch/src/robotswitch_controller/data/50hz_episode_t26.csv";
            epi_file = std::ofstream(epi_path, std::ios::out);
            epi_file << "episode_ends" << std::endl;
            
            std::string _state_path = "/home/yzc/project/robotswitch/src/robotswitch_controller/data/10hz_obs_t26.csv";
            _state_file = std::ofstream(_state_path, std::ios::out);
            _state_file << "observation" << std::endl;
            std::string _act_path = "/home/yzc/project/robotswitch/src/robotswitch_controller/data/10hz_act_t26.csv";
            _act_file = std::ofstream(_act_path, std::ios::out);
            _act_file << "action" << std::endl;
            std::string _epi_path = "/home/yzc/project/robotswitch/src/robotswitch_controller/data/10hz_episode_t26.csv";
            _epi_file = std::ofstream(_epi_path, std::ios::out);
            _epi_file << "episode_ends" << std::endl;
    }
// obs - posy  posz  fy  fz theta
// act - posdy posdz theta
    void update(const std::vector<float>& obs, std::vector<float>& act)  {
        if (obs.size() < 6) {
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
            abs_posz = obs[5];

            if (abs_posz > 0.14){
                std::cout << "\033[1;31mNot in a cut height(<0.14) \033[0m" << std::endl;
                return; 
            }
            if (abs_posz < 0.03){
                if (!print_cut_finish_flag){   
                    epi_file <<  action_steps;
                    _epi_file << action_steps_10;
                    print_cut_finish_flag = true;
                    std::cout << " whole steps " << action_steps << std::endl; 
                    std::cout << " whole times " << (float)action_steps * 0.02 << std::endl; 
                    std::cout << "\033[1;31mCutting Finished.\033[0m" << std::endl;
                }
                
                cut_over_flag = true;
                actionAfterCut(act);
            }
        }
        switch (currentState) {
            case State::CHOP:
                if (fy_before_collide > 100.0 ){
                    fy_before_collide = fy_t;
                }

                if (fz_before_collide > 100.0 ){
                    fz_before_collide = fz_t;
                }

                if (posz_before_chop > 100.0){
                    posz_before_chop = pos_zt;
                    std::cout << "posz_before_chop" << posz_before_chop << std::endl;
                }
                if (posz_before_chop - pos_zt > posz_keep_going_threshold){
                    posz_keep_going_flag = true; // z keep going down
                }

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

                if (fz_t > fz_before_collide + fz_gap){
                    fz_gap_flag = true; // fy reach high value
                }

                if (posz_maxgap_flag && posz_stuck_flag && fz_max_flag){
                    posz_maxgap_flag = false; 
                    posz_stuck_flag = false; 
                    fz_max_flag = false;
                    theta_before_rotate = 101.0;
                    posy_before_chop = 101.0;
                    posz_before_chop = 101.0;
                    fy_before_collide = 101.0;
                    currentState = State::COLLIDE; // change COLLIDE state
                }

                if (fy_gap_flag && fz_gap_flag){
                    fy_gap_flag = false;
                    fz_gap_flag = false;
                    fy_before_collide = 101.0;
                    theta_before_rotate = 101.0;
                    posy_before_chop = 101.0;
                    posz_before_chop = 101.0;
                    currentState = State::TILT_COLLIDE; // change TILT_COLLIDE state
                }

                if (tilt_collide_flag && tilt_action_posdy_flag && tilt_action_therad_flag ){
                    tilt_collide_flag = false;
                    from_chop_flag = true;
                    // posy_before_chop = 101.0;
                    std::cout << "after tilt final" << std::endl;
                    currentState = State::RETRACT; 
                }
                
                if (rotate_count != 0 && posz_keep_going_flag){
                    std::cout << "keep going and retract" << std::endl;
                    currentState = State::RETRACT; // change TILT_COLLIDE state
                    from_chop_flag = true;
                }else
                    posz_keep_going_flag =  false;

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
                    if (!from_chop_flag){
                        rotate_count++;
                        posz_retract_flag = false;
                        posy_retract_flag = false;
                        posy_retract_point = 101.0;
                        posz_retract_point = 101.0;
                        currentState = State::ROTATE;              
                    }else{
                        from_chop_flag = false;
                        currentState = State::FINAL;
                    }
                }
                break;          
            case State::ROTATE:
                if (!tilt_collide_flag)
                {
                    if (rotate_count == 1 && theta > rotate_angle1)
                            rotate_done_flag = true; // rotate done
                    else if (rotate_count == 2 && theta > rotate_angle2)
                            rotate_done_flag = true; // rotate done
                    else if (rotate_count == 3 && theta > rotate_angle3)
                            rotate_done_flag = true; // rotate done
                }else{
                    if(theta < 5.0){
                        rotate_done_flag = true;
                        std::cout << "tilt rotate done" << std::endl;
                    }
                }

                if (rotate_done_flag){
                    rotate_done_flag = false;
                    // tilt_collide_flag =false;
                    theta_before_rotate = 101.0;
                    currentState = State::CHOP;
                }
                break;
            }        
        if (cut_over_flag)
        {
            return;
        }else{
            if (currentState == State::COLLIDE || currentState == State::TILT_COLLIDE) 
            {
                recordState = State::RETRACT;
            }
            else
                recordState = currentState;

            if (action_steps %5 == 0)
            {
                action_steps_10 ++;
                _state_file  << "\"[";

                _state_file  << static_cast<int>(recordState) << ", "
                            << pos_yt  << ", "
                            << posd_yt << ", "
                            << pos_zt  << ", "
                            << posd_zt << ", "
                            << theta   << ", "
                            << thetad  << ", "
                            << fy_t    << ", "
                            << fz_t    << "]\"";
                _state_file << std::endl;
            }
                state_file  << "\"[";
                state_file  << static_cast<int>(recordState) << ", "
                            << pos_yt  << ", "
                            << posd_yt << ", "
                            << pos_zt  << ", "
                            << posd_zt << ", "
                            << theta   << ", "
                            << thetad  << ", "
                            << fy_t    << ", "
                            << fz_t    << "]\"";
                state_file << std::endl;
            performAction(act); // 基于当前状态执行动作

            if (action_steps %5 == 0)
            {
                _act_file << "\"[";
                _act_file << posd_yt << ", "
                        << posd_zt << ", "
                        << thetad  << "]\"";
                _act_file << std::endl;
            }

            act_file << "\"[";
            act_file << posd_yt << ", "
                     << posd_zt << ", "
                     << thetad  << "]\"";
            act_file << std::endl;

            action_steps ++;

        }
    }

private:
    State currentState;
    State recordState;
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
                // std::cout << "Reached final state." << std::endl;
                chopAction(act);
                break;
            case State::IDLE:
                // 当对象处于闲置状态时不执行任何操作
                break;
        }
    }
// act - posdy posdz theta

    void chopAction(std::vector<float>& act) {
        if(currentState == State::FINAL)
            std::cout << "\033[1;32mPerforming FINAL action\033[0m" << std::endl;
        else
            std::cout << "\033[1;32mPerforming chop action\033[0m" << std::endl;
        if (posy_before_chop > 100)
            posy_before_chop = pos_yt;
        
        if (currentState == State::FINAL){
            posd_zt = posd_zt - move_delta_z * 3;
            posd_yt = posd_yt - move_delta_y * 1.5;

                if (theta > 0.0){
                    thetad = thetad - rotate_delta; 
                }else{
                    if (thetad < -2.0)
                    { 
                        thetad = thetad + rotate_delta;
                    } 
                }  
                // if (thetad < -12)
                // {
                //   thetad = -12.0;     
                // }   
        }
        else{
            if (tilt_collide_flag){
                    // std::cout << pos_yt << std::endl;
                    // std::cout << posy_before_chop << std::endl;
                if (pos_yt < posy_before_chop + 0.005){
                    posd_yt = posd_yt + move_delta_y;
                    posy_track = pos_yt;
                }
                else{
                    posd_yt = posd_yt;
                    tilt_action_posdy_flag = true;
                }

                posd_zt = posd_zt - move_delta_z;

                if (thetad < 0.0)
                {
                    thetad = thetad + rotate_delta; // 1.0 to compensate the command
                }else{
                    tilt_action_therad_flag = true;
                }

            }else{
                switch (rotate_count) {
                    case 0:
                        posd_yt = 0;
                        posd_zt = posd_zt - move_delta_z * 3;
                        thetad = 0.0;
                        break;
                    case 1:
                        posd_yt = posd_yt + move_delta_y * 2;
                        posd_zt = posd_zt - move_delta_z * 2;
                        if (thetad > theta + 1.0)
                        {
                            thetad = thetad - 0.1; // 1.0 to compensate the command
                        }
                        break;
                    case 2:
                        posd_yt = posd_yt + move_delta_y * 2;
                        posd_zt = posd_zt - move_delta_z * 2;
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
        }
        
        if (!posz_retract_flag)
        {
            if (posd_zt < posz_retract_point + posz_retract_distance){
                posd_zt = posd_zt + move_delta_z * 3 ;
                // std::cout << posz_retract_point + posz_retract_distance << std::endl;
            }else
                posz_retract_flag = true;
        }
        
        if (!posy_retract_flag)
        {
            if (!from_chop_flag)
            {
                if (posd_yt < posy_retract_point + posy_retract_distance)
                    posd_yt = posd_yt + move_delta_y ;
                else
                    posy_retract_flag = true;
            }else{
                    posy_retract_flag = true;
            }
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
                // if(rotate_count == 1){
                //     if(thetad > rotate_angle1+12)
                //         thetad = rotate_angle1+12;
                // }else if(rotate_count == 2){
                //     if(thetad > rotate_angle2+12)
                //         thetad = rotate_angle2+12;
                // }else if(rotate_count == 3){
                //     if(thetad > rotate_angle3+12)
                //         thetad = rotate_angle3+12;
                // }
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

    void actionAfterCut(std::vector<float>& act){
        if(pos_yt < 0.06)
            posd_yt += 0.0005; 
        if(abs_posz < 0.033)
            posd_zt += 0.00008; 
        if(theta < 2)
            thetad = thetad + 0.2;
        act[0] = posd_yt;
        act[1] = posd_zt;
        act[2] = thetad;
    }
};


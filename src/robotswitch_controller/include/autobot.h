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
    double pos_yt = 0;
    double pos_zt = 0;
    double pos_yt_1 = 0;
    double pos_yt_2 = 0;
    double pos_zt_1 = 0;
    double pos_zt_2 = 0;

    double posd_yt = 0;
    double posd_zt = 0;
    double posd_yt_1 = 0;
    double posd_zt_1 = 0;

    double fy_t = 0;
    double fy_t_1 = 0;
    double fy_t_2 = 0;

    double fz_t = 0;
    double fz_t_1 = 0;
    double fz_t_2 = 0;
    double theta = 0;
    // move delta
    double move_delta_y = 0.000001;
    double move_delta_z = 0.000005;
    double posz_maxgap = 0.008;
    double posz_mingap = 0.0005 ; 
    double fz_gap = 10.0;     

    bool fz_gap_flag = false;      // fz reach high value
    bool posz_stuck_flag = false;  //curr z keep same
    bool posz_mingap_flag = false; // delta z reach low value    
    bool posz_maxgap_flag = false; // delta z reach high value

    Autocut() : currentState(State::CHOP) {}
// obs - posy  posz    fy    fz
// act - posdy posdz theta
    void update(double *obs, double *act) {
        switch (currentState) {
            case State::CHOP:
                if (pos_zt - posd_zt > posz_gap){
                    posz_maxgap_flag = true;
                }

                currentState = State::COLLIDE;
                break;
            case State::COLLIDE:
                currentState = State::RETRACT;
                break;                    
            case State::RETRACT:
                currentState = State::ROTATE;
                break;          
            case State::ROTATE:
                currentState = State::CHOP;
                break;

            }        

        performAction(act); // 基于当前状态执行动作
    }

private:
    State currentState;
    State nextState;
    void performAction(double *act) {
        switch (currentState) {
            case State::CHOP:
                chopAction();
                break;
            case State::COLLIDE:
                collideAction();
                break;
            case State::ROTATE:
                rotateAction();
                break;
            case State::FINAL:
                std::cout << "Reached final state." << std::endl;
                break;
            case State::IDLE:
                // 当对象处于闲置状态时不执行任何操作
                break;
        }
    }

    void chopAction() {
        std::cout << "Performing chop action" << std::endl;
    }

    void collideAction() {
        std::cout << "Performing collide action" << std::endl;
    }

    void rotateAction() {
        std::cout << "Performing rotate action" << std::endl;
    }
};
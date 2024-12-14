#ifndef __FILTER_H
#define __FILTER_H
#include "filter_math.h"
#ifdef __cplusplus
#include "math_utils.hpp"
#include "iostream"
using namespace RobotSwitch;
extern "C" {
#endif

struct filter_s;
typedef struct filter_s filter_t;

typedef struct pt1Filter_s
{
    float state;
    float k;
} pt1Filter_t;

typedef struct pt2Filter_s
{
    float state;
    float state1;
    float k;
} pt2Filter_t;

typedef struct pt3Filter_s
{
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;

typedef struct slewFilter_s
{
    float state;
    float slewLimit;
    float threshold;
} slewFilter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s
{
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} BiquadFilter_t;

typedef struct laggedMovingAverage_s
{
    uint16_t movingWindowIndex;
    uint16_t windowSize;
    float movingSum;
    float *buf;
    uint8_t primed;
} laggedMovingAverage_t;

typedef enum
{
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

typedef enum
{
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

float nullFilterApply(filter_t *filter, float input);

void biquadFilterInitLPF(BiquadFilter_t *filter, float _filterCallFreq, float _cutoffFreq);

void biquadFilterInit(BiquadFilter_t *filter, float _filterCallFreq, float _cutoffFreq, float Q,
                      biquadFilterType_e filterType);

void biquadFilterUpdate(BiquadFilter_t *filter,float _filterCallFreq, float _cutoffFreq, float Q,
                        biquadFilterType_e filterType);

void biquadFilterUpdateLPF(BiquadFilter_t *filter, float filterFreq, float refreshRate);

float biquadFilterApplyDF1(BiquadFilter_t *filter, float input);

float biquadFilterApply(BiquadFilter_t *filter, float input);

float filterGetNotchQ(float centerFreq, float cutoffFreq);

void laggedMovingAverageInit(laggedMovingAverage_t *filter, uint16_t windowSize, float *buf);

float laggedMovingAverageUpdate(laggedMovingAverage_t *filter, float input);

float pt1FilterGain(float f_cut, float dT);

void pt1FilterInit(pt1Filter_t *filter, float k);

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);

float pt1FilterApply(pt1Filter_t *filter, float input);

float pt2FilterGain(float f_cut, float dT);

void pt2FilterInit(pt2Filter_t *filter, float k);

void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k);

float pt2FilterApply(pt2Filter_t *filter, float input);

float pt3FilterGain(float f_cut, float dT);

void pt3FilterInit(pt3Filter_t *filter, float k);

void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k);

float pt3FilterApply(pt3Filter_t *filter, float input);

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold);

float slewFilterApply(slewFilter_t *filter, float input);

#ifdef __cplusplus
    class HighPassFilter {
        private:
            double prev_input1 = 0.0;
            double prev_output1 = 0.0;
            double prev_input2[2] = {0.0, 0.0};
            double prev_output2[2] = {0.0, 0.0};
            double a0, a1, a2, b0, b1, b2;
            Eigen::Vector3d prev_input1_3f;
            Eigen::Vector3d prev_output1_3f;
            Eigen::Vector3d prev_input2_3f[2];
            Eigen::Vector3d prev_output2_3f[2];
            int order_ = 1;
        public:
            HighPassFilter(){};
            ~HighPassFilter(){};
            void InitFilter(int order, double sampleRate, double cutoffFreq) {
                order_ = order;
                if(order_ == 1){
                    prev_input1_3f = Eigen::Vector3d::Zero();
                    prev_output1_3f= Eigen::Vector3d::Zero();
                    
                    double dt = 1.0 / sampleRate;
                    double rc = 1.0 / (2 * M_PI * cutoffFreq);
                    double alpha = rc / (rc + dt);

                    a0 = 1;
                    a1 = -alpha;
                    b0 = alpha;
                    b1 = -alpha;
                }else{
                    for (size_t i = 0; i < order_; i++)
                    {
                        prev_input2_3f[i] = Eigen::Vector3d::Zero();
                        prev_output2_3f[i]= Eigen::Vector3d::Zero();
                    }

                    double omega = 2.0 * M_PI * cutoffFreq / sampleRate;
                    double alpha = sin(omega) / (2.0 * sqrt(2.0));

                    a0 = 1.0 + alpha;

                    b0 = (1.0 + cos(omega)) / 2.0 / a0;
                    b1 = -(1.0 + cos(omega)) / a0;
                    b2 = b0;

                    a1 = -2.0 * cos(omega) / a0;
                    a2 = (1.0 - alpha) / a0;
                }
            }

            double process(double input) {
                if(order_ == 1){
                    double output = b0 * input + b1 * prev_input1 - a1 * prev_output1;

                    prev_input1 = input;
                    prev_output1 = output;
                    return output;
                }else{
                    double output = b0 * input + b1 * prev_input2[0] + b2 * prev_input2[1]
                        - a1 * prev_output2[0] - a2 * prev_output2[1];

                    prev_input2[1] = prev_input2[0];
                    prev_input2[0] = input;

                    prev_output2[1] = prev_output2[0];
                    prev_output2[0] = output;
                    return output;
                }
            }

            Eigen::Vector3d process(const Eigen::Vector3d input) {
                if(order_ == 1){
                    Eigen::Vector3d output = b0 * input + b1 * prev_input1_3f - a1 * prev_output1_3f;

                    prev_input1_3f = input;
                    prev_output1_3f = output;
                    return output;
                }else{
                    
                    Eigen::Vector3d output = b0 * input + b1 * prev_input2_3f[0] + b2 * prev_input2_3f[1]
                                - a1 * prev_output2_3f[0] - a2 * prev_output2_3f[1];

                    prev_input2_3f[1] = prev_input2_3f[0];
                    prev_input2_3f[0] = input;

                    prev_output2_3f[1] = prev_output2_3f[0];
                    prev_output2_3f[0] = output;
                    return output;
                }
            }
        };
}
#endif
#endif

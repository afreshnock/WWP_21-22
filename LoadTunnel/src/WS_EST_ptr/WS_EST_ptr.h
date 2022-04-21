/*
  WS_EST.h 
*/
#ifndef WS_EST_ptr_h
#define WS_EST_ptr_h

#include "Arduino.h"

class WindspeedEstimation
{
  public:
    WindspeedEstimation(uint16_t *rpm, float *theta, uint16_t *l_current, uint16_t *l_voltage, uint16_t *l_power);
    double linear();
    double secondOrder();
    double bestFit();
  private:
    uint16_t *_ptrrpm;
    float *_ptrtheta;
    uint16_t *_ptrl_current;
    uint16_t *_ptrl_voltage;
    uint16_t *_ptrl_power;
    void updateVars();
    double _rpm;
    double _theta;
    double _l_voltage;
    double _l_current;
    double _l_power;
};

#endif
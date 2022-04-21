/*
  WS_EST.h 
*/
#ifndef WS_EST_h
#define WS_EST_h

#include "Arduino.h"

class WindspeedEstimation
{
  public:
    WindspeedEstimation();
    double linear(uint16_t rpm, float theta, uint16_t l_current, uint16_t l_voltage, uint16_t l_power);
    double secondOrder(uint16_t rpm, float theta, uint16_t l_current, uint16_t l_voltage, uint16_t l_power);
    double bestFit(uint16_t rpm, float theta, uint16_t l_current, uint16_t l_voltage, uint16_t l_power);
  private:
    double _rpm;
    double _theta;
    double _l_voltage;
    double _l_current;
    double _l_power;
    void updateVars(uint16_t rpm, float theta, uint16_t l_current, uint16_t l_voltage, uint16_t l_power);
};

#endif
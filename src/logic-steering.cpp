/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "logic-steering.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>
//#include <stdlib.h>

float Steering::decode(const std::string &data) noexcept {
    std::cout << "[UDP] Got data:" << data << std::endl;
    float temp = std::stof(data);
    return temp;
}

Steering::Steering(bool verbose, uint32_t id, float pconst, float iconst, float tolerance, cluon::OD4Session &od4, cluon::OD4Session &od4Gpio, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Pwm)
    : m_od4(od4)
    , m_od4Gpio(od4Gpio)
    , m_od4Analog(od4Analog)
    , m_od4Pwm(od4Pwm)
    , m_debug(verbose)
    , m_bbbId(id)
    , m_senderStampOffsetGpio(id*1000)
    , m_senderStampOffsetAnalog(id*1000+200)
    , m_senderStampOffsetPwm(id*1000+300)
    , m_initialised()
    , m_groundSteeringRequest()
    , m_steeringCurrentDuty()
    //, m_steerLeft(1)
    , m_steerRight(1)
    //, m_steerSelect(0)
    , m_steerCurrent()
    , m_steerPosition()
    , m_steerPositionRack()
    , m_steerVoltage()
    , m_iControlOld()
    , m_pConst(pconst)
    , m_iConstTI(iconst)
    , m_tolerance(tolerance)
    , m_clamped()
    , m_steeringCurrentDutyOld()
    , m_clampedOld()
    , m_steerRightOld()
    , m_rackFound()
    , m_findRackSeqNo()
    , m_findRackTuning()
    , m_asms()
    , m_clampExtended()
    , m_currentState()
    , m_pressureServiceTank()

{
	Steering::setUp();
}

Steering::~Steering() 
{
  Steering::tearDown();
}

void Steering::body()
{
    m_clamped = false;
    m_steeringCurrentDuty = 0;

    if (m_rackFound && (m_currentState == asState::AS_DRIVING)){
        controlPosition(m_groundSteeringRequest, m_steerPositionRack);
    } else if (m_asms && (m_pressureServiceTank >= 6)){
        findRack();
    }

    if (!m_asms){
        m_findRackSeqNo = 0;
        m_rackFound = false;
        m_clamped = false;
        m_findRackTuning = 0;
        controlPosition(m_steerPosition, m_steerPosition);
    }


    cluon::data::TimeStamp sampleTime = cluon::time::now();
    int16_t senderStamp = 0;

    opendlv::proxy::SwitchStateRequest msgGpio;

    if(m_steerRight != m_steerRightOld){
        senderStamp = m_gpioPinSteerRight + m_senderStampOffsetGpio;
        msgGpio.state(m_steerRight);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_steerRightOld = m_steerRight;
    }
    if(m_clamped != m_clampedOld){
        senderStamp = m_gpioPinClamp + m_senderStampOffsetGpio;
        msgGpio.state(m_clamped);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_clampedOld = m_clamped;
    }

    if(m_steeringCurrentDuty != m_steeringCurrentDutyOld){
        opendlv::proxy::PulseWidthModulationRequest msgPwm;
        senderStamp = m_pwmPinSteer + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_steeringCurrentDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_steeringCurrentDutyOld = m_steeringCurrentDuty;
    }
}

bool Steering::controlPosition(float setPoint, float refPoint)
{
    if (setPoint <= -21){
        setPoint = -21;
    }else if (setPoint >= 21){
        setPoint = 21;
    }

    bool ret = false;
    float steerError = setPoint-refPoint;
    float pControl = m_pConst * steerError;
    float iControl = 0;

    if (abs((int) pControl) < 10000){
          iControl = (float) (pControl*m_iConstTS/m_iConstTI+m_iControlOld);
    } 

    if (iControl > 25000){
        iControl = 25000;
    } else if(iControl < -25000){
        iControl = -25000;
    }

    m_iControlOld = iControl;

    float controlSignal = pControl + iControl;

    m_steeringCurrentDuty = (uint32_t) abs((int)round(controlSignal));

    if (m_steeringCurrentDuty > 50000){
        m_steeringCurrentDuty = 50000;
    }

    m_steerRight = controlSignal < 0;
     
    if(steerError > -m_tolerance && steerError < m_tolerance) {
      m_steeringCurrentDuty = 0;
      iControl = 0;
      ret = true;
    }
    if (m_debug){
        std::cout << "[LOGIC-STEERING-FINDRACK] Error: " << steerError 
                    << "\t Duty: " << m_steeringCurrentDuty 
                    << "\t pControl: " << pControl 
                    << "\t iControl: " << iControl 
                    << "\t Direction: " << m_steerRight
                    << "\t Request: " << setPoint
                    << "\t Refpoint" << refPoint
                    << "\t Measure: " << m_steerPosition 
                    << std::endl;
    }
    return ret;

}

void Steering::findRack()
{
    switch(m_findRackSeqNo){
        case 0: // 
            m_rackFound = false;
            if (controlPosition((m_steerPositionRack+(float) 0.75), m_steerPosition))
                m_findRackSeqNo = 10;
            break;

        case 10:
            m_clamped = true;
            if (controlPosition((m_steerPositionRack+(float) 0.75 - m_findRackTuning), m_steerPosition))
                m_findRackTuning += (float) 0.1;
            
            if (m_clampExtended)
                m_findRackSeqNo = 20;
            break;
        case 20:
	        m_clamped = true;
            m_findRackTuning = 0;
            m_rackFound = true;
            break;
        default:
        break;
    }
}

void Steering::setUp()
{
  m_initialised = true;
}

void Steering::tearDown()
{
}

uint16_t Steering::getGpioPinClampSensor(){
  return m_gpioPinClampSensor;
}

uint16_t Steering::getGpioPinAsms(){
  return m_gpioPinAsms;
}

uint16_t Steering::getAnalogPinSteerPosition(){
  return m_analogPinSteerPosition;
}

uint16_t Steering::getAnalogPinSteerPositionRack(){
  return m_analogPinSteerPositionRack;
}

uint32_t Steering::getSenderStampOffsetGpio(){
  return m_senderStampOffsetGpio;
}

uint32_t Steering::getSenderStampOffsetAnalog(){
  return m_senderStampOffsetAnalog;
}
uint16_t Steering::getAnalogPinServiceTank(){
  return m_analogPinServiceTank;
}

void Steering::setSteerPositionRack(float pos){
    m_steerPositionRack = pos;
}
void Steering::setSteerPosition(float pos){
    m_steerPosition = pos;
}
void Steering::setGroundSteeringRequest(float pos){
    m_groundSteeringRequest = pos;
}

void Steering::setClampExtended(bool state){
    m_clampExtended = state;
}
void Steering::setAsms(bool state){
    m_asms = state;
}
void Steering::setCurrentState(uint16_t state){
    m_currentState = (asState) state;
}
void Steering::setPressureServiceTank(float pos){
    m_pressureServiceTank = pos;
}

bool Steering::getInitialised(){
  return m_initialised;
}
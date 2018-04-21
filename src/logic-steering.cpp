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

Steering::Steering(bool verbose, uint32_t id, float pconst, float iconst, float tolerance)
    : m_debug(verbose)
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
    , m_findRackSeqNo()
    , m_findRackTuning()
    , m_asms()
    , m_clampExtended()

{
	Steering::setUp();
}

Steering::~Steering() 
{
  Steering::tearDown();
}

void Steering::callOnReceive(cluon::data::Envelope data){
    if (!m_initialised) {
        return;
    }
    if (data.dataType() == static_cast<int32_t>(opendlv::proxy::GroundSteeringRequest::ID())) {
        opendlv::proxy::GroundSteeringRequest steeringReq = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(data));
        if (steeringReq.groundSteering() >= -21 && steeringReq.groundSteering() <= 21)
            m_groundSteeringRequest = steeringReq.groundSteering();

        std::cout << "[LOGIC-STEERING] Steering Request:" << m_groundSteeringRequest << std::endl;
    }else if (data.dataType() == static_cast<int32_t>(opendlv::proxy::GroundSteeringReading::ID())) {
        opendlv::proxy::GroundSteeringReading analogInput = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(data));
        if (data.senderStamp() - m_senderStampOffsetAnalog == m_analogPinSteerPosition){
          m_steerPosition = analogInput.groundSteering();
	        if (m_debug)
          	    std::cout << "[LOGIC-STEERING-POSITION-ACT] Position reading:" << m_steerPosition << std::endl;

        }else if (data.senderStamp() - m_senderStampOffsetAnalog == m_analogPinSteerPositionRack){
          m_steerPositionRack = analogInput.groundSteering();
	        if (m_debug)
         	    std::cout << "[LOGIC-STEERING-POSITION-RACK] Position reading:" << m_steerPositionRack << std::endl;
        }
    }else if (data.dataType() == static_cast<int32_t>(opendlv::proxy::SwitchStateReading::ID())) {
        uint16_t pin = data.senderStamp()-m_senderStampOffsetGpio;
        if (pin == m_gpioPinClampSensor){
            opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
            m_clampExtended = gpioState.state();
        }else if (pin == m_gpioPinAsms){
            opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
            m_asms = gpioState.state();
        }
        //std::cout << "[LOGIC-STEERING-VOLTAGE] The read pin: " << pin << " state:" << value << std::endl;
     }

    

}

void Steering::body(cluon::OD4Session &od4)
{
   // if (m_steerPositionRack > -22 && m_steerPositionRack < 22)
   //     controlPosition(od4, m_steerPositionRack);

    if (m_clamped){
        controlPosition(od4, m_groundSteeringRequest);
    } else if (m_asms){
        findRack(od4);
    }

    if (!m_asms){
        m_findRackSeqNo = 0;
        m_clamped = false;
	m_findRackTuning = 0;
	controlPosition(od4, m_steerPosition);

    	std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    	cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    	int16_t senderStamp = 0;

    	opendlv::proxy::SwitchStateRequest msgGpio;

    	senderStamp = m_gpioPinClamp + m_senderStampOffsetGpio;
    	msgGpio.state(false);
	od4.send(msgGpio, sampleTime, senderStamp);
    
     }
}

bool Steering::controlPosition(cluon::OD4Session &od4, float setPoint)
{
    if (setPoint <= -21){
        setPoint = -21;
    }else if (setPoint >= 21){
        setPoint = 21;
    }

    bool ret = false;
    float steerError = setPoint-m_steerPosition;
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
                    << "\t Measure: " << m_steerPosition 
                    << std::endl;
    }

   
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    int16_t senderStamp = 0;
    
    opendlv::proxy::SwitchStateRequest msgGpio;

    senderStamp = m_gpioPinSteerRight + m_senderStampOffsetGpio;
    msgGpio.state(m_steerRight);
    od4.send(msgGpio, sampleTime, senderStamp);


    opendlv::proxy::PulseWidthModulationRequest msgPwm;
 
    senderStamp = m_pwmPinSteer + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_steeringCurrentDuty);
    od4.send(msgPwm, sampleTime, senderStamp);
    return ret;

}

void Steering::findRack(cluon::OD4Session &od4)
{
    bool clamp = false;
    switch(m_findRackSeqNo){
        case 0: // 
            if (controlPosition(od4, m_steerPositionRack+(float) 0.75))
                m_findRackSeqNo = 10;
            break;

        case 10:
            clamp = true;
            if (controlPosition(od4, m_steerPositionRack+(float) 0.75 - m_findRackTuning))
                m_findRackTuning += (float) 0.1;
            
            if (m_clampExtended)
                m_findRackSeqNo = 20;
            break;
        case 20:
	    clamp = true;
            m_findRackTuning = 0;
            m_clamped = true;
            break;


        default:
        break;
    }

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
    int16_t senderStamp = 0;

    opendlv::proxy::SwitchStateRequest msgGpio;

    senderStamp = m_gpioPinClamp + m_senderStampOffsetGpio;
    msgGpio.state(clamp);
    od4.send(msgGpio, sampleTime, senderStamp);


}

void Steering::setUp()
{
  m_initialised = true;
}

void Steering::tearDown()
{
}

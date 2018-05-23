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

#ifndef STEERING
#define STEERING

#include "opendlv-standard-message-set.hpp"

#include <memory>
#include <string>
#include <vector>
#include <utility>

enum asState {
    AS_OFF,
    AS_READY, 
    AS_DRIVING, 
    AS_FINISHED, 
    EBS_TRIGGERED
 };

class Steering {
   private:
    //Steering(const Steering &) = delete;
    //Steering(Steering &&)      = delete;
    //Steering &operator=(const Steering &) = delete;
    //Steering &operator=(Steering &&) = delete;

   public:
    Steering(bool verbose, uint32_t id, float pconst, float iconst, float tolerance, cluon::OD4Session &od4, cluon::OD4Session &od4Gpio, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Pwm);
    ~Steering();
    uint16_t getGpioPinClampSensor();
    uint16_t getGpioPinAsms();
    uint16_t getAnalogPinSteerPosition();
    uint16_t getAnalogPinSteerPositionRack();
    uint32_t getSenderStampOffsetGpio();
    uint32_t getSenderStampOffsetAnalog();
    uint16_t getAnalogPinServiceTank();
    void setSteerPositionRack(float pos);
    void setSteerPosition(float pos);
    void setGroundSteeringRequest(float pos);
    void setClampExtended(bool state);
    void setAsms(bool state);
    void setCurrentState(uint16_t);
    void setPressureServiceTank(float pos);
    bool getInitialised();

    const uint32_t m_senderStampCurrentState = 1401;

   public:
    float decode(const std::string &data) noexcept;
    void body();

   private:
    bool controlPosition(float setPoint, float refPoint);
    void findRack();
    void setUp();
    void tearDown();
    

    cluon::OD4Session &m_od4;
    cluon::OD4Session &m_od4Gpio;
    cluon::OD4Session &m_od4Analog;
    cluon::OD4Session &m_od4Pwm;
    bool m_debug;
    uint32_t m_bbbId;
    uint32_t m_senderStampOffsetGpio;
    uint32_t m_senderStampOffsetAnalog;
    uint32_t m_senderStampOffsetPwm;
    bool m_initialised;
    float m_groundSteeringRequest;
    uint32_t m_steeringCurrentDuty;
    //bool m_steerLeft;
    bool m_steerRight;
    //bool m_steerSelect;
    float m_steerCurrent;
    float m_steerPosition;
    float m_steerPositionRack;
    float m_steerVoltage;
    float m_iControlOld;
    float m_pConst;
    float m_iConstTI;
    float m_tolerance;
    bool m_clamped;
    int m_findRackSeqNo;
    float m_findRackTuning;
    bool m_asms;
    bool m_clampExtended;
    asState m_currentState;
    float m_pressureServiceTank;


    //const uint16_t m_gpioPinSteerLeft = 47;
    const uint16_t m_gpioPinSteerRight = 46;
    //const uint16_t m_gpioPinSteerSelect = 26;
    const uint16_t m_gpioPinAsms = 115;
    const uint16_t m_gpioPinClampSensor = 112;
    const uint16_t m_gpioPinClamp = 65;
    const uint16_t m_pwmPinSteer = 40;

    const uint16_t m_analogPinSteerCurrent = 4;
    const uint16_t m_analogPinSteerPosition = 0;
    const uint16_t m_analogPinSteerPositionRack = 6;
    const uint16_t m_analogPinServiceTank = 2;

    const double m_analogConvSteerCurrent = 1;
    const double m_analogConvSteerPosition = 80.38;
    const double m_analogConvSteerPositionRack = 80.86;
    const double m_analogOffsetSteerPosition = 27.74;
    const double m_analogOffsetSteerPositionRack = 28.06;
    const double m_iConstTS = 0.03;
    
};

#endif


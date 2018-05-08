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
#include "opendlv-standard-message-set.hpp"

#include "logic-steering.hpp"

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <ctime>
#include <chrono>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("port")) || (0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("pconst")) || (0 == commandlineArguments.count("iconst")) || (0 == commandlineArguments.count("tolerance"))) {
        std::cerr << argv[0] << " testing unit and publishes it to a running OpenDaVINCI session using the OpenDLV Standard Message Set." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --port=<udp port>--cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple beaglebone units>] [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --port=8884 --cid=111 --cidgpio=220 --cidanalog=221 --cidpwm=222 --id=1 --verbose=1 --freq=30 --pconst=10000 --iconst=0.5 --tolerance=0.1" << std::endl;
        retCode = 1;
    } else {
        const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const float FREQ{std::stof(commandlineArguments["freq"])};
        std::cout << "Micro-Service ID:" << ID << std::endl;

        // Interface to a running OpenDaVINCI session.
        
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        cluon::OD4Session od4Gpio{static_cast<uint16_t>(std::stoi(commandlineArguments["cidgpio"]))};
        cluon::OD4Session od4Analog{static_cast<uint16_t>(std::stoi(commandlineArguments["cidanalog"]))};
        cluon::OD4Session od4Pwm{static_cast<uint16_t>(std::stoi(commandlineArguments["cidpwm"]))};
        
        Steering steering(VERBOSE, ID, std::stof(commandlineArguments["pconst"]), std::stof(commandlineArguments["iconst"]), std::stof(commandlineArguments["tolerance"]), od4, od4Gpio, od4Analog, od4Pwm);


        // Interface to OxTS.
        const std::string ADDR("0.0.0.0");
        const std::string PORT(commandlineArguments["port"]);
        
        cluon::UDPReceiver UdpSocket(ADDR, std::stoi(PORT),
            [&od4Session = od4, &steer=steering, VERBOSE, senderStamp=ID](std::string &&d, std::string &&/*from*/, std::chrono::system_clock::time_point &&tp) noexcept {
            
            cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
            std::time_t epoch_time = std::chrono::system_clock::to_time_t(tp);
            std::cout << "[UDP] Time: " << std::ctime(&epoch_time) << std::endl;
            float groundSteer = steer.decode(d);

            opendlv::proxy::GroundSteeringRequest msg;
            msg.groundSteering(groundSteer);
            od4Session.send(msg, sampleTime, senderStamp);
	    std::cout << "[UDP] Message sent: " << groundSteer << std::endl;
        });


       auto onGroundSteeringRequest{[&steering](cluon::data::Envelope &&envelope)
        {   
            if (!steering.getInitialised()){
                return;
            }
            opendlv::proxy::GroundSteeringRequest steeringReq = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
            if (steeringReq.groundSteering() >= -21 && steeringReq.groundSteering() <= 21)
                steering.setGroundSteeringRequest(steeringReq.groundSteering());
        }};
        od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
        
        auto onGroundSteeringReading{[&steering, &VERBOSE](cluon::data::Envelope &&envelope)
            {
                if (!steering.getInitialised()){
                    return;
                }
                uint16_t channel = envelope.senderStamp()-steering.getSenderStampOffsetAnalog();
                opendlv::proxy::GroundSteeringReading analogInput = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(envelope));
                if (channel == steering.getAnalogPinSteerPosition()){
                steering.setSteerPosition(analogInput.groundSteering());
                   if (VERBOSE)
                        std::cout << "[LOGIC-STEERING-POSITION-ACT] Position reading:" << analogInput.groundSteering() << std::endl;

                }else if (channel == steering.getAnalogPinSteerPositionRack()){
                steering.setSteerPositionRack(analogInput.groundSteering());
                    if (VERBOSE)
                        std::cout << "[LOGIC-STEERING-POSITION-RACK] Position reading:" << analogInput.groundSteering() << std::endl;
                }
            }};
            od4Analog.dataTrigger(opendlv::proxy::GroundSteeringReading::ID(), onGroundSteeringReading);

        auto onSwitchStateReading{[&steering](cluon::data::Envelope &&envelope)
            {
                if (!steering.getInitialised()){
                    return;
                }
                uint16_t pin = envelope.senderStamp()-steering.getSenderStampOffsetGpio();
                if (pin == steering.getGpioPinClampSensor()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    steering.setClampEntended(gpioState.state());
                }else if (pin == steering.getGpioPinAsms()){
                    opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                    steering.setAsms(gpioState.state());
                }
            }};
            od4Gpio.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);



        // Just sleep as this microservice is data driven.
        using namespace std::literals::chrono_literals;

        auto atFrequency{[&steering]() -> bool
        {            
            steering.body();
            return true;
        }};

        od4.timeTrigger(FREQ, atFrequency);
    }
    return retCode;
}


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
        std::cerr << "Example: " << argv[0] << " --port=8884 --cid=111 --id=1 --verbose=1 --freq=30 --pconst=10000 --iconst=0.5 --tolerance=0.1" << std::endl;
        retCode = 1;
    } else {
        const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const double FREQ{static_cast<double>(std::stof(commandlineArguments["freq"]))};
        std::cout << "Micro-Service ID:" << ID << std::endl;

        // Interface to a running OpenDaVINCI session.
        Steering steering(VERBOSE, ID, std::stof(commandlineArguments["pconst"]), std::stof(commandlineArguments["iconst"]), std::stof(commandlineArguments["tolerance"]));

        cluon::data::Envelope data;
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])),
            [&data, &steer = steering](cluon::data::Envelope &&envelope){
                steer.callOnReceive(envelope);
                // IMPORTANT INTRODUCE A MUTEX
            }
        };

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

        // Just sleep as this microservice is data driven.
        using namespace std::literals::chrono_literals;

        std::chrono::system_clock::time_point threadTime = std::chrono::system_clock::now();
        while (od4.isRunning()) {
            
            std::this_thread::sleep_until(std::chrono::duration<double>(1/FREQ)+threadTime);
            threadTime = std::chrono::system_clock::now();

            steering.body(od4);
        }
    }
    return retCode;
}


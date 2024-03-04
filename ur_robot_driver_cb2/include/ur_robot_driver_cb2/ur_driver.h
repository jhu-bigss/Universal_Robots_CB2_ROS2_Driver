/*
 * ur_driver
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UR_DRIVER_H_
#define UR_DRIVER_H_

#include "rclcpp/rclcpp.hpp"

#include <mutex>
#include <condition_variable>
#include "ur_realtime_communication.h"
#include "ur_communication.h"
#include <vector>
#include <math.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <chrono>

namespace cb2_hw
{
  
class UrDriver {
private:
	double minimum_payload_;
	double maximum_payload_;
	std::vector<std::string> joint_names_;
	std::string ip_addr_;
	const int MULT_JOINTSTATE_ = 1000000;
	const int MULT_TIME_ = 1000000;
  unsigned int m_reverse_port;
	int incoming_sockfd_;
	int new_sockfd_;
	bool reverse_connected_;
	double firmware_version_;
public:
	UrRealtimeCommunication* rt_interface_;
	UrCommunication* sec_interface_;

    UrDriver(std::condition_variable& rt_msg_cond, std::condition_variable& msg_cond, std::string host, unsigned int reverse_port = 50007, unsigned int safety_count_max = 12, double min_payload = 0., double max_payload = 1.);
	bool start();
	void halt();

	void setSpeed(double q0, double q1, double q2, double q3, double q4, double q5, double acc = 100.);
  void stop(const double& acc = 100.);
  void setPosition(double q0, double q1, double q2, double q3, double q4, double q5, double dt, double lookahead=0.1, double gain=300);

	std::vector<std::string> getJointNames();
	void setJointNames(std::vector<std::string> jn);
	void setToolVoltage(unsigned int v);
	void setFlag(unsigned int n, bool b);
	void setDigitalOut(unsigned int n, bool b);
	void setAnalogOut(unsigned int n, double f);
	bool setPayload(double m);

	void setMinPayload(double m);
	void setMaxPayload(double m);

};
}
#endif /* UR_DRIVER_H_ */

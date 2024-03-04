/*
 * ur_driver.cpp
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

#include "ur_robot_driver_cb2/ur_driver.h"

namespace cb2_hw
{
  
UrDriver::UrDriver(std::condition_variable& rt_msg_cond,
		std::condition_variable& msg_cond, std::string host,
		unsigned int reverse_port, unsigned int safety_count_max, double min_payload, double max_payload) 
		
{
  m_reverse_port=reverse_port;
  maximum_payload_=max_payload;
  minimum_payload_=min_payload;
  
	struct sockaddr_in serv_addr;
	int flag;

	firmware_version_ = 0;
	reverse_connected_ = false;
	rt_interface_ = new UrRealtimeCommunication(rt_msg_cond, host,
                                              safety_count_max);
	new_sockfd_ = -1;
	sec_interface_ = new UrCommunication(msg_cond, host);

	incoming_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (incoming_sockfd_ < 0) {
		RCLCPP_FATAL(rclcpp::get_logger("UrRobotHW"), "ERROR opening socket for reverse communication");
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(m_reverse_port);
	flag = 1;
	setsockopt(incoming_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag,
			sizeof(int));
	setsockopt(incoming_sockfd_, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));
	if (bind(incoming_sockfd_, (struct sockaddr *) &serv_addr,
			sizeof(serv_addr)) < 0) {
		RCLCPP_WARN_STREAM(rclcpp::get_logger("UrRobotHW"),"Listening on " + ip_addr_ + ":" + std::to_string(m_reverse_port)+ "\n");
		
	}
	listen(incoming_sockfd_, 5);
}




bool UrDriver::start() {
	if (!sec_interface_->start())
  {
	RCLCPP_ERROR(rclcpp::get_logger("UrRobotHW"), "Unable to open secondary connection");
	return false;
  }
	firmware_version_ = sec_interface_->robot_state_->getVersion();
  	RCLCPP_INFO(rclcpp::get_logger("UrRobotHW"), "FIRMWARE VERSION %f",firmware_version_);
	rt_interface_->robot_state_->setVersion(firmware_version_);
	if (!rt_interface_->start())
		return false;
	ip_addr_ = rt_interface_->getLocalIp();
	RCLCPP_DEBUG_STREAM(rclcpp::get_logger("UrRobotHW"), "Listening on " + ip_addr_ + ":" + std::to_string(m_reverse_port) + "\n");
	return true;

}

void UrDriver::halt() {
	sec_interface_->halt();
	rt_interface_->halt();
	close(incoming_sockfd_);
}

void UrDriver::setSpeed(double q0, double q1, double q2, double q3, double q4,
		double q5, double acc) {
	rt_interface_->setSpeed(q0, q1, q2, q3, q4, q5, acc);
	// std::cout << "OUT" << q0 <<" " << q1<<" " << q2<<" " << q3<<" " << q4<<" " << q5<<" " << acc<<"\n" << std::flush;	
}

void UrDriver::stop(const double& acc)
{
  rt_interface_->stop(acc);
}


void UrDriver::setPosition(double q0, double q1, double q2, double q3, double q4, double q5, double dt, double lookahead, double gain)
{
  rt_interface_->setPosition(q0, q1, q2, q3, q4, q5, dt,lookahead,gain);
}
                        

std::vector<std::string> UrDriver::getJointNames() {
	return joint_names_;
}

void UrDriver::setJointNames(std::vector<std::string> jn) {
	joint_names_ = jn;
}

void UrDriver::setToolVoltage(unsigned int v) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_tool_voltage(%d)\nend\n", v);
	rt_interface_->addCommandToQueue(buf);
}
void UrDriver::setFlag(unsigned int n, bool b) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_flag(%d, %s)\nend\n", n,
			b ? "True" : "False");
	rt_interface_->addCommandToQueue(buf);
}
void UrDriver::setDigitalOut(unsigned int n, bool b) {
	char buf[256];
	if (firmware_version_ < 2) {
		sprintf(buf, "sec setOut():\n\tset_digital_out(%d, %s)\nend\n", n,
				b ? "True" : "False");
    } else if (n > 15) {
        sprintf(buf,
                "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n",
                n - 16, b ? "True" : "False");
	} else if (n > 7) {
        sprintf(buf, "sec setOut():\n\tset_configurable_digital_out(%d, %s)\nend\n",
				n - 8, b ? "True" : "False");

	} else {
		sprintf(buf, "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n",
				n, b ? "True" : "False");

	}
	rt_interface_->addCommandToQueue(buf);

}
void UrDriver::setAnalogOut(unsigned int n, double f) {
	char buf[256];
	if (firmware_version_ < 2) {
		sprintf(buf, "sec setOut():\n\tset_analog_out(%d, %1.4f)\nend\n", n, f);
	} else {
		sprintf(buf, "sec setOut():\n\tset_standard_analog_out(%d, %1.4f)\nend\n", n, f);
	}

	rt_interface_->addCommandToQueue(buf);
}

bool UrDriver::setPayload(double m) {
	if ((m < maximum_payload_) && (m > minimum_payload_)) {
		char buf[256];
		sprintf(buf, "sec setOut():\n\tset_payload(%1.3f)\nend\n", m);
		rt_interface_->addCommandToQueue(buf);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("UrRobotHW"),buf);
		return true;
	} else
		return false;
}

void UrDriver::setMinPayload(double m) {
	if (m > 0) {
		minimum_payload_ = m;
	} else {
		minimum_payload_ = 0;
	}

}
void UrDriver::setMaxPayload(double m) {
	maximum_payload_ = m;
}

}
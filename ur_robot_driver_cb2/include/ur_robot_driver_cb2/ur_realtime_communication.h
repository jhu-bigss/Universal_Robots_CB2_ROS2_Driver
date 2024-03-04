
#ifndef UR_REALTIME_COMMUNICATION_H_
#define UR_REALTIME_COMMUNICATION_H_

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_state_RT.h"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <chrono>

class UrRealtimeCommunication {
private:
	unsigned int safety_count_max_;
	std::ofstream inp;
	int sockfd_;
	struct sockaddr_in serv_addr_;
	struct hostent *server_;
	std::string local_ip_;
	bool keepalive_;
	std::thread comThread_;
	int flag_;
	std::recursive_mutex command_string_lock_;
	std::string command_;
	unsigned int safety_count_;
	void run();

  double m_speedj_timeout;
public:
	bool connected_;
	RobotStateRT* robot_state_;

	UrRealtimeCommunication(std::condition_variable& msg_cond, std::string host,
                          unsigned int safety_count_max = 12);
	bool start();
	void halt();
	void setSpeed(double q0, double q1, double q2, double q3, double q4, double q5, double acc = 100.);
	void stop(const double& acc = 100.);
	void setPosition(double q0, double q1, double q2, double q3, double q4, double q5, double dt = 0.008, double lookahead=0.1, double gain=300);
	void addCommandToQueue(std::string inpt);
	void setSafetyCountMax(uint inp);
	std::string getLocalIp();

};

#endif /* UR_REALTIME_COMMUNICATION_H_ */

#include<iostream>
#include<cstdio>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h> 
#include <stdio.h> 
#include "ros/ros.h"
#include <PerFoRoControl/MODE.h>
#include <PerFoRoControl/NavigatePerFoRo.h>

class PerFoRo_Control
{
public:
	PerFoRo_Control();

	~PerFoRo_Control() {} 

	void init();
	void setup_serial();

protected:
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	int fd;
	char *uart_name0 = (char*)"/dev/ttyACM0";
	char *uart_name1 = (char*)"/dev/ttyACM1";
	char *uart_name;
	bool PORTOPEN = false;

	ros::NodeHandle _n;
	ros::Subscriber _mode_perforo_sub;
	ros::Subscriber _navigate_perforo_sub;
	void ModeCallback(const PerFoRoControl::MODE msg);
	void NavigatePerFoRoCallback(const PerFoRoControl::NavigatePerFoRo msg);
	int open_port(const char* port);
	bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
	void close_port(int fd);
};

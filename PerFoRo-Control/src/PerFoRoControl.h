#include<iostream>
#include<cstdio>
#include "ros/ros.h"
#include <PerFoRoControl/MODE.h>
#include <PerFoRoControl/NavigatePerFoRo.h>

class PerFoRo_Control
{
public:
	PerFoRo_Control();

	~PerFoRo_Control() {} 

	void init();

protected:
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle _n;
	ros::Subscriber _mode_perforo_sub;
	ros::Subscriber _navigate_perforo_sub;
	void ModeCallback(const PerFoRoControl::MODE msg);
	void NavigatePerFoRoCallback(const PerFoRoControl::NavigatePerFoRo msg);
};

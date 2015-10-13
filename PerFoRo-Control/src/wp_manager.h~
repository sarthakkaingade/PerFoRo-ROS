#include<vector>
#include<iostream>
#include<cstdio>
#include "ros/ros.h"
#include <wpm/single_waypoint.h>
#include <wpm/waypoints.h>
#include <px4/vehicle_local_position_setpoint.h>

class WaypointManager
{
public:
	WaypointManager();

	~WaypointManager() {} 

	struct Way_points {
		std::vector< std::vector<double> > WP;
		double waypoints[][NO_OF_COLUMNS];
		int no_of_waypoints;
		int current_waypoint;
		//int default_current_waypoint;
	};
	std::vector< std::vector<double> >::iterator i;
	struct Way_points wps;
	void wpm_init();
	void set_waypoints(double [MAX_WAYPOINTS][NO_OF_COLUMNS], int no_of_waypoints, int current_waypoint);
	void add_waypoint(double waypoint[1][NO_OF_COLUMNS], int position);
	void set_current_waypoint(int waypoint_no);
	void clear_all_waypoints();
	void clear_waypoint(int waypoint_no);
	void sync_waypoints();
	void print();

protected:
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle _n;
	ros::Subscriber _sub;
	ros::Publisher _waypoints_pub;
	void pos_spCallback(const px4::vehicle_local_position_setpoint msg);
};

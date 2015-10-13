#include "PerFoRoControl.h"

/**
 * PerFoRo Control
*/

PerFoRo_Control::PerFoRo_Control() :
	_n(),
	_mode_perforo_sub(_n.subscribe("/ModePerFoRo", 1000, &PerFoRo_Control::ModeCallback,this)),
	_navigate_perforo_sub(_n.subscribe("/NavigatePerFoRo", 1000, &PerFoRo_Control::NavigatePerFoRoCallback,this))
{
	
}	

void PerFoRo_Control::ModeCallback(const PerFoRoControl::MODE msg)
{
  ROS_INFO("PerFoRo Control: I heard Mode Command: [%d]", msg.MODE);
}

void PerFoRo_Control::NavigatePerFoRoCallback(const PerFoRoControl::NavigatePerFoRo msg)
{
  ROS_INFO("PerFoRo Control: I heard Navigate Command: [%d]", msg.command);
}

void PerFoRo_Control::init()
{
	ROS_INFO("Initialized PerFoRo Control Module");
}

int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "PerFoRo_Control");

	PerFoRo_Control PFRC;
	PFRC.init();

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();
	
	return 0;
}

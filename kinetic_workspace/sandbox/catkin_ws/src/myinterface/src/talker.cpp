////////////////
// Talker.cpp //
////////////////
/**
 * This is a sample of how to do a publisher using C++
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "network_interface/network_interface.h"

#include <sstream>
#include <string>

// Alias to make it easier to use UDP stuff
namespace as = AS::Network;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

   // Try to generate a udp connection and print the output
	AS::Network::UDPInterface udp;
	std::string ip = "10.12.13.14";
	int port = 11111;
	ROS_INFO("Attempting connection to %s on port %d", ip.c_str(), port);

	as::return_statuses status;
	while(ros::ok() &&
			((status = udp.open(ip.c_str(),port)) != as::return_statuses::OK))
	{
		std::string status_msg = as::return_status_desc(status);
		ROS_INFO("Failed to connect to UDP; %s", status_msg.c_str());
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Connected to UDP.");

	// TODO: generate a UDP message with a header, size, and data[] through ROS 

  // For now, don't actually run the node, just do nothing

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  /*
  int count = 0;
  while (false && ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
  /*
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
  /*
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  */


  return 0;
}

/* vim: ts=3 sts=3 sw=3 noet nowrap :*/

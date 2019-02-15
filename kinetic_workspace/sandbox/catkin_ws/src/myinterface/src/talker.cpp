////////////////
// Talker.cpp //
////////////////
/**
 * This is a sample of how to do a publisher using C++
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

using boost::asio::ip::udp;
using boost::asio::io_service;

/*
 * The boost asio library does not provide a blocking read with timeout function so we have to roll our own.
 */
int receive_from(
    boost::asio::ip::udp::socket&         socket,
    const boost::asio::mutable_buffers_1& buf,
    boost::asio::ip::udp::endpoint&       remoteEndpoint,
    boost::system::error_code&            error,
    std::chrono::milliseconds                  timeout)
{
    volatile bool ioDone = false;
    int numBytesReceived = 0;
    boost::asio::io_service& ioService = socket.get_io_service();    

    socket.async_receive_from(buf, remoteEndpoint,
                              [&error, &ioDone, &numBytesReceived](const boost::system::error_code& errorAsync, size_t bytesReceived)
                              {
                                  ioDone = true;
                                  error = errorAsync;
                                  numBytesReceived = bytesReceived;
                              });

	 std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ioService.reset();
    ioService.poll_one();

    auto endTime = std::chrono::system_clock::now() + timeout;

    while (!ioDone)
    {
        ioService.reset();
		  std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto now = std::chrono::system_clock::now();
        if (now > endTime)
        {
            socket.cancel();
            error = boost::asio::error::timed_out;
            return 0;
        }
        ioService.poll_one();
    }
    ioService.reset();

    return numBytesReceived;
}


int WaitForPacket(
		uint16_t portNum, 
		std::vector<char>& udpBuf, 
		udp::endpoint& remoteEndpoint, 
		const std::chrono::milliseconds timeout)
{
		boost::asio::io_service ioService;

		boost::asio::ip::udp::socket socket(ioService, 
							  boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), portNum));
    socket.set_option(boost::asio::socket_base::broadcast(true));

    boost::system::error_code error;
    int numBytes = receive_from(socket, 
										  boost::asio::buffer(udpBuf), 
										  remoteEndpoint, 
										  error, 
										  timeout);

    if (error && error != boost::asio::error::message_size && 
			 error != boost::asio::error::timed_out)
    {
        printf("Got error: %s\n", error.message().c_str());
        return -1;
    }

    return numBytes;
}
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

  std::vector<char> packets(1112);
udp::endpoint endpoint(boost::asio::udp::v4(), "192.168.1.2", 31122);

  // Generate a UDP server with boost::asio, asynchronously.
  while(ros::ok())
  {
	int numbytes = WaitForPacket(31122, packets, endpoint, std::chrono::milliseconds(1000));
	ROS_INFO("Read %d packets", numbytes);
  }

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

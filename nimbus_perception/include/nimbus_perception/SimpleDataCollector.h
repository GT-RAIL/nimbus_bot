#ifndef SIMPLE_DATA_COLLECTOR_H_
#define SIMPLE_DATA_COLLECTOR_H_

//ROS
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>

//C++
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_D 0x64

class SimpleDataCollector
{

public:

  /**
   * \brief Constructor
   */
  SimpleDataCollector();

  /*!
   * \brief Monitors the keyboard
   */
  void loop();

private:
  void newDataCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData);

  void showObject(unsigned int index);

  ros::NodeHandle n, pnh;

  ros::Subscriber newDataSubscriber;
  ros::Publisher currentObjectPublisher;

  rail_manipulation_msgs::SegmentedObjectList objects;
  unsigned int index;

  boost::mutex objectMutex;
};

/*!
 * \brief A function to close ROS and exit the program.
 *
 * \param sig The signal value.
 */
void shutdown(int sig);

#endif

/*!
 * \NimbusActionQueue.h
 * \brief Sequential execution of general actions on Nimbus, the RAIL lab's tabletop robot arm.
 *
 * NimbusActionQueue creates and maintains a list of general actions, which can be executed one at a time or
 * in its entirety.  The list includes a set of services for adding and removing actions, and the action themselves
 * include general failure cases, reported if execution has to stop.
 *
 * \author David Kent, Georgia Institute of Technology - dekent@gatech.edu
 * \date February 25, 2016
 */

#ifndef NIMBUS_ACTION_QUEUE
#define NIMBUS_ACTION_QUEUE

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <rail_action_queue_msgs/AddAction.h>
#include <rail_action_queue_msgs/ExecuteAction.h>
#include <rail_action_queue_msgs/GeneralAction.h>
#include <rail_action_queue_msgs/GetActionList.h>
#include <rail_action_queue_msgs/InsertAction.h>
#include <rail_action_queue_msgs/RemoveAction.h>
#include <rail_manipulation_msgs/ArmAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/StoreAction.h>
#include <std_srvs/Empty.h>

class NimbusActionQueue
{
public:
  /*!
   * Creates a nimbus_action_queue object that can be used to define and execute an action list.
   */
  NimbusActionQueue();

private:
  ros::NodeHandle node, pn;

  ros::Subscriber recognizedObjectsSubscriber;

  ros::ServiceClient segmentClient;

  ros::ServiceServer add_action_server;
  ros::ServiceServer clear_action_list_server;
  ros::ServiceServer get_action_list_server;
  ros::ServiceServer insert_action_server;
  ros::ServiceServer remove_action_server;

  actionlib::SimpleActionClient<rail_manipulation_msgs::ArmAction> armActionClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::StoreAction> storeClient;

  actionlib::SimpleActionServer<rail_action_queue_msgs::ExecuteAction> executeServer;

  int recognizedObjectsCounter;

  std::vector<rail_action_queue_msgs::GeneralAction> action_list;
  rail_manipulation_msgs::SegmentedObjectList recognizedObjects;

  boost::mutex recognizedObjectsMutex;

  void executeActions(const rail_action_queue_msgs::ExecuteGoalConstPtr &goal);

  unsigned char executeSingleAction(const rail_action_queue_msgs::GeneralAction &action);

  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);

  /**
  * \brief Callback for adding an action to the list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool addAction(rail_action_queue_msgs::AddAction::Request &req, rail_action_queue_msgs::AddAction::Response &res);

  /**
  * \brief Callback for clearing the action list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool clearActionList(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
  * \brief Callback for getting the list of actions
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool getActionList(rail_action_queue_msgs::GetActionList::Request &req, rail_action_queue_msgs::GetActionList::Response &res);

  /**
  * \brief Callback for inserting an action to the list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool insertAction(rail_action_queue_msgs::InsertAction::Request &req, rail_action_queue_msgs::InsertAction::Response &res);

  /**
  * \brief Callback for removing an action from the list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool removeAction(rail_action_queue_msgs::RemoveAction::Request &req, rail_action_queue_msgs::RemoveAction::Response &res);
};

int main(int argc, char **argv);

#endif

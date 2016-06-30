/*!
 * \NimbusActionQueue.cpp
 * \brief Sequential execution of general actions on Nimbus, the RAIL lab's tabletop robot arm.
 *
 * NimbusActionQueue creates and maintains a list of general actions, which can be executed one at a time or
 * in its entirety.  The list includes a set of services for adding and removing actions, and the action themselves
 * include general failure cases, reported if execution has to stop.
 *
 * \author David Kent, Georgia Institute of Technology - dekent@gatech.edu
 * \date February 25, 2016
 */

#include <nimbus_action_queue/NimbusActionQueue.h>

using namespace std;

NimbusActionQueue::NimbusActionQueue() :
  armActionClient("nimbus_moveit/common_actions/arm_action"),
  pickupClient("nimbus_moveit/common_actions/pickup"),
  storeClient("nimbus_moveit/common_actions/store"),
  executeServer(node, "nimbus_action_queue/execute", boost::bind(&NimbusActionQueue::executeActions, this, _1), false),
  pn("~")
{
  recognizedObjectsCounter = 0;

  recognizedObjectsSubscriber = node.subscribe("/object_recognition_listener/recognized_objects", 1, &NimbusActionQueue::recognizedObjectsCallback, this);

  segmentClient = node.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");

  add_action_server = pn.advertiseService("add_action", &NimbusActionQueue::addAction, this);
  clear_action_list_server = pn.advertiseService("clear_action_list", &NimbusActionQueue::clearActionList, this);
  get_action_list_server = pn.advertiseService("get_action_list", &NimbusActionQueue::getActionList, this);
  insert_action_server = pn.advertiseService("insert_action", &NimbusActionQueue::insertAction, this);
  remove_action_server = pn.advertiseService("remove_action", &NimbusActionQueue::removeAction, this);

  executeServer.start();
}

void NimbusActionQueue::executeActions(const rail_action_queue_msgs::ExecuteGoalConstPtr &goal)
{
  rail_action_queue_msgs::ExecuteResult result;
  rail_action_queue_msgs::ExecuteFeedback feedback;
  result.success = false;
  result.error = rail_action_queue_msgs::GeneralAction::SUCCESS;

  if (goal->execute_all)
  {
    while (!action_list.empty())
    {
      //execute actions until one fails or no actions remain
      feedback.current_action = action_list.at(0);
      feedback.actions_remaining = action_list.size() - 1;
      feedback.last_completed_action.action_type = rail_action_queue_msgs::GeneralAction::NO_ACTION;
      executeServer.publishFeedback(feedback);

      result.error = executeSingleAction(action_list.at(0));
      //remove the action if it was successfully executed
      if (result.error == rail_action_queue_msgs::GeneralAction::SUCCESS)
      {
        feedback.last_completed_action = action_list.at(0);
        result.last_action = action_list.at(0);
        action_list.erase(action_list.begin());
      }
      //set preempted if the action server requests preempt
      else if (result.error == rail_action_queue_msgs::GeneralAction::PREEMPTED)
      {
        result.last_action = action_list.at(0);
        executeServer.setPreempted(result);
        return;
      }
      //otherwise stop execution and report failure
      else
      {
        result.last_action = action_list.at(0);
        executeServer.setSucceeded(result);
        return;
      }
    }
  }
  else
  {
    //execute only the current action
    if (!action_list.empty())
    {
      feedback.last_completed_action.action_type = rail_action_queue_msgs::GeneralAction::NO_ACTION;
      feedback.current_action = action_list.at(0);
      feedback.actions_remaining = action_list.size() - 1;
      executeServer.publishFeedback(feedback);

      result.error = executeSingleAction(action_list.at(0));
      //remove the action if it was successfully executed
      if (result.error == rail_action_queue_msgs::GeneralAction::SUCCESS)
      {
        result.last_action = action_list.at(0);
        action_list.erase(action_list.begin());
      }
      else if (result.error == rail_action_queue_msgs::GeneralAction::PREEMPTED)
      {
        result.last_action = action_list.at(0);
        executeServer.setPreempted(result);
      }
      else
      {
        result.last_action = action_list.at(0);
        executeServer.setSucceeded(result);
      }
    }
  }

  if (result.error == rail_action_queue_msgs::GeneralAction::SUCCESS)
    result.success = true;

  executeServer.setSucceeded(result);
}

unsigned char NimbusActionQueue::executeSingleAction(const rail_action_queue_msgs::GeneralAction &action)
{
  switch (action.action_type)
  {
    case rail_action_queue_msgs::GeneralAction::READY_ARM:
    {
      rail_manipulation_msgs::ArmGoal readyArmGoal;
      readyArmGoal.action = rail_manipulation_msgs::ArmGoal::READY;
      armActionClient.sendGoal(readyArmGoal);
      while (!armActionClient.getState().isDone()) {
        if (executeServer.isPreemptRequested()) {
          ROS_INFO("Canceling arm ready goal...");
          armActionClient.cancelAllGoals();
          return rail_action_queue_msgs::GeneralAction::PREEMPTED;
        }
      }

      if (armActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
          && armActionClient.getResult()->success) {
        return rail_action_queue_msgs::GeneralAction::SUCCESS;
      }
      else {
        return rail_action_queue_msgs::GeneralAction::MANIPULATION_PLANNING_FAILURE;
      }
    }


    case rail_action_queue_msgs::GeneralAction::RETRACT_ARM:
    {
      rail_manipulation_msgs::ArmGoal retractArmGoal;
      retractArmGoal.action = rail_manipulation_msgs::ArmGoal::RETRACT;
      armActionClient.sendGoal(retractArmGoal);
      while (!armActionClient.getState().isDone()) {
        if (executeServer.isPreemptRequested()) {
          ROS_INFO("Canceling arm retract goal...");
          armActionClient.cancelAllGoals();
          return rail_action_queue_msgs::GeneralAction::PREEMPTED;
        }
      }
      if (armActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
          && armActionClient.getResult()->success) {
        return rail_action_queue_msgs::GeneralAction::SUCCESS;
      }
      else {
        return rail_action_queue_msgs::GeneralAction::MANIPULATION_PLANNING_FAILURE;
      }
    }


    case rail_action_queue_msgs::GeneralAction::PICKUP:
    {
      //if no object name was specified, grasp at the given manipulation pose instead
      if (action.object_name.empty())
      {
        rail_manipulation_msgs::PickupGoal pickupGoal;
        pickupGoal.lift = true;
        pickupGoal.verify = true;  //note: for now pickups are unverified always, because CARL can't do this.  If
        // possible, change this to always verify pickups.
        pickupGoal.pose = action.manipulation_pose;
        pickupClient.sendGoal(pickupGoal);

        while (!pickupClient.getState().isDone())
        {
          if (executeServer.isPreemptRequested())
          {
            ROS_INFO("Canceling pickup goal...");
            pickupClient.cancelAllGoals();
            return rail_action_queue_msgs::GeneralAction::PREEMPTED;
          }
        }

        if (pickupClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Pickup failed.");
          return rail_action_queue_msgs::GeneralAction::MANIPULATION_EXECUTION_FAILURE;
        }
        else if (!pickupClient.getResult()->executionSuccess)
        {
          ROS_INFO("Pickup failed due to planner.");
          return rail_action_queue_msgs::GeneralAction::MANIPULATION_PLANNING_FAILURE;
        }
        else if (!pickupClient.getResult()->success)
        {
          ROS_INFO("Pickup failed during execution.");
          return rail_action_queue_msgs::GeneralAction::MANIPULATION_EXECUTION_FAILURE;
        }
        else
        {
          ROS_INFO("Pickup succeeded!");
          return rail_action_queue_msgs::GeneralAction::SUCCESS;
        }
      }
      //otherwise grab an object with the given name
      else
      {
        string objectName = boost::to_upper_copy(action.object_name);
        for (unsigned int i = 0; i < recognizedObjects.objects.size(); i++)
        {
          if (recognizedObjects.objects[i].name == objectName)
          {
            ROS_INFO("Found object, attempting pickup...");
            rail_manipulation_msgs::PickupGoal pickupGoal;
            pickupGoal.lift = true;
            pickupGoal.verify = false;  //note: for now pickups are unverified always, because CARL can't do this.  If
            // possible, change this to always verify pickups.

            for (unsigned int j = 0; j < recognizedObjects.objects[i].grasps.size(); j++)
            {
              ROS_INFO("Attempting pickup with grasp %d", j);
              pickupGoal.pose = recognizedObjects.objects[i].grasps[j].grasp_pose;
              pickupClient.sendGoal(pickupGoal);
              while (!pickupClient.getState().isDone())
              {
                if (executeServer.isPreemptRequested())
                {
                  ROS_INFO("Canceling pickup goal...");
                  pickupClient.cancelAllGoals();
                  return rail_action_queue_msgs::GeneralAction::PREEMPTED;
                }
              }

              if (pickupClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickupClient.getResult()->executionSuccess)
              {
                ROS_INFO("Pickup failed due to planner, moving on to new grasp.");
              }
              else if (!pickupClient.getResult()->success)
              {
                ROS_INFO("Pickup failed during execution.");
                return rail_action_queue_msgs::GeneralAction::MANIPULATION_EXECUTION_FAILURE;
              }
              else
              {
                ROS_INFO("Pickup succeeded!");
                return rail_action_queue_msgs::GeneralAction::SUCCESS;
              }
            }

            return rail_action_queue_msgs::GeneralAction::MANIPULATION_PLANNING_FAILURE;
          }
        }
      }

      return rail_action_queue_msgs::GeneralAction::OBJECT_NOT_FOUND;
    }

    case rail_action_queue_msgs::GeneralAction::STORE:
    {
      rail_manipulation_msgs::StoreGoal storeGoal;
      storeGoal.store_pose = action.manipulation_pose;

      storeClient.sendGoal(storeGoal);
      while (!storeClient.getState().isDone())
      {
        if (executeServer.isPreemptRequested())
        {
          ROS_INFO("Canceling store goal...");
          storeClient.cancelAllGoals();
          return rail_action_queue_msgs::GeneralAction::PREEMPTED;
        }
      }

      if (storeClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !storeClient.getResult()->success)
      {
        ROS_INFO("Store failed.");
        return rail_action_queue_msgs::GeneralAction::MANIPULATION_PLANNING_FAILURE;
      }
      else
      {
        ROS_INFO("Store succeeded!");
        return rail_action_queue_msgs::GeneralAction::SUCCESS;
      }
    }


    case rail_action_queue_msgs::GeneralAction::SEGMENT:
    {
      std_srvs::Empty segmentSrv;
      if (!segmentClient.call(segmentSrv))
      {
        ROS_INFO("Couldn't call segmentation service.");
        return rail_action_queue_msgs::GeneralAction::SENSING_FAILURE;
      }

      return rail_action_queue_msgs::GeneralAction::SUCCESS;
    }


    case rail_action_queue_msgs::GeneralAction::SEGMENT_AND_RECOGNIZE:
    {
      std_srvs::Empty segmentSrv;
      recognizedObjectsCounter = 0;
      if (!segmentClient.call(segmentSrv))
      {
        ROS_INFO("Couldn't call segmentation service.");
        return rail_action_queue_msgs::GeneralAction::SENSING_FAILURE;
      }

      //spin and wait
      bool finished = false;
      while (!finished)
      {
        if (executeServer.isPreemptRequested())
        {
          ROS_INFO("Canceling action execution during recognition...");
          return rail_action_queue_msgs::GeneralAction::PREEMPTED;
        }

        {
          boost::mutex::scoped_lock lock(recognizedObjectsMutex);
          finished = recognizedObjectsCounter >= 2;
        }
      }

      return rail_action_queue_msgs::GeneralAction::SUCCESS;
    }

    default:
    {
      ROS_INFO("Action not implemented for nimbus...");
      return rail_action_queue_msgs::GeneralAction::SUCCESS;
    }
  }

  return rail_action_queue_msgs::GeneralAction::SUCCESS;
}

void NimbusActionQueue::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  boost::mutex::scoped_lock lock(recognizedObjectsMutex);

  recognizedObjects = objects;
  recognizedObjectsCounter++;
}


/*********************************************************************
 ************************** List Management **************************
 *********************************************************************/

bool NimbusActionQueue::addAction(rail_action_queue_msgs::AddAction::Request &req, rail_action_queue_msgs::AddAction::Response &res)
{
  action_list.push_back(req.action);
  return true;
}

bool NimbusActionQueue::clearActionList(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  action_list.clear();
  return true;
}

bool NimbusActionQueue::getActionList(rail_action_queue_msgs::GetActionList::Request &req, rail_action_queue_msgs::GetActionList::Response &res)
{
  res.action_list = action_list;
  return true;
}

bool NimbusActionQueue::insertAction(rail_action_queue_msgs::InsertAction::Request &req, rail_action_queue_msgs::InsertAction::Response &res)
{
  if (action_list.size() < req.index)
  {
    action_list.push_back(req.action);
  }
  else
  {
    action_list.insert(action_list.begin() + req.index, req.action);
  }
  return true;
}

bool NimbusActionQueue::removeAction(rail_action_queue_msgs::RemoveAction::Request &req, rail_action_queue_msgs::RemoveAction::Response &res)
{
  if (req.index < action_list.size())
  {
    action_list.erase(action_list.begin() + req.index);
  }
  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "nimbus_action_queue");

  // initialize the joystick controller
  NimbusActionQueue naq;

  ros::spin();

  return EXIT_SUCCESS;
}

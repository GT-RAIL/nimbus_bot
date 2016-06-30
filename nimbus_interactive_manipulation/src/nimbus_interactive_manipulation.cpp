#include <nimbus_interactive_manipulation/nimbus_interactive_manipulation.h>

using namespace std;

NimbusInteractiveManipulation::NimbusInteractiveManipulation() :
        acArm("nimbus_moveit/common_actions/arm_action"),
        acGripper("gripper_actions/gripper_manipulation"),
        acLift("nimbus_moveit/common_actions/lift"),
        acPickup("nimbus_moveit/common_actions/pickup", true)
{
  //read parameters
  ros::NodeHandle pnh("~");
  string segmentedObjectsTopic("/object_recognition_listener/recognized_objects");
  usingPickup = false;
  usingRecognition = false;
  pnh.getParam("using_pickup", usingPickup);
  pnh.getParam("using_recognition", usingRecognition);

  joints.resize(6);

  //messages
  cartesianCmd = n.advertise<wpi_jaco_msgs::CartesianCommand>("jaco_arm/cartesian_cmd", 1);
  if (usingRecognition)
    recognizedObjectsSubscriber = n.subscribe("object_recognition_listener/recognized_objects", 1, &NimbusInteractiveManipulation::segmentedObjectsCallback, this);
  else
    recognizedObjectsSubscriber = n.subscribe("rail_segmentation/segmented_objects", 1, &NimbusInteractiveManipulation::segmentedObjectsCallback, this);
  jointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &NimbusInteractiveManipulation::updateJoints, this);

  //services
  if (usingRecognition)
    removeObjectClient = n.serviceClient<rail_pick_and_place_msgs::RemoveObject>("/object_recognition_listener/remove_object");
  else
    removeObjectClient = n.serviceClient<rail_segmentation::RemoveObject>("rail_segmentation/remove_object");
  detachObjectsClient = n.serviceClient<std_srvs::Empty>("nimbus_moveit/detach_objects");
  jacoFkClient = n.serviceClient<wpi_jaco_msgs::JacoFK>("jaco_arm/kinematics/fk");
  qeClient = n.serviceClient<wpi_jaco_msgs::QuaternionToEuler>("jaco_conversions/quaternion_to_euler");
  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("jaco_arm/erase_trajectories");

  lockPose = false;

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_interactive_manipulation", "nimbus_markers", false));

  ros::Duration(0.1).sleep();

  makeHandMarker();

  //setup object menu
  if (usingPickup)
    objectMenuHandler.insert("Pickup", boost::bind(&NimbusInteractiveManipulation::processPickupMarkerFeedback, this, _1));
  objectMenuHandler.insert("Remove", boost::bind(&NimbusInteractiveManipulation::processRemoveMarkerFeedback, this, _1));

  imServer->applyChanges();
}

void NimbusInteractiveManipulation::updateJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (unsigned int i = 0; i < 6; i++)
  {
    joints.at(i) = msg->position.at(i);
  }
}

void NimbusInteractiveManipulation::makeHandMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "jaco_base_link";

  //initialize position to the jaco arm's current position
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
  {
    fkSrv.request.joints.push_back(joints.at(i));
  }
  if (jacoFkClient.call(fkSrv))
  {
    iMarker.pose = fkSrv.response.handPose.pose;
  }
  else
  {
    iMarker.pose.position.x = 0.0;
    iMarker.pose.position.y = 0.0;
    iMarker.pose.position.z = 0.0;
    iMarker.pose.orientation.x = 0.0;
    iMarker.pose.orientation.y = 0.0;
    iMarker.pose.orientation.z = 0.0;
    iMarker.pose.orientation.w = 1.0;
  }
  iMarker.scale = .2;

  iMarker.name = "jaco_hand_marker";
  iMarker.description = "JACO Hand Control";

  //make a sphere control to represent the end effector position
  visualization_msgs::Marker sphereMarker;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;
  sphereMarker.scale.x = iMarker.scale * 1;
  sphereMarker.scale.y = iMarker.scale * 1;
  sphereMarker.scale.z = iMarker.scale * 1;
  sphereMarker.color.r = .5;
  sphereMarker.color.g = .5;
  sphereMarker.color.b = .5;
  sphereMarker.color.a = 0.0;
  visualization_msgs::InteractiveMarkerControl sphereControl;
  sphereControl.markers.push_back(sphereMarker);
  sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  sphereControl.name = "jaco_hand_origin_marker";
  iMarker.controls.push_back(sphereControl);

  //add 6-DOF controls
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  //menu
  interactive_markers::MenuHandler::EntryHandle fingersSubMenuHandle = menuHandler.insert("Gripper");
  menuHandler.insert(fingersSubMenuHandle, "Close",
                     boost::bind(&NimbusInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert(fingersSubMenuHandle, "Open",
                     boost::bind(&NimbusInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Pickup", boost::bind(&NimbusInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Home", boost::bind(&NimbusInteractiveManipulation::processHandMarkerFeedback, this, _1));
  menuHandler.insert("Retract", boost::bind(&NimbusInteractiveManipulation::processHandMarkerFeedback, this, _1));

  visualization_msgs::InteractiveMarkerControl menuControl;
  menuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  menuControl.name = "jaco_hand_menu";
  iMarker.controls.push_back(menuControl);

  imServer->insert(iMarker);
  imServer->setCallback(iMarker.name, boost::bind(&NimbusInteractiveManipulation::processHandMarkerFeedback, this, _1));

  menuHandler.apply(*imServer, iMarker.name);
}

void NimbusInteractiveManipulation::processHandMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
  {
    //Send a stop command so that when the marker is released the arm stops moving
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0)
      {
        lockPose = true;
        sendStopCommand();
      }
          break;

          //Menu actions
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0)
      {
        if (feedback->menu_entry_id == 2)	//grasp requested
        {
          rail_manipulation_msgs::GripperGoal gripperGoal;
          gripperGoal.close = true;
          acGripper.sendGoal(gripperGoal);
        }
        else if (feedback->menu_entry_id == 3)	//release requested
        {
          rail_manipulation_msgs::GripperGoal gripperGoal;
          gripperGoal.close = false;
          acGripper.sendGoal(gripperGoal);

          std_srvs::Empty detachSrv;
          detachObjectsClient.call(detachSrv);
        }
        else if (feedback->menu_entry_id == 4)	//pickup requested
        {
          rail_manipulation_msgs::LiftGoal liftGoal;
          acLift.sendGoal(liftGoal);
        }
        else if (feedback->menu_entry_id == 5)  //home requested
        {
          acGripper.cancelAllGoals();
          acLift.cancelAllGoals();
          rail_manipulation_msgs::ArmGoal homeGoal;
          homeGoal.action = rail_manipulation_msgs::ArmGoal::READY;
          acArm.sendGoal(homeGoal);
          acArm.waitForResult(ros::Duration(10.0));
        }
        else if (feedback->menu_entry_id == 6)
        {
          acGripper.cancelAllGoals();
          acLift.cancelAllGoals();
          rail_manipulation_msgs::ArmGoal homeGoal;
          homeGoal.action = rail_manipulation_msgs::ArmGoal::RETRACT;
          acArm.sendGoal(homeGoal);
          acArm.waitForResult(ros::Duration(10.0));
        }
      }
          break;

          //Send movement commands to the arm to follow the pose marker
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0
          && feedback->control_name.compare("jaco_hand_origin_marker") != 0)
      {
        if (!lockPose)
        {
          acGripper.cancelAllGoals();
          acLift.cancelAllGoals();

          //convert pose for compatibility with JACO API
          wpi_jaco_msgs::QuaternionToEuler qeSrv;
          qeSrv.request.orientation = feedback->pose.orientation;
          if (qeClient.call(qeSrv))
          {
            wpi_jaco_msgs::CartesianCommand cmd;
            cmd.position = true;
            cmd.armCommand = true;
            cmd.fingerCommand = false;
            cmd.repeat = false;
            cmd.arm.linear.x = feedback->pose.position.x;
            cmd.arm.linear.y = feedback->pose.position.y;
            cmd.arm.linear.z = feedback->pose.position.z;
            cmd.arm.angular.x = qeSrv.response.roll;
            cmd.arm.angular.y = qeSrv.response.pitch;
            cmd.arm.angular.z = qeSrv.response.yaw;

            cartesianCmd.publish(cmd);
          }
          else
            ROS_INFO("Quaternion to Euler conversion service failed, could not send pose update");
        }
      }
          break;

          //Mouse down events
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      lockPose = false;
          break;

          //As with mouse clicked, send a stop command when the mouse is released on the marker
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      if (feedback->marker_name.compare("jaco_hand_marker") == 0)
      {
        lockPose = true;
        sendStopCommand();
      }
          break;
  }

  //Update interactive marker server
  imServer->applyChanges();
}

void NimbusInteractiveManipulation::sendStopCommand()
{
  wpi_jaco_msgs::CartesianCommand cmd;
  cmd.position = false;
  cmd.armCommand = true;
  cmd.fingerCommand = false;
  cmd.repeat = true;
  cmd.arm.linear.x = 0.0;
  cmd.arm.linear.y = 0.0;
  cmd.arm.linear.z = 0.0;
  cmd.arm.angular.x = 0.0;
  cmd.arm.angular.y = 0.0;
  cmd.arm.angular.z = 0.0;
  cartesianCmd.publish(cmd);

  std_srvs::Empty srv;
  if (!eraseTrajectoriesClient.call(srv))
  {
    ROS_INFO("Could not call erase trajectories service...");
  }
}

void NimbusInteractiveManipulation::updateMarkerPosition()
{
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
  {
    fkSrv.request.joints.push_back(joints.at(i));
  }

  if (jacoFkClient.call(fkSrv))
  {
    imServer->setPose("jaco_hand_marker", fkSrv.response.handPose.pose);
    imServer->applyChanges();
  }
  else
  {
    ROS_INFO("Failed to call forward kinematics service");
  }
}

void NimbusInteractiveManipulation::segmentedObjectsCallback(
    const rail_manipulation_msgs::SegmentedObjectList::ConstPtr& objectList)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for object list

  //store list of objects
  segmentedObjectList = *objectList;

  ROS_INFO("Received new segmented point clouds");
  clearSegmentedObjects();
  recognizedMenuHandlers.clear();
  recognizedMenuHandlers.resize(objectList->objects.size());
  for (unsigned int i = 0; i < objectList->objects.size(); i++)
  {
    visualization_msgs::InteractiveMarker objectMarker;
    objectMarker.header = objectList->objects[i].marker.header;

    objectMarker.pose.position.x = 0.0;
    objectMarker.pose.position.y = 0.0;
    objectMarker.pose.position.z = 0.0;
    objectMarker.pose.orientation.x = 0.0;
    objectMarker.pose.orientation.y = 0.0;
    objectMarker.pose.orientation.z = 0.0;
    objectMarker.pose.orientation.w = 1.0;

    stringstream ss;
    ss.str("");
    ss << "object" << i;
    objectMarker.name = ss.str();

    visualization_msgs::InteractiveMarkerControl objectControl;
    ss << "control";
    objectControl.name = ss.str();
    objectControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    //objectControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    objectControl.always_visible = true;
    objectControl.markers.resize(1);
    //objectControl.markers[0] = cloudMarker;
    objectControl.markers[0] = objectList->objects[i].marker;
    objectMarker.controls.push_back(objectControl);

    //object label
    visualization_msgs::InteractiveMarkerControl objectLabelControl;
    stringstream namestream;
    namestream.str("");
    namestream << "object" << i << "_label";
    objectLabelControl.name = namestream.str();
    objectLabelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    objectLabelControl.always_visible = true;
    visualization_msgs::Marker objectLabel;
    objectLabel.header = objectList->objects[i].point_cloud.header;
    objectLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    objectLabel.pose.position.x = objectList->objects[i].centroid.x;
    objectLabel.pose.position.y = objectList->objects[i].centroid.y;
    objectLabel.pose.position.z = objectList->objects[i].centroid.z + .1;
    objectLabel.scale.x = .1;
    objectLabel.scale.y = .1;
    objectLabel.scale.z = .1;
    objectLabel.color.r = 1.0;
    objectLabel.color.g = 1.0;
    objectLabel.color.b = 1.0;
    objectLabel.color.a = 1.0;
    objectLabel.text = objectList->objects[i].name;
    objectLabelControl.markers.resize(1);
    objectLabelControl.markers[0] = objectLabel;
    objectMarker.controls.push_back(objectLabelControl);

    //pickup menu
    visualization_msgs::InteractiveMarkerControl objectMenuControl;
    objectMenuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    ss << "_menu";
    objectMenuControl.name = ss.str();
    objectMarker.controls.push_back(objectMenuControl);

    imServer->insert(objectMarker);

    if (objectList->objects[i].recognized)
    {
      if (usingPickup)
      {
        stringstream ss2;
        ss2.str("");
        ss2 << "Pickup " << objectList->objects[i].name;
        recognizedMenuHandlers[i].insert(ss2.str(), boost::bind(&NimbusInteractiveManipulation::processPickupMarkerFeedback, this, _1));
      }
      recognizedMenuHandlers[i].insert("Remove", boost::bind(&NimbusInteractiveManipulation::processRemoveMarkerFeedback, this, _1));
      recognizedMenuHandlers[i].apply(*imServer, objectMarker.name);
    }
    else
    {
      objectMenuHandler.apply(*imServer, objectMarker.name);
    }

    segmentedObjects.push_back(objectMarker);
  }

  imServer->applyChanges();

  ROS_INFO("Point cloud markers created");
}

void NimbusInteractiveManipulation::clearSegmentedObjects()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for object list

  for (unsigned int i = 0; i < segmentedObjects.size(); i++)
  {
    stringstream ss;
    ss.str("");
    ss << "object" << i;
    imServer->erase(ss.str());
  }
  segmentedObjects.clear();

  imServer->applyChanges();
}

void NimbusInteractiveManipulation::processPickupMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for segmented objects

    rail_manipulation_msgs::PickupGoal pickupGoal;
    pickupGoal.lift = true;
    pickupGoal.verify = true;
    int objectIndex = atoi(feedback->marker_name.substr(6).c_str());
    for (unsigned int i = 0; i < segmentedObjectList.objects[objectIndex].grasps.size(); i ++)
    {
      ROS_INFO("ATTEMPTING PICKUP WITH GRASP %d", i);
      pickupGoal.pose = segmentedObjectList.objects[objectIndex].grasps[i].grasp_pose;
      acPickup.sendGoal(pickupGoal);
      acPickup.waitForResult(ros::Duration(30.0));

      rail_manipulation_msgs::PickupResultConstPtr pickupResult = acPickup.getResult();
      if (!pickupResult->executionSuccess)
      {
        ROS_INFO("PICKUP FAILED TO FULLY EXECUTE, moving on to a new grasp...");
        continue;
      }

      if (!pickupResult->success)
      {
        ROS_INFO("PICKUP WAS UNSUCCESSFUL, stopping pickup attempts...");
      }
      else
      {
        ROS_INFO("PICKUP SUCCEEDED!");
      }

      removeObjectMarker(objectIndex);
      break;
    }
    ROS_INFO("FINISHED ATTEMPTING PICKUPS");

    imServer->applyChanges();
  }
}

void NimbusInteractiveManipulation::processRemoveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for object list

    if (!removeObjectMarker(atoi(feedback->marker_name.substr(6).c_str())))
      return;

    imServer->applyChanges();
  }
}

bool NimbusInteractiveManipulation::removeObjectMarker(int index)
{
  rail_pick_and_place_msgs::RemoveObject::Request req;
  rail_pick_and_place_msgs::RemoveObject::Response res;
  req.index = index;
  if (!removeObjectClient.call(req, res))
  {
    ROS_INFO("Could not call remove object service.");
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_interactive_manipulation");

  NimbusInteractiveManipulation tim;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    tim.updateMarkerPosition();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

#include <nimbus_interactive_manipulation/nimbus6dofPlanning.h>

using namespace std;

Nimbus6dofPlanning::Nimbus6dofPlanning() :
        pnh("~"), pickupUnrecognizedClient("nimbus_moveit/common_actions/pickup_unrecognized"),
        specifiedGraspServer(pnh, "execute_grasp", boost::bind(&Nimbus6dofPlanning::executeGraspCallback, this, _1), false)
{
  joints.resize(6);

  //messages
  markerPosePublisher = pnh.advertise<geometry_msgs::PoseStamped>("gripper_marker_pose", 1);
  jointStateSubscriber = n.subscribe("jaco_arm/joint_states", 1, &Nimbus6dofPlanning::updateJoints, this);

  //services
  jacoFkClient = n.serviceClient<wpi_jaco_msgs::JacoFK>("jaco_arm/kinematics/fk");
  resetMarkerPositionServer = pnh.advertiseService("reset_marker_position", &Nimbus6dofPlanning::resetMarkerPositionCallback, this);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_interactive_manipulation", "nimbus_6dof_planning", false));

  ros::Duration(0.1).sleep();

  makeGripperMarker();

  imServer->applyChanges();
}

void Nimbus6dofPlanning::updateJoints(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (unsigned int i = 0; i < 6; i++)
  {
    joints.at(i) = msg->position.at(i);
  }
}

void Nimbus6dofPlanning::makeGripperMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "table_base_link";

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
  iMarker.scale = 1.0;

  iMarker.name = "nimbus_gripper";
  iMarker.description = "Set Nimbus' gripper pose";

  //make a sphere control to represent the end effector position
  visualization_msgs::Marker gripperMarker;
  gripperMarker.pose.position.x = -0.125;
  gripperMarker.pose.orientation.x = 0.7071;
  gripperMarker.pose.orientation.w = 0.7071;
  gripperMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  gripperMarker.mesh_resource = "package://agile_test_nodes/meshes/robotiq_85_base_link.dae";
  gripperMarker.scale.x = 1.0;
  gripperMarker.scale.y = 1.0;
  gripperMarker.scale.z = 1.0;
  gripperMarker.color.r = 1.0;
  gripperMarker.color.g = 1.0;
  gripperMarker.color.b = 0.0;
  gripperMarker.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl gripperControl;
  gripperControl.markers.push_back(gripperMarker);
  gripperControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  gripperControl.name = "gripper_control";

  iMarker.controls.push_back(gripperControl);

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

  imServer->insert(iMarker);
}

void Nimbus6dofPlanning::executeGraspCallback(const nimbus_interactive_manipulation::SpecifiedGraspGoalConstPtr &goal)
{
  nimbus_interactive_manipulation::SpecifiedGraspFeedback feedback;
  nimbus_interactive_manipulation::SpecifiedGraspResult result;

  feedback.message = "Moving arm to your set position...";
  specifiedGraspServer.publishFeedback(feedback);

  visualization_msgs::InteractiveMarker poseMarker;
  imServer->get("nimbus_gripper", poseMarker);

  rail_manipulation_msgs::PickupGoal pickupGoal;
  pickupGoal.pose.header = poseMarker.header;
  pickupGoal.pose.pose = poseMarker.pose;
  pickupGoal.lift = false;
  pickupGoal.verify = false;
  pickupUnrecognizedClient.sendGoal(pickupGoal);
  pickupUnrecognizedClient.waitForResult(ros::Duration(30.0));
  rail_manipulation_msgs::PickupResultConstPtr pickupResult = pickupUnrecognizedClient.getResult();
  result.success = pickupResult->success;
  result.executionSuccess = pickupResult->executionSuccess;
  if (!pickupResult->executionSuccess)
  {
    ROS_INFO("Grasp failed!");
    feedback.message = "The robot failed to plan to your selected grasp.";
  }
  else
  {
    ROS_INFO("Grasp succeeded.");
    feedback.message = "Success!";
  }
  specifiedGraspServer.publishFeedback(feedback);
  specifiedGraspServer.setSucceeded(result);
}

bool Nimbus6dofPlanning::resetMarkerPositionCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  updateMarkerPosition();

  return true;
}

void Nimbus6dofPlanning::updateMarkerPosition()
{
  wpi_jaco_msgs::JacoFK fkSrv;
  for (unsigned int i = 0; i < 6; i++)
  {
    fkSrv.request.joints.push_back(joints.at(i));
  }

  if (jacoFkClient.call(fkSrv))
  {
    imServer->setPose("nimbus_gripper", fkSrv.response.handPose.pose);
    imServer->applyChanges();
  }
  else
  {
    ROS_INFO("Failed to call forward kinematics service");
  }
}

void Nimbus6dofPlanning::publishMarkerPosition()
{
  visualization_msgs::InteractiveMarker poseMarker;
  imServer->get("nimbus_gripper", poseMarker);

  geometry_msgs::PoseStamped pose;
  pose.header = poseMarker.header;
  pose.pose = poseMarker.pose;

  markerPosePublisher.publish(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_6dof_planning");

  Nimbus6dofPlanning n6p;

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    n6p.publishMarkerPosition();
    ros::spinOnce();
    loopRate.sleep();
  }
}

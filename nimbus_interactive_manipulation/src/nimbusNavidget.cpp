#include <nimbus_interactive_manipulation/nimbusNavidget.h>

using namespace std;

NimbusNavidget::NimbusNavidget() :
        pnh("~"), pickupUnrecognizedClient("nimbus_moveit/common_actions/pickup_unrecognized"),
        specifiedGraspServer(pnh, "execute_grasp", boost::bind(&NimbusNavidget::executeGraspCallback, this, _1), false)
{
  //services
  clearGripperMarkerServer = pnh.advertiseService("clear_gripper_marker", &NimbusNavidget::clearGripperMarkerCallback, this);
  clearFullMarkerServer = pnh.advertiseService("clear_full_marker", &NimbusNavidget::clearFullMarkerCallback, this);
  createNavidgetServer = pnh.advertiseService("create_navidget_sphere", &NimbusNavidget::createSphereMarkerCallback, this);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_navidget", "nimbus_navidget", false));

  ros::Duration(0.1).sleep();

  imServer->applyChanges();

  specifiedGraspServer.start();
}

bool NimbusNavidget::createSphereMarkerCallback(nimbus_interactive_manipulation::CreateNavidgetSphere::Request &req, nimbus_interactive_manipulation::CreateNavidgetSphere::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);

  imServer->clear();

  visualization_msgs::InteractiveMarker navidgetSphere;
  navidgetSphere.header.frame_id = req.point.header.frame_id;
  navidgetSphere.pose.position.x = req.point.point.x;
  navidgetSphere.pose.position.y = req.point.point.y;
  navidgetSphere.pose.position.z = req.point.point.z;
  navidgetSphere.pose.orientation.w = 1.0;

  navidgetSphere.scale = 1.0;

  navidgetSphere.name = "navidget_sphere";
  navidgetSphere.description = "Specify a grasp";

  visualization_msgs::Marker sphereMarker;
  sphereMarker.ns = "navidget";
  sphereMarker.id = 0;
  sphereMarker.type = visualization_msgs::Marker::SPHERE;

  sphereMarker.pose.orientation.w = 1.0;
  sphereMarker.scale.x = 0.3;
  sphereMarker.scale.y = 0.3;
  sphereMarker.scale.z = 0.3;
  sphereMarker.color.r = 0.8;
  sphereMarker.color.g = 0.3;
  sphereMarker.color.b = 0.1;
  sphereMarker.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl sphereMarkerControl;
  sphereMarkerControl.name = "navidget_sphere_control";
  sphereMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  sphereMarkerControl.always_visible = true;
  sphereMarkerControl.description = "Click to set an approach angle";
  sphereMarkerControl.markers.push_back(sphereMarker);

  visualization_msgs::Marker centerMarker;
  centerMarker.ns = "navidget";
  centerMarker.id = 1;
  centerMarker.type = visualization_msgs::Marker::SPHERE;
  centerMarker.pose.orientation.w = 1.0;
  centerMarker.scale.x = 0.02;
  centerMarker.scale.y = 0.02;
  centerMarker.scale.z = 0.02;
  centerMarker.color.r = 0.0;
  centerMarker.color.g = 0.0;
  centerMarker.color.b = 1.0;
  centerMarker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl centerMarkerControl;
  centerMarkerControl.name = "navidget_center_control";
  centerMarkerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  centerMarkerControl.always_visible = true;
  centerMarkerControl.description = "";
  centerMarkerControl.markers.push_back(centerMarker);

  navidgetSphere.controls.push_back(sphereMarkerControl);
  navidgetSphere.controls.push_back(centerMarkerControl);

  imServer->insert(navidgetSphere);
  imServer->setCallback(navidgetSphere.name, boost::bind(&NimbusNavidget::processMarkerFeedback, this, _1));
  imServer->applyChanges();

  return true;
}

void NimbusNavidget::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);
  if (feedback->control_name == "navidget_sphere_control" && feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    if (feedback->mouse_point_valid)
    {
      visualization_msgs::InteractiveMarker navidgetSphere;
      imServer->get("navidget_sphere", navidgetSphere);
      visualization_msgs::Marker sphere = navidgetSphere.controls[0].markers[0];
      double roll = 0;
      double pitch = -asin((navidgetSphere.pose.position.z - feedback->mouse_point.z)/sqrt(pow(feedback->mouse_point.x - navidgetSphere.pose.position.x, 2) + pow(feedback->mouse_point.y - navidgetSphere.pose.position.y, 2) + pow(feedback->mouse_point.z - navidgetSphere.pose.position.z, 2)));
      double yaw = atan2(navidgetSphere.pose.position.y - feedback->mouse_point.y, navidgetSphere.pose.position.x - feedback->mouse_point.x);
      //ROS_INFO("Mouse point: %f, %f, %f", feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
      //ROS_INFO("Sphere center: %f, %f, %f", navidgetSphere.pose.position.x, navidgetSphere.pose.position.y, navidgetSphere.pose.position.z);
      //ROS_INFO("Roll, Pitch, Yaw: %f, %f, %f", roll, pitch, yaw);
      geometry_msgs::QuaternionStamped orientation;
      orientation.quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      orientation.header.frame_id = navidgetSphere.header.frame_id;

      visualization_msgs::InteractiveMarker gripperMarker;
      if (imServer->get("nimbus_gripper", gripperMarker))
      {
        geometry_msgs::Pose updatedPose;
        updatedPose.position = feedback->mouse_point;
        updatedPose.orientation = orientation.quaternion;
        imServer->setPose("nimbus_gripper", updatedPose);
      }
      else
      {
        makeGripperMarker(feedback->mouse_point, orientation);
      }
      imServer->applyChanges();
    }
    else
    {
      ROS_INFO("Invalid mouse point");
    }
  }
}

void NimbusNavidget::makeGripperMarker(geometry_msgs::Point point, geometry_msgs::QuaternionStamped orientation)
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = orientation.header.frame_id;

  iMarker.pose.position = point;
  iMarker.pose.orientation = orientation.quaternion;

  iMarker.scale = 0.2;

  iMarker.name = "nimbus_gripper";
  iMarker.description = "Fine-tune the gripper pose";

  //gripper mesh marker
  visualization_msgs::Marker gripperBase = createGripperMeshMarker(
      -0.055, 0, 0,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae");
  visualization_msgs::Marker gripperLeftKnuckle = createGripperMeshMarker(
      -0.001, 0, -0.031,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae");
  visualization_msgs::Marker gripperRightKnuckle = createGripperMeshMarker(
      -0.001, 0, 0.031,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae");
  visualization_msgs::Marker gripperLeftFinger = createGripperMeshMarker(
      -0.005, 0, -0.062,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae");
  visualization_msgs::Marker gripperRightFinger = createGripperMeshMarker(
      -0.005, 0, 0.062,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae");
  visualization_msgs::Marker gripperLeftInnerKnuckle = createGripperMeshMarker(
      0.006, 0, -0.013,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae");
  visualization_msgs::Marker gripperRightInnerKnuckle = createGripperMeshMarker(
      0.006, 0, 0.013,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae");
  visualization_msgs::Marker gripperLeftFingerTip = createGripperMeshMarker(
      0.049, 0, -0.05,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae");
  visualization_msgs::Marker gripperRightFingerTip = createGripperMeshMarker(
      0.049, 0, 0.05,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae");

  visualization_msgs::InteractiveMarkerControl gripperControl;
  gripperControl.markers.push_back(gripperBase);
  gripperControl.markers.push_back(gripperLeftKnuckle);
  gripperControl.markers.push_back(gripperRightKnuckle);
  gripperControl.markers.push_back(gripperLeftFinger);
  gripperControl.markers.push_back(gripperRightFinger);
  gripperControl.markers.push_back(gripperLeftInnerKnuckle);
  gripperControl.markers.push_back(gripperRightInnerKnuckle);
  gripperControl.markers.push_back(gripperLeftFingerTip);
  gripperControl.markers.push_back(gripperRightFingerTip);
  gripperControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  gripperControl.name = "gripper_control";
  gripperControl.always_visible = true;

  iMarker.controls.push_back(gripperControl);

  //add controls
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);

  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  imServer->insert(iMarker);
}

visualization_msgs::Marker NimbusNavidget::createGripperMeshMarker(double x, double y, double z, double rx, double ry, double rz, double rw, string meshLocation)
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = rx;
  marker.pose.orientation.y = ry;
  marker.pose.orientation.z = rz;
  marker.pose.orientation.w = rw;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = meshLocation;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.65;
  marker.color.g = 0.0;
  marker.color.b = 0.65;
  marker.color.a = 1.0;

  return marker;
}


void NimbusNavidget::executeGraspCallback(const nimbus_interactive_manipulation::SpecifiedGraspGoalConstPtr &goal)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);

  nimbus_interactive_manipulation::SpecifiedGraspFeedback feedback;
  nimbus_interactive_manipulation::SpecifiedGraspResult result;

  feedback.message = "Moving arm to your set position...";
  specifiedGraspServer.publishFeedback(feedback);

  visualization_msgs::InteractiveMarker poseMarker;
  if (!imServer->get("nimbus_gripper", poseMarker))
  {
    feedback.message = "Please specify a grasp first.  See the instructions pane for details.";
    specifiedGraspServer.publishFeedback(feedback);
    result.success = false;
    result.executionSuccess = false;
    specifiedGraspServer.setSucceeded(result);
    return;
  }

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
    feedback.message = "Failed to plan to your grasp. Try a different start or end pose, and watch out for collisions.";
  }
  else
  {
    ROS_INFO("Grasp succeeded.");
    feedback.message = "Success!";
  }
  specifiedGraspServer.publishFeedback(feedback);
  specifiedGraspServer.setSucceeded(result);
}

bool NimbusNavidget::clearGripperMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);
  imServer->erase("nimbus_gripper");
  imServer->applyChanges();

  return true;
}

bool NimbusNavidget::clearFullMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(graspMutex);
  imServer->clear();
  imServer->applyChanges();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_navidget");

  NimbusNavidget nn;

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
  }
}

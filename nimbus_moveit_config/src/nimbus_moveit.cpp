#include <nimbus_moveit_config/nimbus_moveit.h>

using namespace std;

NimbusMoveIt::NimbusMoveIt() :
    pnh("~"), tfListener(tfBuffer),
    armTrajectoryClient("/j2s7s300/follow_joint_trajectory"),
    moveToPoseServer(pnh, "move_to_pose", boost::bind(&NimbusMoveIt::moveToPose, this, _1), false),
    moveToJointPoseServer(pnh, "move_to_joint_pose", boost::bind(&NimbusMoveIt::moveToJointPose, this, _1), false)
{
  gripperNames.push_back("j2s7s300_base_link");
  gripperNames.push_back("j2s7s300_left_finger_link");
  gripperNames.push_back("j2s7s300_left_finger_tip_link");
  gripperNames.push_back("j2s7s300_left_inner_knuckle_link");
  gripperNames.push_back("j2s7s300_left_knuckle_link");
  gripperNames.push_back("j2s7s300_right_finger_link");
  gripperNames.push_back("j2s7s300_right_finger_tip_link");
  gripperNames.push_back("j2s7s300_right_inner_knuckle_link");
  gripperNames.push_back("j2s7s300_right_knuckle_link");

  ignoredObject = "";

  armJointStateSubscriber = n.subscribe("joint_states", 1, &NimbusMoveIt::armJointStatesCallback, this);
  //cartesianControlSubscriber = pnh.subscribe("cartesian_control", 1, &NimbusMoveIt::cartesianControlCallback, this);
  //armHomedSubscriber = n.subscribe("jaco_arm/arm_homed", 1, &NimbusMoveIt::armHomedCallback, this);
  recognizedObjectsSubscriber = n.subscribe("object_recognition_listener/recognized_objects", 1, &NimbusMoveIt::recognizedObjectsCallback, this);
  followJointTrajectoryResultSubscriber = n.subscribe("jaco_arm/joint_velocity_controller/trajectory/result", 1, &NimbusMoveIt::followJointTrajectoryResultCallback, this);

  trajectoryVisPublisher = pnh.advertise<moveit_msgs::DisplayTrajectory>("computed_trajectory", 1);
  graspingStatePublisher = pnh.advertise<rail_manipulation_msgs::GraspingState>("grasping_state", 1);
  planningScenePublisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene",1);

  ikClient = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  clearOctomapClient = n.serviceClient<std_srvs::Empty>("clear_octomap");
  planningSceneClient = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  jacoArmGroup = new move_group_interface::MoveGroup("arm");
  jacoArmGroup->startStateMonitor();

  planningSceneInterface = new move_group_interface::PlanningSceneInterface();
  /*planningSceneMonitor = planning_scene_monitor::PlanningSceneMonitorPtr(new
                                                                            planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  */

  //advertise service
  cartesianPathServer = pnh.advertiseService("cartesian_path", &NimbusMoveIt::cartesianPathCallback, this);
  ikServer = pnh.advertiseService("call_ik", &NimbusMoveIt::ikCallback, this);
  attachObjectServer = pnh.advertiseService("attach_closest_object", &NimbusMoveIt::attachClosestSceneObject, this);
  detachObjectServer = pnh.advertiseService("detach_objects", &NimbusMoveIt::detachSceneObjects, this);
  prepareGraspServer = pnh.advertiseService("prepare_grasp", &NimbusMoveIt::prepareGrasp, this);
  reactivateGraspCollisionServer = pnh.advertiseService("reactivate_grasp_collision", &NimbusMoveIt::reactivateGraspCollision, this);

  //start action server
  moveToPoseServer.start();
  moveToJointPoseServer.start();

  cout << "End effector link: " << jacoArmGroup->getEndEffectorLink() << endl;

  /*
  moveit_msgs::GetPlanningScene planningSceneSrv;
  planningSceneSrv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  if (!planningSceneClient.call(planningSceneSrv))
  {
    ROS_INFO("Could not get the current planning scene.");
    ROS_INFO("COLLISIONS BETWEEN <octomap> AND simplified geometry WILL NOT BE IGNORED!!!");
    return;
  }

  collision_detection::AllowedCollisionMatrix acm(planningSceneSrv.response.scene.allowed_collision_matrix);
  //Disable collisions on self-filter geometry
  acm.setEntry("<octomap>", simplifiedGeometryNames, true);
  moveit_msgs::PlanningScene planningSceneUpdate;
  acm.getMessage(planningSceneUpdate.allowed_collision_matrix);
  planningSceneUpdate.is_diff = true;
  planningScenePublisher.publish(planningSceneUpdate);

  ros::Duration(0.5).sleep(); //delay for publish to go through
  */
}

NimbusMoveIt::~NimbusMoveIt()
{
  delete jacoArmGroup;
  delete planningSceneInterface;
}

/*
void NimbusMoveIt::armHomedCallback(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    std_srvs::Empty srv;
    if (!clearOctomapClient.call(srv))
      ROS_INFO("Failed to call clear octomap service.");
  }
}
*/

void NimbusMoveIt::armJointStatesCallback(const sensor_msgs::JointState &msg)
{
  jointState = msg;
}

/** Adjust angle to equivalent angle on [-pi, pi]
*  @param angle the angle to be simplified (-inf, inf)
*  @return the simplified angle on [-pi, pi]
*/
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

/** Calculates nearest desired angle to the current angle
*  @param desired desired joint angle [-pi, pi]
*  @param current current angle (-inf, inf)
*  @return the closest equivalent angle (-inf, inf)
*/
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

void NimbusMoveIt::moveToPose(const rail_manipulation_msgs::MoveToPoseGoalConstPtr &goal)
{
  moveit_msgs::GetPositionIK::Response ikRes = callIK(goal->pose);

  rail_manipulation_msgs::MoveToPoseResult result;
  if (ikRes.error_code.val == ikRes.error_code.SUCCESS)
  {
    ROS_INFO("IK service call succeeded");

    double jointStateDifference = 0.0;
    //set joints to be closest to current joint positions
    for (unsigned int i = 0; i < ikRes.solution.joint_state.name.size(); i++)
    {
      int jointStateIndex = distance(jointState.name.begin(), find(jointState.name.begin(), jointState.name.end(), ikRes.solution.joint_state.name[i]));
      double value;
      if (ikRes.solution.joint_state.name[i] == "j2s7s300_joint_2" || ikRes.solution.joint_state.name[i] == "j2s7s300_joint_4" || ikRes.solution.joint_state.name[i] == "j2s7s300_joint_6")
        value = ikRes.solution.joint_state.position[i];
      else
        value = nearest_equivalent(simplify_angle(ikRes.solution.joint_state.position[i]), jointState.position[jointStateIndex]);
      jacoArmGroup->setJointValueTarget(ikRes.solution.joint_state.name[i], value);
      jointStateDifference += fabs(value - jointState.position[jointStateIndex]);
    }

    if (goal->jointStateDifference > 0.0 && jointStateDifference > goal->jointStateDifference)
    {
      result.ikSuccess = false;
      result.success = false;
      moveToPoseServer.setSucceeded(result);
    }
    else
    {
      result.ikSuccess = true;
    }

    //plan and execute
    jacoArmGroup->setPlannerId("arm[RRTConnectkConfigDefault]");

    if (goal->planningTime == 0.0)
      jacoArmGroup->setPlanningTime(5.0);
    else
      jacoArmGroup->setPlanningTime(goal->planningTime);
    jacoArmGroup->setStartStateToCurrentState();
    ROS_INFO("Planning and moving...");
    //armGroup->asyncMove();
    move_group_interface::MoveItErrorCode errorCode = jacoArmGroup->move();
    ROS_INFO("Finished plan and move");
    if (errorCode == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Succeeded");
      result.success = true;
    }
    else
    {
      ROS_INFO("Failed with MoveIt error code: %d", errorCode.val);
      if (errorCode == moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)  //NOTE: temporary fix for undesired grasping behavior
      {
        ROS_INFO("Ignoring failure, continuing execution...");
        result.success = true;
      }
      else
        result.success = false;
    }
  }
  else
  {
    ROS_INFO("IK service call failed with error code: %d", ikRes.error_code.val);
    result.success = false;
  }

  moveToPoseServer.setSucceeded(result);
}

void NimbusMoveIt::moveToJointPose(const rail_manipulation_msgs::MoveToJointPoseGoalConstPtr &goal)
{
  //extract joint states
  int jacoStartIndex = distance(jointState.name.begin(), find(jointState.name.begin(), jointState.name.end(), "j2s7s300_joint_1"));
  /*
  for (unsigned int i = 0; i < jointState.name.size(); i++)
  {
    if (jointState.name[i].compare("jaco_shoulder_pan_joint") == 0)
    {
      jacoStartIndex = i;
      break;
    }
  }
  */

  cout << "Planning to pose: ";
  for (unsigned int i = 0; i < goal->joints.size(); i ++)
  {
    cout << goal->joints[i] << " ";
  }
  cout << endl;

  //set planning goal for joints to be closest to current joint positions
  for (unsigned int i = jacoStartIndex; i < jacoStartIndex + NUM_JACO_JOINTS; i++)
  {
    if ((i - jacoStartIndex) % 2 == 0)
      jacoArmGroup->setJointValueTarget(jointState.name[i], nearest_equivalent(simplify_angle(goal->joints[i - jacoStartIndex]), jointState.position[i]));
    else
      jacoArmGroup->setJointValueTarget(jointState.name[i], goal->joints[i - jacoStartIndex]);
  }

  //plan and execute
  rail_manipulation_msgs::MoveToJointPoseResult result;
  jacoArmGroup->setPlannerId("arm[RRTConnectkConfigDefault]");
  if (goal->planningTime == 0.0)
    jacoArmGroup->setPlanningTime(5.0);
  else
    jacoArmGroup->setPlanningTime(goal->planningTime);
  jacoArmGroup->setStartStateToCurrentState();
  //jacoArmGroup->setJointValueTarget(jointGoal);
  ROS_INFO("Planning and moving...");
  //armGroup->asyncMove();
  move_group_interface::MoveItErrorCode errorCode = jacoArmGroup->move();
  ROS_INFO("Finished plan and move");
  if (errorCode == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Succeeded");
    result.success = true;
  }
  else
  {
    ROS_INFO("Failed with MoveIt error code: %d", errorCode.val);
    result.success = false;
  }

  moveToJointPoseServer.setSucceeded(result);
}

bool NimbusMoveIt::cartesianPathCallback(rail_manipulation_msgs::CartesianPath::Request &req, rail_manipulation_msgs::CartesianPath::Response &res)
{
  double eefStep = .05;
  double  jumpThreshold = 1.5;
  moveit_msgs::RobotTrajectory finalTraj;

  //convert waypoints to correct frame
  vector<geometry_msgs::Pose> convertedWaypoints;
  for (unsigned int i = 0; i < req.waypoints.size(); i ++)
  {
    geometry_msgs::PoseStamped tempPose;
    tempPose.header.frame_id = jacoArmGroup->getPoseReferenceFrame();
    //tfBuffer.transform(req.waypoints[i], tempPose, jacoArmGroup->getPoseReferenceFrame(), ros::Time(0), "table_base_link");
    tf.transformPose(jacoArmGroup->getPoseReferenceFrame(), req.waypoints[i], tempPose);
    convertedWaypoints.push_back(tempPose.pose);
  }

  //calculate trajectory
  moveit_msgs::RobotTrajectory tempTraj;
  double completion = jacoArmGroup->computeCartesianPath(convertedWaypoints, eefStep, jumpThreshold, tempTraj, req.avoidCollisions);
  if (completion == -1)
  {
    ROS_INFO("Could not calculate a path.");
    res.success = false;
    return true;
  }

  if (completion == 1.0)
  {
    finalTraj = tempTraj;
  }
  else
  {
    finalTraj = tempTraj;
    ROS_INFO("Could not find a complete path, varying parameters and recalculating...");
    //vary jumpThreshold and eefStep
    double newCompletion;
      for (unsigned int j = 0; j < 3; j ++)
      {
        if (j == 0)
          eefStep /= 2.0;
        else
          eefStep *= 2.0;
        newCompletion = jacoArmGroup->computeCartesianPath(convertedWaypoints, eefStep, jumpThreshold, tempTraj, req.avoidCollisions);
        if (newCompletion > completion)
        {
          ROS_INFO("Found a better path.");
          finalTraj = tempTraj;
          completion = newCompletion;
          if (newCompletion == 1.0)
          {
            ROS_INFO("Found a complete path!");
            break;
          }
        }
      }
  }

  if (completion == 0.0)
  {
    ROS_INFO("Could not calculate a path.");
    res.success = false;
    return true;
  }
  else
  {
    res.success = true;
    res.completion = completion;
  }

  //display trajectory
  /*
  moveit_msgs::DisplayTrajectory trajVis;
  trajVis.model_id = "nimbus";
  trajVis.trajectory.clear();
  trajVis.trajectory.push_back(finalTraj);
  moveit::core::robotStateToRobotStateMsg(*(armGroup->getCurrentState()), trajVis.trajectory_start);
  trajectoryVisPublisher.publish(trajVis);
  */

  //execute the trajectory
  move_group_interface::MoveGroup::Plan plan;
  plan.trajectory_ = finalTraj;
  moveit::core::robotStateToRobotStateMsg(*(jacoArmGroup->getCurrentState()), plan.start_state_);
  plan.planning_time_ = 1.0; //does this matter?

  ROS_INFO("Calculated a trajectory with %lu points:", plan.trajectory_.joint_trajectory.points.size());
  for (unsigned int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i ++)
  {
    for (unsigned int j = 0; j < plan.trajectory_.joint_trajectory.joint_names.size(); j ++)
    {
      cout << "\t" << plan.trajectory_.joint_trajectory.joint_names[j] << ": " << plan.trajectory_.joint_trajectory.points[i].positions[j] << endl;
    }
  }

  //jacoArmGroup->asyncExecute(plan);
  jacoArmGroup->execute(plan);

  //TODO: timeout should be a function of distance (calculated as (goal - start)*completion)
  executionFinished = false;
  ros::Rate loopRate(30);
  ros::Time timeout = ros::Time::now() + ros::Duration(1.5);
  res.success = true;
  while (!executionFinished)
  {
    if (ros::Time::now() >= timeout)
    {
      jacoArmGroup->stop();
      res.success = false;
      ROS_INFO("Timeout reached, stopping trajectory execution.");
      return true;
    }
    loopRate.sleep();
    ros::spinOnce();
  }

  return true;
}

void NimbusMoveIt::followJointTrajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult &msg)
{
  executionFinished = true;
}

bool NimbusMoveIt::ikCallback(rail_manipulation_msgs::CallIK::Request &req, rail_manipulation_msgs::CallIK::Response &res)
{
  moveit_msgs::GetPositionIK::Response ikRes = callIK(req.pose);

  if (ikRes.error_code.val == ikRes.error_code.SUCCESS)
  {
    ROS_INFO("IK service call succeeded");

    //extract joint states
    int jacoStartIndex = distance(jointState.name.begin(), find(jointState.name.begin(), jointState.name.end(), "j2s7s300_joint_1"));
    /*
    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
      if (jointState.name[i].compare("jaco_shoulder_pan_joint") == 0)
      {
        jacoStartIndex = i;
        break;
      }
    }
    */

    std::vector<double> joints;
    joints.resize(NUM_JACO_JOINTS);
    //set joints to be closest to current joint positions
    for (unsigned int i = jacoStartIndex; i < jacoStartIndex + NUM_JACO_JOINTS; i++)
    {
      joints[i - jacoStartIndex] = nearest_equivalent(simplify_angle(ikRes.solution.joint_state.position[i]), jointState.position[i]);
    }

    res.jointPositions = joints;
    res.success = true;
  }
  else
  {
    ROS_INFO("IK service call failed with error code: %d", ikRes.error_code.val);
    res.success = false;
  }

  return true;
}

moveit_msgs::GetPositionIK::Response NimbusMoveIt::callIK(geometry_msgs::PoseStamped pose)
{
  moveit_msgs::GetPositionIK::Request ikReq;
  moveit_msgs::GetPositionIK::Response ikRes;

  //robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(kinematicModel));
  robot_state::RobotStatePtr kinematicState = jacoArmGroup->getCurrentState();
  const robot_state::JointModelGroup *jointModelGroup = kinematicState->getRobotModel()->getJointModelGroup("arm");
  //kinematicState->setVariableValues(jointState);

  ikReq.ik_request.group_name = "arm";
  ikReq.ik_request.pose_stamped = pose;
  ikReq.ik_request.ik_link_name = "nimbus_ee_link";
  //seed state
  ikReq.ik_request.robot_state.joint_state.name = jointModelGroup->getJointModelNames();
  kinematicState->copyJointGroupPositions(jointModelGroup, ikReq.ik_request.robot_state.joint_state.position);
  //other parameters
  //ikReq.ik_request.avoid_collisions = true;
  ikReq.ik_request.timeout = ros::Duration(.1);
  ikReq.ik_request.attempts = 10;

  ikClient.call(ikReq, ikRes);

  return ikRes;
}

/*
void NimbusMoveIt::cartesianControlCallback(const geometry_msgs::Twist &msg)
{
  //get the jacobian
  robot_state::RobotStatePtr kinematicState = jacoArmGroup->getCurrentState();
  const moveit::core::JointModelGroup* jointModelGroup = kinematicState->getRobotModel()->getJointModelGroup("jaco_arm");
  Eigen::Vector3d referencePointPosition(0.0, 0.0, 0.0);  //what does this do?
  Eigen::MatrixXd jacobian;
  kinematicState->getJacobian(jointModelGroup, kinematicState->getLinkModel(jointModelGroup->getLinkModelNames().back()), referencePointPosition, jacobian);

  //calculate the jacobian pseudoinverse

  //Method 1: SVD
  Eigen::MatrixXd pInv = EIGEN_PINV::pinv(jacobian, 0.001);

*/
  //Method 2: Permuting the jacobian's main diagonal
  /*
  Eigen::MatrixXd pInv;
  float val = .001;
  Eigen::MatrixXd permutedJacobian = jacobian + val*Eigen::MatrixXd::Identity(6, 6);
  pInv = permutedJacobian.inverse();
  */

/*
  //calculate joint velocities
  Eigen::VectorXd twist(6);
  twist << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;
  Eigen::VectorXd jointVel(6);
  jointVel = pInv * twist;

  //publish joint velocity command to the arm
  wpi_jaco_msgs::AngularCommand cmd;
  cmd.position = false;
  cmd.armCommand = true;
  cmd.fingerCommand = false;
  cmd.repeat = true;
  cmd.joints.resize(6);
  for(unsigned int i = 0; i < jointVel.size(); i ++)
  {
    cmd.joints[i] = jointVel[i];
  }
  angularCmdPublisher.publish(cmd);
}
*/

void NimbusMoveIt::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  //remove previously detected collision objects
  unattachedObjects.clear();  //clear list of unattached scene object names
  vector<string> previousObjects = planningSceneInterface->getKnownObjectNames();
  if (!attachedObjects.empty())
  {
    //don't remove the attached object
    for (unsigned int i = 0; i < previousObjects.size(); i ++)
    {
      for (unsigned int j = 0; j < attachedObjects.size(); j ++)
      if (previousObjects[i] == attachedObjects[j])
      {
        previousObjects.erase(previousObjects.begin() + i);
        i --;
        break;
      }
    }
  }
  planningSceneInterface->removeCollisionObjects(previousObjects);

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for the stored objects array

    //store objects
    objectList = msg;

    if (!msg.objects.empty())
    {
      //add all objects to the planning scene
      vector<moveit_msgs::CollisionObject> collisionObjects;
      collisionObjects.resize(msg.objects.size());
      for (unsigned int i = 0; i < collisionObjects.size(); i++)
      {
        //create collision object
        collisionObjects[i].header.frame_id = msg.objects[i].point_cloud.header.frame_id;
        stringstream ss;
        if (msg.objects[i].recognized)
          ss << msg.objects[i].name << i;
        else
          ss << "object" << i;
        //check for name collisions
        for (unsigned int j = 0; j < attachedObjects.size(); j ++)
        {
          if (ss.str() == attachedObjects[j])
            ss << "(2)";
        }
        collisionObjects[i].id = ss.str();
        unattachedObjects.push_back(ss.str());

        //convert point cloud to pcl point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCLPointCloud2 converter;
        pcl_conversions::toPCL(msg.objects[i].point_cloud, converter);
        pcl::fromPCLPointCloud2(converter, *objectCloud);

        // compute principal direction
        Eigen::Matrix3f covariance;
        Eigen::Vector4f centroid;
        centroid[0] = msg.objects[i].centroid.x;
        centroid[1] = msg.objects[i].centroid.y;
        centroid[2] = msg.objects[i].centroid.z;
        pcl::computeCovarianceMatrixNormalized(*objectCloud, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eig_dx = eigen_solver.eigenvectors();
        eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

        //move the points to that reference frame
        Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
        p2w.block(0, 0, 3, 3) = eig_dx.transpose();
        p2w.block(0, 3, 3, 1) = -1.f * (p2w.block(0, 0, 3, 3) * centroid.head(3));
        pcl::PointCloud<pcl::PointXYZRGB> c_points;
        pcl::transformPointCloud(*objectCloud, c_points, p2w);

        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(c_points, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

        //final transform
        const Eigen::Quaternionf qfinal(eig_dx);
        const Eigen::Vector3f tfinal = eig_dx * mean_diag + centroid.head(3);

        //set object shape
        shape_msgs::SolidPrimitive boundingVolume;
        boundingVolume.type = shape_msgs::SolidPrimitive::BOX;
        boundingVolume.dimensions.resize(3);
        boundingVolume.dimensions[shape_msgs::SolidPrimitive::BOX_X] = max_pt.x - min_pt.x;
        boundingVolume.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = max_pt.y - min_pt.y;
        boundingVolume.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_pt.z - min_pt.z;
        collisionObjects[i].primitives.push_back(boundingVolume);
        geometry_msgs::Pose pose;
        pose.position.x = tfinal[0];
        pose.position.y = tfinal[1];
        pose.position.z = tfinal[2];
        pose.orientation.w = qfinal.w();
        pose.orientation.x = qfinal.x();
        pose.orientation.y = qfinal.y();
        pose.orientation.z = qfinal.z();
        collisionObjects[i].primitive_poses.push_back(pose);
        collisionObjects[i].operation = moveit_msgs::CollisionObject::ADD;
      }

      planningSceneInterface->addCollisionObjects(collisionObjects);
    }
  }
}

bool NimbusMoveIt::attachClosestSceneObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);  //lock for the stored objects array

  //find the closest point to the gripper pose
  int closest = 0;
  if (objectList.objects.size() == 0)
  {
    ROS_INFO("No scene objects to attach.");
    return true;
  } else if (objectList.objects.size() > 1)
  {
    // find the closest point
    float min = numeric_limits<float>::infinity();
    //geometry_msgs::Vector3 &v = grasp.transform.translation;
    // check each segmented object
    for (size_t i = 0; i < objectList.objects.size(); i++)
    {
      geometry_msgs::TransformStamped eef_transform = tfBuffer.lookupTransform(
              objectList.objects[i].point_cloud.header.frame_id, jacoArmGroup->getEndEffectorLink(), ros::Time(0)
      );
      geometry_msgs::Vector3 &v = eef_transform.transform.translation;
      //convert PointCloud2 to PointCloud to access the data easily
      sensor_msgs::PointCloud cloud;
      sensor_msgs::convertPointCloud2ToPointCloud(objectList.objects[i].point_cloud, cloud);
      // check each point in the cloud
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        // euclidean distance to the point
        float dist = sqrt(
                pow(cloud.points[j].x - v.x, 2) + pow(cloud.points[j].y - v.y, 2) + pow(cloud.points[j].z - v.z, 2)
        );
        if (dist < min)
        {
          min = dist;
          closest = i;
        }
      }
    }

    if (min > SCENE_OBJECT_DST_THRESHOLD)
    {
      ROS_INFO("No scene objects are close enough to the end effector to be attached.");
      return true;
    }
  }

  rail_manipulation_msgs::GraspingState graspingState;
  graspingState.object_in_gripper = true;
  graspingState.object_name = objectList.objects[closest].name;
  graspingStatePublisher.publish(graspingState);

  vector<string> touchLinks;
  touchLinks.push_back("j2s7s300_ee_link");
  touchLinks.push_back("nimbus_ee_link");
  touchLinks.push_back("j2s7s300_base_link");
  touchLinks.push_back("j2s7s300_left_finger_link");
  touchLinks.push_back("j2s7s300_left_finger_tip_link");
  touchLinks.push_back("j2s7s300_left_inner_knuckle_link");
  touchLinks.push_back("j2s7s300_left_knuckle_link");
  touchLinks.push_back("j2s7s300_right_finger_link");
  touchLinks.push_back("j2s7s300_right_finger_tip_link");
  touchLinks.push_back("j2s7s300_right_inner_knuckle_link");
  touchLinks.push_back("j2s7s300_right_knuckle_link");
  jacoArmGroup->attachObject(unattachedObjects[closest], jacoArmGroup->getEndEffectorLink(), touchLinks);
  attachedObjects.push_back(unattachedObjects[closest]);
  unattachedObjects.erase(unattachedObjects.begin() + closest);

  return true;
}

bool NimbusMoveIt::detachSceneObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for (int i = 0; i < attachedObjects.size(); i ++)
  {
    jacoArmGroup->detachObject(attachedObjects[i]);
  }
  planningSceneInterface->removeCollisionObjects(attachedObjects);
  attachedObjects.clear();

  rail_manipulation_msgs::GraspingState graspingState;
  graspingState.object_in_gripper = false;
  graspingStatePublisher.publish(graspingState);

  return true;
}

bool NimbusMoveIt::prepareGrasp(rail_manipulation_msgs::PrepareGrasp::Request &req, rail_manipulation_msgs::PrepareGrasp::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);  //lock for the stored objects array

  //find the closest point to the gripper pose
  int closest = 0;
  if (objectList.objects.size() == 0)
  {
    ROS_INFO("No scene objects to attach.");
    res.success = true;
    return true;
  } else if (objectList.objects.size() > 1)
  {
    // find the closest point
    float min = numeric_limits<float>::infinity();
    //geometry_msgs::Vector3 &v = grasp.transform.translation;
    // check each segmented object
    for (size_t i = 0; i < objectList.objects.size(); i++)
    {
      geometry_msgs::PoseStamped transformedPose;
      if (objectList.objects[i].point_cloud.header.frame_id != req.graspPose.header.frame_id)
      {
        transformedPose.header.frame_id = objectList.objects[i].point_cloud.header.frame_id;
        tf.transformPose(objectList.objects[i].point_cloud.header.frame_id, req.graspPose, transformedPose);
      }
      else
      {
        transformedPose = req.graspPose;
      }
      //convert PointCloud2 to PointCloud to access the data easily
      sensor_msgs::PointCloud cloud;
      sensor_msgs::convertPointCloud2ToPointCloud(objectList.objects[i].point_cloud, cloud);
      // check each point in the cloud
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        // euclidean distance to the point
        float dist = sqrt(
                pow(cloud.points[j].x - transformedPose.pose.position.x, 2) + pow(cloud.points[j].y - transformedPose.pose.position.y, 2) + pow(cloud.points[j].z - transformedPose.pose.position.z, 2)
        );
        if (dist < min)
        {
          min = dist;
          closest = i;
        }
      }
    }
  }

  moveit_msgs::GetPlanningScene planningSceneSrv;
  planningSceneSrv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  if (!planningSceneClient.call(planningSceneSrv))
  {
    ROS_INFO("Could not get the current planning scene.");
    res.success = false;
    return true;
  }

  collision_detection::AllowedCollisionMatrix acm(planningSceneSrv.response.scene.allowed_collision_matrix);
  acm.setEntry(unattachedObjects[closest], gripperNames, true);
  //moveit_msgs::PlanningScene planningSceneUpdate;
  //acm.getMessage(planningSceneUpdate.allowed_collision_matrix);
  //planningSceneUpdate.is_diff = true;
  //planningScenePublisher.publish(planningSceneUpdate);

  //Disable collisions on self-filter geometry
  //acm.setEntry("<octomap>", simplifiedGeometryNames, true);

  moveit_msgs::PlanningScene planningSceneUpdate;
  acm.getMessage(planningSceneUpdate.allowed_collision_matrix);
  planningSceneUpdate.is_diff = true;
  planningScenePublisher.publish(planningSceneUpdate);

  ros::Duration(0.5).sleep(); //delay for publish to go through

  ignoredObject = unattachedObjects[closest];
  ROS_INFO("Ignoring collisions for %s", unattachedObjects[closest].c_str());

  res.success = true;
  return true;
}

bool NimbusMoveIt::reactivateGraspCollision(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (ignoredObject != "")
  {
    moveit_msgs::GetPlanningScene planningSceneSrv;
    planningSceneSrv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    if (!planningSceneClient.call(planningSceneSrv))
    {
      ROS_INFO("Could not get the current planning scene.");
      return false;
    }

    collision_detection::AllowedCollisionMatrix acm(planningSceneSrv.response.scene.allowed_collision_matrix);
    acm.setEntry(ignoredObject, gripperNames, false);

    moveit_msgs::PlanningScene planningSceneUpdate;
    acm.getMessage(planningSceneUpdate.allowed_collision_matrix);
    planningSceneUpdate.is_diff = true;
    planningScenePublisher.publish(planningSceneUpdate);

    ros::Duration(0.5).sleep(); //delay for publish to go through
    ignoredObject = "";
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_moveit");

  NimbusMoveIt nmi;

  ros::spin();
}

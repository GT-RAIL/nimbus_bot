#include <nimbus_world_model/world_model.h>

using namespace std;

WorldModel::WorldModel() : pn("~"), rng(time(NULL))
{
  srand(time(NULL));

  pn.param("allow_duplicate_objects", allowDuplicateObjects, false);
  pn.param("num_particles_per_object", particlesPerObject, 50);

  string recognizedObjectsTopic("/object_recognition_listener/recognized_objects");
  pn.getParam("recognized_objects_topic", recognizedObjectsTopic);

  recognizedObjectsSubscriber = n.subscribe(recognizedObjectsTopic, 1, &WorldModel::measurementUpdate, this);
  actionFeedbackSubscriber = n.subscribe("nimbus_action_queue/execute/feedback", 1, &WorldModel::actionFeedbackCallback, this);
  actionResultSubscriber = n.subscribe("nimbus_action_queue/execute/result", 1, &WorldModel::actionResultCallback, this);

  worldModelPublisher = pn.advertise<nimbus_world_model::WorldModel>("world_model", 1, true);
  debugPublisher = pn.advertise<geometry_msgs::PoseArray>("debug_poses", 1, true);
  debugApplePosePublisher = pn.advertise<geometry_msgs::PoseStamped>("apple_pose", 1, true);
  debugOrangePosePublisher = pn.advertise<geometry_msgs::PoseStamped>("orange_pose", 1, true);
}

void WorldModel::timePredictionUpdate()
{
  boost::recursive_mutex::scoped_lock lock(model_update_mutex);
  for (int i = (int)worldObjects.size() - 1; i >= 0; i --)
  {
    //add random noise to object poses
    if (!worldObjects[i].empty())
    {
      for (int j = worldObjects[i].size() - 1; j >= 0; j --)
      {
        if (!worldObjects.at(i).at(j).inGripper)
        {
          double dst = rng.gaussian(TIME_UPDATE_SIGMA);
          double dir = (double)rand()/(double)RAND_MAX * 2 * M_PI;
          worldObjects.at(i).at(j).pose.position.x += dst*cos(dir);
          worldObjects.at(i).at(j).pose.position.y += dst*sin(dir);

          //decay confidence and remove any particle below the minimum threshold (object semi-permanence...)
          worldObjects.at(i).at(j).confidence *= TIME_DECAY;
          //worldObjects.at(i).at(j).lastUpdated = ros::Time::now();
          if (worldObjects.at(i).at(j).confidence < MINIMUM_CONFIDENCE)
            worldObjects.at(i).erase(worldObjects.at(i).begin() + j);
        }
      }
    }

    if (worldObjects[i].empty())
    {
      worldObjects.erase(worldObjects.begin() + i);
    }
  }

  publishWorldModel();
}

void WorldModel::actionFeedbackCallback(const rail_action_queue_msgs::ExecuteActionFeedback::ConstPtr &actionFeedback)
{
  motionPredictionUpdate(actionFeedback->feedback.last_completed_action, rail_action_queue_msgs::GeneralAction::SUCCESS);
}

void WorldModel::actionResultCallback(const rail_action_queue_msgs::ExecuteActionResult::ConstPtr &actionResult)
{
  motionPredictionUpdate(actionResult->result.last_action, actionResult->result.error);
}

void WorldModel::motionPredictionUpdate(rail_action_queue_msgs::GeneralAction action, unsigned char errorCode)
{
  switch (action.action_type)
  {
    case rail_action_queue_msgs::GeneralAction::PICKUP:
      if (errorCode == rail_action_queue_msgs::GeneralAction::SUCCESS)
      {
        bool objectFound = false;
        for (unsigned int i = 0; i < worldObjects.size(); i ++)
        {
          if (!worldObjects.at(i).empty() && worldObjects.at(i).at(0).name == action.object_name)
          {
            //successful pickup: remove all particles, add one particle in gripper
            worldObjects.at(i).at(0).inGripper = true;
            worldObjects.at(i).at(0).confidence = 1.0;
            worldObjects.at(i).at(0).lastUpdated = ros::Time::now();
            if (worldObjects.at(i).size() > 1)
              worldObjects.at(i).erase(worldObjects.at(i).begin() + 1, worldObjects.at(i).end());

            objectFound = true;
            break;
          }
        }
        if (!objectFound)
        {
          //initialize new filter for picked-up object
          vector<WorldObjectParticle> newObjectFilter;
          WorldObjectParticle pickupParticle;
          pickupParticle.inGripper = true;
          pickupParticle.confidence = 1.0;
          pickupParticle.lastUpdated = ros::Time::now();
          newObjectFilter.push_back(pickupParticle);
          worldObjects.push_back(newObjectFilter);
        }
      }
      else if (errorCode == rail_action_queue_msgs::GeneralAction::MANIPULATION_EXECUTION_FAILURE)
      {
        for (unsigned int i = 0; i < worldObjects.size(); i ++)
        {
          if (!worldObjects.at(i).empty() && worldObjects.at(i).at(0).name == action.object_name)
          {
            //failed pickup: add pickup failure noise to all particles
            for (unsigned int j = 0; j < worldObjects.at(i).size(); j ++)
            {
              double dst = rng.gaussian(PICKUP_FAILURE_SIGMA);
              double dir = (double)rand()/(double)RAND_MAX * 2 * M_PI;
              worldObjects.at(i).at(j).pose.position.x += dst*cos(dir);
              worldObjects.at(i).at(j).pose.position.y += dst*sin(dir);
              worldObjects.at(i).at(j).lastUpdated = ros::Time::now();
            }
            break;
          }
        }
      }
    break;
    case rail_action_queue_msgs::GeneralAction::STORE:
      if (errorCode == rail_action_queue_msgs::GeneralAction::SUCCESS)
      {
        for (unsigned int i = 0; i < worldObjects.size(); i ++)
        {
          if (!worldObjects.at(i).empty() && worldObjects.at(i).at(0).name == action.object_name)
          {
            //successful place: remove all particles, initialize a full set of particles around store location
            worldObjects.at(i).clear();
            WorldObjectParticle newParticle;
            newParticle.confidence = 1.0 / (double)particlesPerObject;
            newParticle.inGripper = false;
            newParticle.name = action.object_name;
            newParticle.pose = action.manipulation_pose.pose;
            for (unsigned int j = 0; j < particlesPerObject; j ++)
            {
              worldObjects.at(i).push_back(newParticle.copy());
              double dst = rng.gaussian(PICKUP_FAILURE_SIGMA);
              double dir = (double)rand()/(double)RAND_MAX * 2 * M_PI;
              worldObjects.at(i).at(worldObjects.at(i).size() - 1).pose.position.x += dst*cos(dir);
              worldObjects.at(i).at(worldObjects.at(i).size() - 1).pose.position.y += dst*sin(dir);
              worldObjects.at(i).at(worldObjects.at(i).size() - 1).lastUpdated = ros::Time::now();
            }
            break;
          }
        }
      }
    break;
    default:
      return; //no update for unlisted action types
  }
}

void WorldModel::measurementUpdate(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objectList)
{

  boost::recursive_mutex::scoped_lock lock(model_update_mutex);
  if (objectList->cleared || objectList->objects.empty())
  {
    ROS_INFO("Object list cleared or empty, skipping update");
    return;
  }

  vector<int> updatedObjects; //track what needs to be normalized and resampled
  vector<int> measurementsUsed;

  for (unsigned int i = 0; i < worldObjects.size(); i ++)
  {
    if (worldObjects[i].empty())
      continue;

    vector<int> measurements;
    for (unsigned int j = 0; j < objectList->objects.size(); j++)
    {
      if (objectList->objects[j].recognized && worldObjects.at(i).at(0).name == objectList->objects[j].name)
      {
        measurements.push_back(j);
        measurementsUsed.push_back(j);
      }
    }

    if (!measurements.empty())  //update particles
    {
      updatedObjects.push_back(i);

      vector<double> measurementMinDsts;
      measurementMinDsts.resize(measurements.size());
      for (unsigned int j = 0; j < measurementMinDsts.size(); j ++)
      {
        measurementMinDsts[j] = numeric_limits<double>::max();
      }

      //update particle weights
      for (unsigned int j = 0; j < worldObjects.at(i).size(); j++)
      {
        if (!worldObjects.at(i).at(j).inGripper)
        {
          int closestMeasurement = 0;
          double closestDst = numeric_limits<double>::max();
          for (unsigned int k = 0; k < measurements.size(); k ++)
          {
            double dst = pow(objectList->objects[measurements[k]].center.x - worldObjects.at(i).at(j).pose.position.x, 2)
                              + pow(objectList->objects[measurements[k]].center.y - worldObjects.at(i).at(j).pose.position.y, 2);
            if (dst < closestDst)
            {
              closestDst = dst;
              closestMeasurement = k;
            }
            if (dst < measurementMinDsts[k])
            {
              measurementMinDsts[k] = dst;
            }
          }
          worldObjects.at(i).at(j).confidence *=
              gaussian(objectList->objects[measurements[closestMeasurement]].center.x, worldObjects.at(i).at(j).pose.position.x, MEASUREMENT_SIGMA) *
              gaussian(objectList->objects[measurements[closestMeasurement]].center.y, worldObjects.at(i).at(j).pose.position.y, MEASUREMENT_SIGMA);
        }
      }

      //add some new particles if the measurement is too far away
      for (unsigned int j = 0; j < measurements.size(); j ++)
      {
        if (measurementMinDsts[j] > NEW_OBJECT_MEASUREMENT_DST)
        {
          int particlesToAdd = (int) ceil(((double) particlesPerObject) / 4.0);

          if (worldObjects[i].size() + particlesToAdd > particlesPerObject)
          {
            //remove low-confidence particles
            int particlesToRemove = particlesToAdd + worldObjects[i].size() - particlesPerObject;
            sort(worldObjects[i].begin(), worldObjects[i].end(), WorldObjectParticle::worldObjectComparator);
            for (int j = 0; j < particlesToRemove; j++)
            {
              worldObjects[i].erase(worldObjects[i].end() - 1);
            }
          }

          //add new particles
          WorldObjectParticle newParticle;
          newParticle.name = objectList->objects[measurements[j]].name;
          newParticle.confidence = 1.0 / (double) particlesPerObject * pow(gaussian(0, 0, MEASUREMENT_SIGMA), 2);
          newParticle.pose.position.x = objectList->objects[measurements[j]].center.x;
          newParticle.pose.position.y = objectList->objects[measurements[j]].center.y;
          newParticle.pose.position.z = objectList->objects[measurements[j]].center.z;
          for (unsigned int k = 0; k < particlesToAdd; k++)
          {
            worldObjects[i].push_back(newParticle.copy());
          }
        }
      }
    }
  }

  //add new filters for any new objects
  sort(measurementsUsed.begin(), measurementsUsed.end());
  measurementsUsed.erase(unique(measurementsUsed.begin(), measurementsUsed.end()), measurementsUsed.end());
  for (unsigned int i = 0; i < objectList->objects.size(); i ++)
  {
    bool used = false;
    for (unsigned int j = 0; j < measurementsUsed.size(); j ++)
    {
      if (i == measurementsUsed[j])
      {
        used = true;
        break;
      }
    }

    if (!used)
    {
      vector<WorldObjectParticle> newParticleTemplates;
      string objectName = objectList->objects[i].name;
      for (unsigned int j = i; j < objectList->objects.size(); j ++)
      {
        if (objectList->objects[j].name == objectName)
        {
          WorldObjectParticle newParticle;
          newParticle.name = objectList->objects[j].name;
          newParticle.confidence = 1.0 / (double) particlesPerObject;
          newParticle.pose.position.x = objectList->objects[j].center.x;
          newParticle.pose.position.y = objectList->objects[j].center.y;
          newParticle.pose.position.z = objectList->objects[j].center.z;
          newParticleTemplates.push_back(newParticle);
          if (j > i)
            measurementsUsed.push_back(j);
        }
      }
      vector<WorldObjectParticle> newObjectParticles;
      for (unsigned int j = 0; j < newParticleTemplates.size(); j ++)
      {
        for (unsigned int k = 0; k < particlesPerObject / newParticleTemplates.size(); k ++)
        {
          newObjectParticles.push_back(newParticleTemplates[j].copy());
        }
      }
      worldObjects.push_back(newObjectParticles);
    }
  }

  //normalize, calculate ESS, resample
  if (updatedObjects.size() > 1)
  {
    sort(updatedObjects.begin(), updatedObjects.end());
    updatedObjects.erase(unique(updatedObjects.begin(), updatedObjects.end()), updatedObjects.end());
  }
  for (unsigned int i = 0; i < updatedObjects.size(); i++)
  {
    double sum = 0.0;
    double ess, cv = 0;
    int missingParticles = particlesPerObject - worldObjects[updatedObjects[i]].size();

    for (unsigned int j = 0; j < worldObjects[updatedObjects[i]].size(); j++)
    {
      sum += worldObjects.at(updatedObjects[i]).at(j).confidence;
    }

    for (unsigned int j = 0; j < worldObjects[updatedObjects[i]].size(); j++)
    {
      worldObjects.at(updatedObjects[i]).at(j).confidence /= sum;
      cv += pow(particlesPerObject*worldObjects.at(updatedObjects[i]).at(j).confidence - 1, 2);
    }
    cv += missingParticles*pow(particlesPerObject*0-1,2);

    cv /= (double)particlesPerObject;
    ess = (double)particlesPerObject/(1 + cv);

    //resample if ESS is too low
    ROS_INFO("ESS after measurement update: %f", ess);
    if (ess < RESAMPLING_RATE*particlesPerObject)
    {
      ROS_INFO("Resampling for object %s", worldObjects.at(updatedObjects[i]).at(0).name.c_str());
      resample(i);
    }
  }
  ROS_INFO("DONE");

  publishWorldModel();
}

void WorldModel::resample(int index)
{
  boost::recursive_mutex::scoped_lock lock(model_update_mutex);
  vector<WorldObjectParticle> resampledParticles;
  double sum = 0.0;
  for (unsigned int i = 0; i < particlesPerObject; i ++)
  {
    double sample = (double)rand()/(double)RAND_MAX;
    double counter = 0.0;
    for (unsigned int j = 0; j < worldObjects[index].size(); j ++)
    {
      counter += worldObjects.at(index).at(j).confidence;
      if (counter >= sample)
      {
        resampledParticles.push_back(worldObjects.at(index).at(j).copy());
        //TODO: Test keeping the confidence and re-normalizing
        sum += resampledParticles.at(resampledParticles.size() - 1).confidence;
        //resampledParticles.at(resampledParticles.size() - 1).confidence = 1.0/(double)particlesPerObject;
        break;
      }
    }
  }
  //TODO: Part of above change to test
  for (unsigned int i = 0; i < resampledParticles.size(); i ++)
  {
    resampledParticles.at(i).confidence /= sum;
  }

  worldObjects.at(index) = resampledParticles;
}

bool WorldModel::worldObjectComparator(const nimbus_world_model::WorldObject &object1, const nimbus_world_model::WorldObject &object2)
{
  return object1.confidence > object2.confidence;
}

void WorldModel::publishWorldModel()
{
  boost::recursive_mutex::scoped_lock lock(model_update_mutex);
  nimbus_world_model::WorldModel model;
  model.header.stamp = ros::Time::now();

  for (unsigned int i = 0; i < worldObjects.size(); i ++)
  {
    if (!worldObjects[i].empty())
    {
      sort(worldObjects[i].begin(), worldObjects[i].end(), WorldObjectParticle::worldObjectComparator);
      WorldObjectParticle worldObject = worldObjects.at(i).at(0).copy();

      //special case: object in gripper
      if (worldObjects.at(i).at(0).inGripper)
      {
        //TODO: what to set for the object pose...
      }
      //special case: all particles have equal weight (occurs after resampling)
      else if (worldObjects.at(i).at(0).confidence == worldObjects.at(i).at(worldObjects.size() - 1).confidence)
      {
        //compute average position
        double x = 0.0, y = 0.0, z = 0.0;
        for (unsigned int j = 0; j < worldObjects[i].size(); j ++)
        {
          x = (j*x + worldObjects.at(i).at(j).pose.position.x) / (double)(j + 1);
          y = (j*y + worldObjects.at(i).at(j).pose.position.y) / (double)(j + 1);
          z = (j*z + worldObjects.at(i).at(j).pose.position.z) / (double)(j + 1);
        }
        worldObject.pose.position.x = x;
        worldObject.pose.position.y = y;
        worldObject.pose.position.z = z;
      }
      else
      {
        //compute locally weighted average
        double totalWeight = 0.0;
        double x = 0.0, y = 0.0, z = 0.0;
        for (unsigned int j = 0; j < worldObjects[i].size(); j ++)
        {
          if (pow(worldObjects.at(i).at(0).pose.position.x - worldObjects.at(i).at(j).pose.position.x, 2) +
                   pow(worldObjects.at(i).at(0).pose.position.y - worldObjects.at(i).at(j).pose.position.y, 2) +
                   pow(worldObjects.at(i).at(0).pose.position.z - worldObjects.at(i).at(j).pose.position.z, 2) < NEIGHBORHOOD_RADIUS)
          {
            x += worldObjects.at(i).at(j).pose.position.x*worldObjects.at(i).at(j).confidence;
            y += worldObjects.at(i).at(j).pose.position.y*worldObjects.at(i).at(j).confidence;
            z += worldObjects.at(i).at(j).pose.position.z*worldObjects.at(i).at(j).confidence;
            totalWeight += worldObjects.at(i).at(j).confidence;
          }
        }
        x /= totalWeight;
        y /= totalWeight;
        z /= totalWeight;
        worldObject.pose.position.x = x;
        worldObject.pose.position.y = y;
        worldObject.pose.position.z = z;
      }

      model.objects.push_back(worldObject.toRosMsg());
    }
  }

  //Debug publishing
  if (!worldObjects.empty())
  {
    //boost::recursive_mutex::scoped_lock lock(model_update_mutex);
    geometry_msgs::PoseArray poses;
    double maxConfidence, minConfidence, avgConfidence = 0.0;
    int missingParticles = particlesPerObject - worldObjects[0].size();
    for (unsigned int i = 0; i < worldObjects[0].size(); i ++)
    {
      poses.header.stamp = ros::Time::now();
      poses.header.frame_id = "table_base_link";
      poses.poses.push_back(worldObjects.at(0).at(i).pose);
      avgConfidence = (avgConfidence*i + worldObjects.at(0).at(i).confidence) / (double)(i + 1);
    }
    maxConfidence = worldObjects.at(0).at(0).confidence;
    minConfidence = worldObjects.at(0).at(worldObjects[0].size() - 1).confidence;
    ROS_INFO("\n\n--------------------------------------------------");
    ROS_INFO("For object %s:", worldObjects.at(0).at(0).name.c_str());
    ROS_INFO("max confidence: %f, min confidence: %f, average confidence: %f", maxConfidence, minConfidence, avgConfidence);
    debugPublisher.publish(poses);

    for (unsigned int i = 0; i < model.objects.size(); i ++)
    {
      if (model.objects[i].name == "PEACH")
      {
        geometry_msgs::PoseStamped applePose;
        applePose.header.stamp = ros::Time::now();
        applePose.header.frame_id = "table_base_link";
        applePose.pose = model.objects[i].pose;
        debugApplePosePublisher.publish(applePose);
      }
      else if (model.objects[i].name == "ORANGE")
      {
        geometry_msgs::PoseStamped orangePose;
        orangePose.header.stamp = ros::Time::now();
        orangePose.header.frame_id = "table_base_link";
        orangePose.pose = model.objects[i].pose;
        debugOrangePosePublisher.publish(orangePose);
      }
    }
  }
  else
  {
    ROS_INFO("No particles!");
  }

  worldModelPublisher.publish(model);
}

double WorldModel::gaussian(double x, double mu, double sigma)
{
  return 1/(sqrt(2*M_PI)*sigma)*pow(M_E,-pow(x-mu,2)/(2*pow(sigma,2)));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "world_model");

  WorldModel wm;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    wm.timePredictionUpdate();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

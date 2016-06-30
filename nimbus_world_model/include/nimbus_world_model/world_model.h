#ifndef WORLD_MODEL_H_
#define WORLD_MODEL_H_

#include <ros/ros.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/core/core.hpp>
#include <rail_action_queue_msgs/ExecuteAction.h>
#include <rail_manipulation_msgs/GraspingState.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <nimbus_world_model/WorldModel.h>
#include <nimbus_world_model/WorldObjectParticle.h>

class WorldModel
{
public:
  /**
   * \brief Constructor
   */
  WorldModel();

  void timePredictionUpdate();

private:
  bool worldObjectComparator(const nimbus_world_model::WorldObject &object1, const nimbus_world_model::WorldObject &object2);

  void actionFeedbackCallback(const rail_action_queue_msgs::ExecuteActionFeedback::ConstPtr &actionFeedback);

  void actionResultCallback(const rail_action_queue_msgs::ExecuteActionResult::ConstPtr &actionResult);

  void motionPredictionUpdate(rail_action_queue_msgs::GeneralAction action, unsigned char errorCode);

  void measurementUpdate(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objectList);

  void resample(int index);

  void publishWorldModel();

  static double gaussian(double x, double mu, double sigma);

  const static double TIME_UPDATE_SIGMA = 0.004;
  const static double TIME_DECAY = .99;
  const static double MINIMUM_CONFIDENCE = 0.003;

  const static double PICKUP_FAILURE_SIGMA = 0.025;
  const static double PLACE_SIGMA = 0.025;

  const static double MEASUREMENT_SIGMA = 0.05; //sensor measurement error
  //const static double MEASUREMENT_SIGMA = 0.3; //sensor accounting for false positives
  const static double NEW_OBJECT_MEASUREMENT_DST = 0.0225; //sqaured radius to use for a measurement to be considered a new instance of the object (.15 m)

  const static double RESAMPLING_RATE = 0.6;  //resample when ESS falls below RESAMPLING_RATE*particlesPerObject

  const static double NEIGHBORHOOD_RADIUS = .0025; //squared radius to use for locally weighted average for final object pose (.05 m)

  ros::NodeHandle n, pn;

  ros::Subscriber recognizedObjectsSubscriber;
  ros::Subscriber actionFeedbackSubscriber;
  ros::Subscriber actionResultSubscriber;
  ros::Publisher worldModelPublisher;
  ros::Publisher debugPublisher;
  ros::Publisher debugApplePosePublisher;
  ros::Publisher debugOrangePosePublisher;

  boost::recursive_mutex model_update_mutex;

  cv::RNG rng;

  bool allowDuplicateObjects;
  int particlesPerObject;
  std::vector< std::vector<WorldObjectParticle> > worldObjects;
  nimbus_world_model::WorldModel model;
};

#endif

//
// Created by davidkent on 2/12/16.
//

#ifndef WORLD_OBJECT_PARTICLE_H
#define WORLD_OBJECT_PARTICLE_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <nimbus_world_model/WorldObject.h>

class WorldObjectParticle {
public:
  geometry_msgs::Pose pose;
  std::string name;
  double confidence;
  bool inGripper;
  std::string onObject;
  std::string inObject;
  bool isStatic;
  ros::Time lastUpdated;

  WorldObjectParticle();

  WorldObjectParticle(const nimbus_world_model::WorldObject &worldObject);

  WorldObjectParticle copy();

  nimbus_world_model::WorldObject toRosMsg();

  void fromRosMsg(const nimbus_world_model::WorldObject &worldObject);

  static bool worldObjectComparator(const WorldObjectParticle &object1, const WorldObjectParticle &object2);
};


#endif //PROJECT_WORLDOBJECT_H

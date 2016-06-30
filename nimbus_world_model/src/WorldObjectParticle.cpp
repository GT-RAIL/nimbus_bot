//
// Created by davidkent on 2/12/16.
//

#include <nimbus_world_model/WorldObjectParticle.h>

WorldObjectParticle::WorldObjectParticle()
{
  name = "";
  confidence = 0.0;
  inGripper = false;
  onObject = "";
  inObject = "";
  isStatic = false;
  lastUpdated = ros::Time::now();
  pose.orientation.w = 1.0;
}

WorldObjectParticle::WorldObjectParticle(const nimbus_world_model::WorldObject &worldObject)
{
  this->fromRosMsg(worldObject);
}

WorldObjectParticle WorldObjectParticle::copy()
{
  WorldObjectParticle particle;
  particle.name = this->name;
  particle.confidence = this->confidence;
  particle.inGripper = this->inGripper;
  particle.onObject = this->onObject;
  particle.inObject = this->inObject;
  particle.isStatic = this->isStatic;
  particle.lastUpdated = this->lastUpdated;
  particle.pose = this->pose;

  return particle;
}

nimbus_world_model::WorldObject WorldObjectParticle::toRosMsg()
{
  nimbus_world_model::WorldObject worldObject;
  worldObject.name = this->name;
  worldObject.confidence = this->confidence;
  worldObject.inGripper = this->inGripper;
  worldObject.onObject = this->onObject;
  worldObject.inObject = this->inObject;
  worldObject.isStatic = this->isStatic;
  worldObject.lastUpdated = this->lastUpdated;
  worldObject.pose = this->pose;

  return worldObject;
}

void WorldObjectParticle::fromRosMsg(const nimbus_world_model::WorldObject &worldObject)
{
  this->name = worldObject.name;
  this->confidence = worldObject.confidence;
  this->inGripper = worldObject.inGripper;
  this->onObject = worldObject.onObject;
  this->inObject = worldObject.inObject;
  this->isStatic = worldObject.isStatic;
  this->lastUpdated = worldObject.lastUpdated;
  this->pose = worldObject.pose;
}

bool WorldObjectParticle::worldObjectComparator(const WorldObjectParticle &object1, const WorldObjectParticle &object2)
{
  return object1.confidence > object2.confidence;
}

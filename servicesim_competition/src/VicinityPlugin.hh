#ifndef GAZEBO_PLUGINS_VICINITY_PLUGIN_HH
#define GAZEBO_PLUGINS_VICINITY_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros/ros.h"
#include <std_msgs/Bool.h>

namespace gazebo
{
  class VicinityPlugin: public ModelPlugin
  {

  public: VicinityPlugin();
  public: virtual ~VicinityPlugin();
  public: void VicinityPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  public: void Update();

  //Store pointer to the model
  private: physics::ModelPtr model;
  //Store pointer to the world
  private: physics::WorldPtr world;
  //Store pointer to the entity
  private: physics::EntityPtr entity;
  private: event::ConnectionPtr updateConnection;
  private: std::string paramName;
  private: std::double threshold;

  private: std::double distance;
  // private: ignition:math:Pose3d robot_pos, entity_pos;
  /// ROS stuff
  private: ros::NodeHandle nh;
  private: ros::Publisher pub;
  private: std_msgs::Bool flag;


  }
}

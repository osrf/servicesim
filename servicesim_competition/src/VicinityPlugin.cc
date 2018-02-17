#include "VicinityPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(VicinityPlugin);

////////////////////////
// Constructor
VicinityPlugin::VicinityPlugin():ModelPlugin()
{

}

//Destructor
VicinityPlugin::~VicinityPlugin()
{

}

void VicinityPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // store the pointer to the model_name
  this->model = _parent;
  this->world = _parent->GetWorld(); // Store the pointer to the world

  std::string model_name = this->model->GetName();
  std::string topic_name = "vicinity";
  pub = nh.advertise<std_msgs::Bool>(topic_name,1);
  paramName = "guest_name";
  if (_sdf->HasElement(paramName))
  {
    this->entity = world->GetEntity(_sdf->Get<std::string>(paramName));
  }
  paramName = "threshold";
  if (_sdf->HasElement(paramName))
  {
    this->threshold = _sdf->Get<double>(paramName);
  }

  //Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&VicinityPlugin::Update, this));
}

void VicinityPlugin::Update()
{
  auto robot_pos = this->model->WorldPose().Pos();
  auto entity_pos = this->entity->WordlPose().Pos();


}

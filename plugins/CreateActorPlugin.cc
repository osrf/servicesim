/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <gazebo/common/ModelDatabase.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>
#include "CreateActorPlugin.hh"

std::map<std::string, std::string> skinMap;
std::map<std::string, std::string> animIdleMap;
std::map<std::string, std::string> animTrajectoryMap;
std::map<std::string, ignition::math::Pose3d> animPoseMap;
unsigned int actorCount{0};

namespace gazebo
{
  /// \brief Private data for the CreateActorPlugin class
  class CreateActorPluginPrivate
  {
    /// \brief Pointer to a node for communication.
    public: transport::NodePtr gzNode;

    /// \brief keyboard publisher.
    public: transport::PublisherPtr factoryPub;

    /// \brief Latest SDF
    public: std::string currentSDF{""};
  };
}

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CreateActorPlugin)

/////////////////////////////////////////////////
// Initiate the insertion of a ghost model
void insertGhost()
{
  auto filename = common::ModelDatabase::Instance()->GetModelFile(
      "model://ghost");
  gui::Events::createEntity("model", filename);
}

/////////////////////////////////////////////////
CreateActorPlugin::CreateActorPlugin()
  : GUIPlugin(), dataPtr(new CreateActorPluginPrivate)
{
  // Maps
  skinMap["Green shirt"] = "SKIN_man_green_shirt";
  skinMap["Red shirt"] = "SKIN_man_red_shirt";
  skinMap["Blue shirt"] = "SKIN_man_blue_shirt";

  animIdleMap["Talking A"] = "ANIMATION_talking_a";
  animPoseMap["Talking A"] = ignition::math::Pose3d(1, 0, -1.25, 0, 0, -IGN_PI_2);

  animIdleMap["Talking B"] = "ANIMATION_talking_b";
  animPoseMap["Talking B"] = ignition::math::Pose3d(1, 0, -1.25, 0, 0, IGN_PI_2);

  animTrajectoryMap["Walking"] = "ANIMATION_walking";
  animPoseMap["Walking"] = ignition::math::Pose3d(0, -1, -1.4, 0, 0, IGN_PI);

  animTrajectoryMap["Running"] = "ANIMATION_running";
  animPoseMap["Running"] = ignition::math::Pose3d(0, -1, -1.4, 0, 0, IGN_PI);

  // Stacked layout
  auto mainLayout = new QStackedLayout();

  // Frame
  auto frame = new QFrame();
  frame->setLayout(mainLayout);
  auto frameLayout = new QVBoxLayout();
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frameLayout->addWidget(frame);
  this->setLayout(frameLayout);

  // Transport
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();
  this->dataPtr->factoryPub =
      this->dataPtr->gzNode->Advertise<msgs::Factory>("~/factory");

  // Skin combo
  auto skinCombo = new QComboBox();
  skinCombo->setObjectName("skinCombo");
  for (auto s : skinMap)
    skinCombo->addItem(QString::fromStdString(s.first));

  // Animation combo
  auto animCombo = new QComboBox();
  animCombo->setObjectName("animCombo");
  for (auto a : animIdleMap)
    animCombo->addItem(QString::fromStdString(a.first));
  for (auto a : animTrajectoryMap)
    animCombo->addItem(QString::fromStdString(a.first));

  // 0: Skin and animation
  {
    // Label
    auto label = new QLabel(tr("Choose skin and animation"));
    label->setMaximumHeight(50);

    // Next button
    auto nextButton = new QPushButton(tr("Next"));
    this->connect(nextButton, &QPushButton::clicked, [=]()
    {
      insertGhost();

      mainLayout->setCurrentIndex(1);
      actorCount++;
    });

    // Layout
    auto layout = new QGridLayout;
    layout->setSpacing(0);
    layout->addWidget(label, 0, 0, 1, 2);
    layout->addWidget(new QLabel("Skin"), 1, 0);
    layout->addWidget(skinCombo, 1, 1);
    layout->addWidget(new QLabel("Animation"), 2, 0);
    layout->addWidget(animCombo, 2, 1);
    layout->addWidget(nextButton, 3, 1);

    // Widget
    auto widget = new QWidget();
    widget->setLayout(layout);

    mainLayout->addWidget(widget);
  }

  // 1: Pose(s)
  {
    // Check animation type
    std::string animType{"idle"};

    // Label
    auto label = new QLabel(
        "Position the ghost and press Next when done.<br>\
         <b>Tip</b>: Use the translation and rotation tools.<br>\
         <b>You won't be able to reposition the actor after spawned</b>");
    label->setMaximumHeight(50);

    // Add button
    // TODO: hide for idle
    auto addButton = new QPushButton(tr("New waypoint"));
    this->connect(addButton, &QPushButton::clicked, [=]()
    {
      insertGhost();
    });

    // Next button
    auto nextButton = new QPushButton(tr("Next"));
    this->connect(nextButton, &QPushButton::clicked, [=]()
    {
      this->Spawn();
      mainLayout->setCurrentIndex(2);
    });

    // Layout
    auto layout = new QGridLayout;
    layout->setSpacing(0);
    layout->addWidget(label, 0, 0, 1, 2);
    layout->addWidget(addButton, 1, 1);
    layout->addWidget(nextButton, 3, 1);

    // Widget
    auto widget = new QWidget();
    widget->setLayout(layout);

    mainLayout->addWidget(widget);
  }

  // 2: Export
  {
    // Label
    auto label = new QLabel(
        "The actor has been spawned,<br>export to a file or start a new actor.");
    label->setMaximumHeight(50);

    // New actor
    auto newButton = new QPushButton(tr("New actor"));
    this->connect(newButton, &QPushButton::clicked, [=]()
    {
      mainLayout->setCurrentIndex(0);
    });

    // Export to SDF
    auto sdfButton = new QPushButton(tr("Export to SDF"));
    this->connect(sdfButton, &QPushButton::clicked, [=]()
    {
      // Choose destination file
      QFileDialog fileDialog(this, tr("Destination SDF file"), QDir::homePath());
      fileDialog.setFileMode(QFileDialog::AnyFile);
      fileDialog.setNameFilter("*.sdf");
      fileDialog.setAcceptMode(QFileDialog::AcceptSave);
      fileDialog.setOptions(QFileDialog::DontResolveSymlinks |
                            QFileDialog::DontUseNativeDialog);
      fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
          Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

      if (fileDialog.exec() != QDialog::Accepted)
        return;

      auto selected = fileDialog.selectedFiles();
      if (selected.empty())
        return;

      // Create dir
      {
        boost::filesystem::path path(selected[0].toStdString().substr(0,
                                     selected[0].toStdString().rfind("/")+1));

        if (!boost::filesystem::create_directories(path))
          gzerr << "Couldn't create folder [" << path << "]" << std::endl;
        else
          gzmsg << "Created folder [" << path << "]" << std::endl;
      }

      // Save model.sdf
      {
        sdf::ElementPtr sdf(new sdf::Element());
        sdf::initFile("actor.sdf", sdf);

        sdf::readString(this->dataPtr->currentSDF, sdf);

        sdf::SDFPtr modelSDF;
        modelSDF.reset(new sdf::SDF);
        modelSDF->Root(sdf);

        std::ofstream savefile;
        savefile.open(selected[0].toStdString().c_str());
        if (!savefile.is_open())
        {
          gzerr << "Couldn't open file for writing: " << selected[0].toStdString() << std::endl;
          return;
        }
        savefile << modelSDF->ToString();
        savefile.close();
        gzdbg << "Saved file to " << selected[0].toStdString() << std::endl;
      }
    });

    // Layout
    auto layout = new QGridLayout;
    layout->setSpacing(0);
    layout->addWidget(label, 0, 0, 1, 2);
    layout->addWidget(newButton, 1, 0);
    layout->addWidget(sdfButton, 1, 1);

    // Widget
    auto widget = new QWidget();
    widget->setLayout(layout);

    mainLayout->addWidget(widget);
  }

  // Make this invisible
  this->move(1, 1);
  this->resize(450, 150);

  this->setStyleSheet(
      "QFrame {background-color: rgba(100, 100, 100, 255); color: black;}");
}

/////////////////////////////////////////////////
CreateActorPlugin::~CreateActorPlugin()
{
  this->dataPtr->factoryPub.reset();
  this->dataPtr->gzNode->Fini();
}

/////////////////////////////////////////////////
void CreateActorPlugin::Spawn()
{
  // Skin
  auto skinValue = this->findChild<QComboBox *>("skinCombo")
      ->currentText().toStdString();
  auto skin = skinMap[skinValue];

  // Anim
  auto animValue = this->findChild<QComboBox *>("animCombo")
      ->currentText().toStdString();

  std::string anim;
  if (animIdleMap.find(animValue) != animIdleMap.end())
  {
    anim = animIdleMap[animValue];
  }
  else
  {
    anim = animTrajectoryMap[animValue];
  }

  // Get ghost poses and delete them
  auto poseOffset = animPoseMap[animValue];
  std::vector<std::string> poses;

  std::string ghostPrefix{"ghost"};
  std::string ghostName{ghostPrefix};

  int count{0};

  while (rendering::get_scene()->GetVisual(ghostName))
  {
    // Get visual
    auto vis = rendering::get_scene()->GetVisual(ghostName);

    // Get pose
    auto pose = vis->WorldPose();

    // Apply offset
    pose = poseOffset + pose;

    // Convert to string
    std::ostringstream poseStr;
    poseStr << pose;
    poses.push_back(poseStr.str());

    // Delete ghost
    transport::requestNoReply("CreateActor", "entity_delete",
                              ghostName);

    // Next ghost
    ghostName = ghostPrefix + "_" + std::to_string(count++);
  }

  // Idle actors need one trajectory waypoint
  std::string trajectory;
  if (poses.size() == 1)
  {
    trajectory +=
        "<script>\
           <trajectory id='0' type='animation'>\
             <waypoint>\
               <time>100</time>\
               <pose>" + poses[0] + "</pose>\
             </waypoint>\
             <waypoint>\
               <time>100</time>\
               <pose>" + poses[0] + "</pose>\
             </waypoint>\
           </trajectory>\
         </script>";
  }
  // Trajectory actors use the plugin
  else
  {
    trajectory +=
        "<plugin name='wandering_plugin' filename='libWanderingActorPlugin.so'>\
            <target_weight>1.15</target_weight>\
            <obstacle_weight>1.8</obstacle_weight>\
            <animation_factor>5.1</animation_factor>";

    for (const auto &pose : poses)
    {
      trajectory += "<target>" + pose + "</target>";
    }

    trajectory += "</plugin>";
  }

  // Name
  auto name = "actor_" + std::to_string(actorCount++);

  this->dataPtr->currentSDF =
      "<?xml version='1.0' ?>\
       <sdf version='" SDF_VERSION "'>\
         <actor name='" + name + "'>\
           <pose>" + poses[0] + "</pose>\
           <skin>\
             <filename>model://actor/meshes/" + skin + ".dae</filename>\
           </skin>\
           <animation name='animation'>\
             <filename>model://actor/meshes/" + anim + ".dae</filename>";

   if (poses.size() > 1)
     this->dataPtr->currentSDF += "<interpolate_x>true</interpolate_x>";

  this->dataPtr->currentSDF +=
           "</animation>\
           " + trajectory + "\
         </actor>\
       </sdf>\
      ";

  // Spawn actor
  gazebo::msgs::Factory msg;
  msg.set_sdf(this->dataPtr->currentSDF);

  this->dataPtr->factoryPub->Publish(msg);
}

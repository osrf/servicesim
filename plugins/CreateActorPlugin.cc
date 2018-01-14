/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <gazebo/transport/Node.hh>
#include "CreateActorPlugin.hh"

std::map<std::string, std::string> skinMap;
std::map<std::string, std::string> animMap;
unsigned int count{0};

namespace gazebo
{
  /// \brief Private data for the CreateActorPlugin class
  class CreateActorPluginPrivate
  {
    /// \brief Pointer to a node for communication.
    public: transport::NodePtr gzNode;

    /// \brief keyboard publisher.
    public: transport::PublisherPtr factoryPub;

    public: std::string currentSDF{""};
  };
}

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CreateActorPlugin)

/////////////////////////////////////////////////
CreateActorPlugin::CreateActorPlugin()
  : GUIPlugin(), dataPtr(new CreateActorPluginPrivate)
{
  // Maps
  skinMap["Green shirt"] = "SKIN_man_green_shirt";
  skinMap["Red shirt"] = "SKIN_man_red_shirt";
  skinMap["Blue shirt"] = "SKIN_man_blue_shirt";

  animMap["Talking A"] = "ANIMATION_talking_a";
  animMap["Talking B"] = "ANIMATION_talking_b";

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

  // 0: Actor type
  {
    // Label
    auto label = new QLabel(tr("<b>Choose actor type</b>"));
    label->setMaximumHeight(50);

    // Idler
    auto idlerButton = new QPushButton(tr("Idler"));
    this->connect(idlerButton, &QPushButton::clicked, [=]()
    {
      mainLayout->setCurrentIndex(1);
    });

    // Layout
    auto layout = new QGridLayout;
    layout->setSpacing(0);
    layout->addWidget(label, 0, 0);
    layout->addWidget(idlerButton, 1, 0);

    // Widget
    auto widget = new QWidget();
    widget->setLayout(layout);

    mainLayout->addWidget(widget);
  }

  // 1: Idler skin
  {
    // Label
    auto label = new QLabel(tr("<b>Choose a skin and animation for the actor</b>"));
    label->setMaximumHeight(50);

    // Skin combo
    auto skinCombo = new QComboBox();
    for (auto s : skinMap)
      skinCombo->addItem(QString::fromStdString(s.first));

    // Animation combo
    auto animCombo = new QComboBox();
    for (auto a : animMap)
      animCombo->addItem(QString::fromStdString(a.first));

    // Back button
    auto backButton = new QPushButton(tr("Back"));
    this->connect(backButton, &QPushButton::clicked, [=]()
    {
      mainLayout->setCurrentIndex(0);
    });

    // Next button
    auto nextButton = new QPushButton(tr("Next"));
    this->connect(nextButton, &QPushButton::clicked, [=]()
    {
      auto skin = skinMap[skinCombo->currentText().toStdString()];
      auto anim = animMap[animCombo->currentText().toStdString()];

      this->dataPtr->currentSDF =
          "<?xml version='1.0' ?>\
           <sdf version='" SDF_VERSION "'>\
             <actor name='actor_" + std::to_string(count) + "'>\
               <pose>0 0 1.25 0 0 0</pose>\
               <skin>\
                 <filename>model://actor/meshes/" + skin + ".dae</filename>\
               </skin>\
               <animation name=\"animation\">\
                 <filename>model://actor/meshes/" + anim + ".dae</filename>\
               </animation>\
             </actor>\
           </sdf>\
          ";

      gazebo::msgs::Factory msg;
      msg.set_sdf(this->dataPtr->currentSDF);

      this->dataPtr->factoryPub->Publish(msg);

      mainLayout->setCurrentIndex(2);
      count++;
    });

    // Layout
    auto layout = new QGridLayout;
    layout->setSpacing(0);
    layout->addWidget(label, 0, 0, 1, 3);
    layout->addWidget(new QLabel("Skin"), 1, 0);
    layout->addWidget(skinCombo, 1, 1);
    layout->addWidget(new QLabel("Animation"), 2, 0);
    layout->addWidget(animCombo, 2, 1);
    layout->addWidget(backButton, 3, 0);
    layout->addWidget(nextButton, 3, 1);

    // Widget
    auto widget = new QWidget();
    widget->setLayout(layout);

    mainLayout->addWidget(widget);
  }

  // 2: Idler pose and export
  {
    // Label
    auto label = new QLabel(tr("<b>The actor has been spawned, reposition it and export.</b>"));
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
  this->resize(400, 400);

  this->setStyleSheet(
      "QFrame {background-color: rgba(100, 100, 100, 255); color: black;}");
}

/////////////////////////////////////////////////
CreateActorPlugin::~CreateActorPlugin()
{
  this->dataPtr->factoryPub.reset();
  this->dataPtr->gzNode->Fini();
}

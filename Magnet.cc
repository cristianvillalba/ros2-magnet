/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "Magnet.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    magnet::Magnet,
    gz::sim::System,
	magnet::Magnet::ISystemConfigure,
	magnet::Magnet::ISystemPreUpdate,
	magnet::Magnet::ISystemPostUpdate)

using namespace magnet;
using namespace gz;
using namespace sim;
using namespace systems;

Magnet::Magnet()
{
}

Magnet::~Magnet()
{
}

void Magnet::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->entity = _entity;
  gzmsg << "Magnet for entity [" << _entity << "]" << std::endl;

    auto linkName = _sdf->Get<std::string>("link_name");
    
    auto model = Model(_entity);
    this->linkEntity = model.LinkByName(_ecm, "chassis");
}

void Magnet::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    if (_info.iterations % 3000)
    {
        //gzmsg << "Applying force..."  << std::endl;
        
        auto linkWorldPose = worldPose(this->linkEntity, _ecm);
        
        math::Vector3d upwardI;

        upwardI = linkWorldPose.Rot().RotateVector(math::Vector3d::UnitZ);
        
        math::Vector3d finalForce = upwardI * -10.0f;
        
        //gzmsg << spanwiseI.X() << " " << spanwiseI.Y() << " " << spanwiseI.Z() << std::endl;
        
        Link link(this->linkEntity);
        link.AddWorldForce(_ecm, finalForce);
    }
    
}



// Here we implement the PostUpdate function, which is called at every
// iteration.
void Magnet::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  // This is a simple example of how to get information from UpdateInfo.
  //std::string msg = "Hello, world Magnet! Simulation is ";
  //if (!_info.paused)
  //  msg += "not ";
  //msg += "paused.";

  // Messages printed with gzmsg only show when running with verbosity 3 or
  // higher (i.e. gz sim -v 3)
  //gzmsg << msg << std::endl;
}

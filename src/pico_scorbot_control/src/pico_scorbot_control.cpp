// Copyright (c) 2022, juandvergara
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "pico_scorbot_control/pico_scorbot_control.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::return_type;

namespace pico_scorbot_control
{

  std::vector<StateInterface> PicoScorbotControl::export_state_interfaces()
  {
    std::vector<StateInterface> state_interfaces;

    return state_interfaces;
  }

  std::vector<CommandInterface> PicoScorbotControl::export_command_interfaces()
  {
    std::vector<CommandInterface> command_interfaces;

    return command_interfaces;
  }

  return_type PicoScorbotControl::start()
  {

    return return_type::OK;
  }

  return_type PicoScorbotControl::stop()
  {

    return return_type::OK;
  }

  return_type PicoScorbotControl::read()
  {
    // TODO(anyone): read robot states

    return hardware_interface::return_type::OK;
  }

  return_type PicoScorbotControl::write()
  {
    // TODO(anyone): write robot's commands'

    return hardware_interface::return_type::OK;
  }

} 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    pico_scorbot_control::PicoScorbotControl, hardware_interface::SystemInterface)

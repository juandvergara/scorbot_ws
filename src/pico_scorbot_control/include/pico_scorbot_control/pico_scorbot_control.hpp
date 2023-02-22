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

#ifndef PICO_SCORBOT_CONTROL__PICO_SCORBOT_CONTROL_HPP_
#define PICO_SCORBOT_CONTROL__PICO_SCORBOT_CONTROL_HPP_

#include <string>
#include <vector>

#include "pico_scorbot_control/visibility_control.h"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pico_scorbot_control/pico_comms.h"

using hardware_interface::return_type;
using hardware_interface::BaseInterface;
using hardware_interface::SystemInterface;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

namespace pico_scorbot_control
{
  class PicoScorbotControl : public BaseInterface<SystemInterface>
  {
  public:

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type configure(const hardware_interface::HardwareInfo &info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<StateInterface> export_state_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<CommandInterface> export_command_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type start() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type stop() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type read() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    return_type write() override;

  private:
    std::vector<float> hw_commands_;
    std::vector<float> hw_states_;
  };

} // namespace pico_scorbot_control

#endif // PICO_SCORBOT_CONTROL__PICO_SCORBOT_CONTROL_HPP_

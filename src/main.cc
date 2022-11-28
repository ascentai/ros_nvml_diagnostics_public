// BSD License
//
// Copyright (c) 2021, Ascent Robotics, Inc.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Ascent Robotics, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author Thomas Kostas/thomas.kostas@ascent.ai

#include <algorithm>
#include <memory>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <nvml.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sys/types.h>

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "ros_nvml_diagnostics/device.hh"
#include "ros_nvml_diagnostics/device_diagnostic_publisher.hh"
#include "ros_nvml_diagnostics/general_information.hh"
#include "ros_nvml_diagnostics/utilities.hh"
#include <boost/bind/placeholders.hpp>

namespace
{
void diagnose_gpus(ros_nvml_diagnostics::DeviceIdentifierType identifier_type,
                   ros::NodeHandle& nh,
                   double update_frequency)
{
    unsigned int gpu_count;
    nvml_tools::ensure_call_nvml(
        "Failed to get gpu count", nvmlDeviceGetCount_v2, &gpu_count);

    std::vector<ros::Timer> timer_trigger_vec;
    std::vector<ros_nvml_diagnostics::DeviceDiagnosticPublisher>
        diag_publisher_vec;

    timer_trigger_vec.reserve(gpu_count);
    std::vector<
        std::unique_ptr<ros_nvml_diagnostics::DeviceDiagnosticPublisher>>
        diag_vector;

    ros::MultiThreadedSpinner spinner;
    for (unsigned int gpu_index = 0; gpu_index < gpu_count; gpu_index++)
    {
        auto device = ros_nvml_diagnostics::Device::build_from_index(
            gpu_index, identifier_type);

        auto diag_publisher =
            std::make_unique<ros_nvml_diagnostics::DeviceDiagnosticPublisher>(
                device);

        diag_vector.push_back(std::move(diag_publisher));

        auto& current_diagnostic = diag_vector[gpu_index];
        auto current_timer =
            nh.createTimer(ros::Duration(1.0 / update_frequency),
                           [&current_diagnostic](const ros::TimerEvent&) {
                               current_diagnostic->emit_diagnostics();
                           });

        timer_trigger_vec.push_back(std::move(current_timer));
    }
    spinner.spin();
}

ros_nvml_diagnostics::DeviceIdentifierType
string_to_identifier_type(std::string& identifier)
{
    static const std::unordered_map<std::string,
                                    ros_nvml_diagnostics::DeviceIdentifierType>
        text_map = {
            { "index",
              ros_nvml_diagnostics::DeviceIdentifierType::device_index },
            { "uuid", ros_nvml_diagnostics::DeviceIdentifierType::device_uuid },
            { "serial",
              ros_nvml_diagnostics::DeviceIdentifierType::device_serial }
        };
    auto found = text_map.find(identifier);
    if (found != text_map.end())
    {
        return found->second;
    }
    else
    {
        ROS_WARN_STREAM(identifier + " is invalid, defaulting to index type "
                                     "identifier");
        return ros_nvml_diagnostics::DeviceIdentifierType::device_index;
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gpu_diagnostics");
    ros::NodeHandle node_handle("~");

    nvml_tools::ensure_call_nvml("Nvml Initialization failed", nvmlInit_v2);
    ros_nvml_diagnostics::log_system_informations();

    std::string gpu_identifier;
    node_handle.param<std::string>("gpu_identifier", gpu_identifier, "index");

    double update_frequency;
    node_handle.param<double>("diagnostic_frequency", update_frequency, 1);
    if (update_frequency <= 0)
    {
        ROS_WARN("Update Frequency of 0 hz exiting");
        nvml_tools::ensure_call_nvml("Failed to shutdown Nvml", nvmlShutdown);
        return 0;
    }

    ros::MultiThreadedSpinner spinner;

    diagnose_gpus(string_to_identifier_type(gpu_identifier),
                  node_handle,
                  update_frequency);

    nvml_tools::ensure_call_nvml("Failed to shutdown Nvml", nvmlShutdown);
    return 0;
}

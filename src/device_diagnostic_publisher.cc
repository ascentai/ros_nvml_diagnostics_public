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

#include "ros_nvml_diagnostics/device_diagnostic_publisher.hh"

#include <string>
#include <type_traits>
#include <vector>

#include "ros_nvml_diagnostics/device.hh"
namespace ros_nvml_diagnostics
{
DeviceDiagnosticPublisher::DeviceDiagnosticPublisher(const Device& device)
  : device_(device)
{
    diagnostics_updater_.setHardwareID(device_.get_device_identifier());

    auto diagnostic_pub_func =
        std::bind(&DeviceDiagnosticPublisher::diagnostics_callback,
                  this,
                  std::placeholders::_1);

    diagnostics_updater_.add(device_.get_device_descriptive_name(),
                             diagnostic_pub_func);
}

void DeviceDiagnosticPublisher::diagnostics_callback(
    diagnostic_updater::DiagnosticStatusWrapper& stat) const
{
    std::vector<std::string> diag_message_vector;
    populate_usage_info(stat, diag_message_vector);
    populate_temperature_info(stat, diag_message_vector);
    populate_clock_info(stat, diag_message_vector);
    populate_power_info(stat, diag_message_vector);
    populate_memory_info(stat, diag_message_vector);

    std::string message = "";

    for (const auto& it : diag_message_vector)
    {
        message += it + "\n";
    }

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, message);
}

void DeviceDiagnosticPublisher::populate_usage_info(
    diagnostic_updater::DiagnosticStatusWrapper& stat,
    std::vector<std::string>& diag_message) const
{
    if (auto utilization = device_.get_utilization())
    {
        stat.add("Graphic processor usage (%)", utilization->gpu);
        stat.add("V-RAM pressure (%)", utilization->memory);
    }
    else
    {
        diag_message.push_back("Utilization: Failed to get utilization data");
    }
}

void DeviceDiagnosticPublisher::populate_temperature_info(
    diagnostic_updater::DiagnosticStatusWrapper& stat,
    std::vector<std::string>& diag_message) const
{
    if (auto temperature = device_.get_temperature())
    {
        stat.add("Temperature (C)", *temperature);
    }
    else
    {
        diag_message.push_back("Temperature: Failed to get data");
    }
}

void DeviceDiagnosticPublisher::populate_power_info(
    diagnostic_updater::DiagnosticStatusWrapper& stat,
    std::vector<std::string>& diag_message) const
{
    if (auto power = device_.get_power_consumption_watts())
    {
        stat.add("Power consumption (W)", *power);
    }
    else
    {
        diag_message.push_back("Power: Failed to get data");
    }
}

void DeviceDiagnosticPublisher::populate_clock_info(
    diagnostic_updater::DiagnosticStatusWrapper& stat,
    std::vector<std::string>& diag_message) const
{
    auto clock_data = device_.get_clock_data();

    auto manage_clck_info =
        [](std::optional<unsigned int> data,
           std::string&& data_type,
           std::vector<std::string>& l_diag_message,
           diagnostic_updater::DiagnosticStatusWrapper& l_stat) {
            if (data.has_value())
            {
                l_stat.add(data_type + " (MHz)", *data);
            }
            else
            {
                l_diag_message.push_back(data_type + ": Failed to get data");
            }
        };

    manage_clck_info(clock_data.graphics_clock_,
                     std::string("Graphics clock"),
                     diag_message,
                     stat);
    manage_clck_info(
        clock_data.sm_clock_, std::string("SM clock"), diag_message, stat);
    manage_clck_info(clock_data.memory_clock_,
                     std::string("Memory clock"),
                     diag_message,
                     stat);
    manage_clck_info(clock_data.video_clock_,
                     std::string("Video clock"),
                     diag_message,
                     stat);
}

void DeviceDiagnosticPublisher::populate_memory_info(
    diagnostic_updater::DiagnosticStatusWrapper& stat,
    std::vector<std::string>& diag_message) const
{
    auto bytes_to_mbytes = [](unsigned long long bytes) {
        constexpr auto factor = 1024 * 1024;
        return (bytes / factor);
    };

    auto mem_info = device_.get_memory_info();
    if (mem_info.has_value())
    {
        stat.add("Total memory (MB)", bytes_to_mbytes(mem_info->total));
        stat.add("Used memory (MB)", bytes_to_mbytes(mem_info->used));
        stat.add("Free memory (MB)", bytes_to_mbytes(mem_info->free));
        stat.add("Used memory (%)",
                 static_cast<unsigned int>(static_cast<float>(mem_info->used) /
                                           static_cast<float>(mem_info->total) *
                                           100.f));
    }
    else
    {
        diag_message.push_back("Failed to get memory informations");
    }
}

void DeviceDiagnosticPublisher::emit_diagnostics()
{
    diagnostics_updater_.update();
}

}  // namespace ros_nvml_diagnostics
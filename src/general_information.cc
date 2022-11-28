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

#include <ros/console.h>

#include "ros_nvml_diagnostics/utilities.hh"

namespace ros_nvml_diagnostics
{
void log_system_informations()
{
    constexpr auto version_string_length = 100;
    int cuda_driver_version = 0;
    std::string driver_version(version_string_length, '\0');
    std::string nvml_version(version_string_length, '\0');

    auto ret = nvml_tools::call_nvml("Failed to get cuda driver version",
                                     nvmlSystemGetCudaDriverVersion,
                                     &cuda_driver_version);
    if (!ret.success_)
        ROS_WARN_STREAM(ret.error_message_);
    else
        ROS_INFO("Cuda driver version is %i", cuda_driver_version);

    ret = nvml_tools::call_nvml("Failed to get driver version",
                                nvmlSystemGetDriverVersion,
                                driver_version.data(),
                                version_string_length);
    if (!ret.success_)
        ROS_WARN_STREAM(ret.error_message_);
    else
        ROS_INFO("System driver version %s", driver_version.c_str());

    ret = nvml_tools::call_nvml("Failed to get NVML version",
                                nvmlSystemGetNVMLVersion,
                                nvml_version.data(),
                                version_string_length);
    if (!ret.success_)
        ROS_WARN_STREAM(ret.error_message_);
    else
        ROS_INFO("NVML version %s", nvml_version.c_str());
}
}  // namespace ros_nvml_diagnostics
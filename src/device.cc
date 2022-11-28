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

#include "ros_nvml_diagnostics/device.hh"

#include <optional>
#include <string>
#include <unordered_map>

#include <nvml.h>
#include <ros/console.h>

#include "ros_nvml_diagnostics/utilities.hh"
namespace ros_nvml_diagnostics
{
Device Device::build_from_index(unsigned int index,
                                DeviceIdentifierType device_reference_type)
{
    return Device(index, device_reference_type);
}

Device Device::build_from_uuid(const std::string& uuid,
                               DeviceIdentifierType device_reference_type)
{
    return Device(
        uuid, DeviceIdentifierType::device_uuid, device_reference_type);
}

Device Device::build_from_serial(const std::string& serial,
                                 DeviceIdentifierType device_reference_type)
{
    return Device(
        serial, DeviceIdentifierType::device_serial, device_reference_type);
}

Device::Device(const std::string& device_identifier,
               DeviceIdentifierType device_identifier_type,
               DeviceIdentifierType device_reference_type)
  : device_identifier_(device_identifier),
    device_identifier_type_(device_identifier_type),
    device_reference_type_(device_reference_type)
{
    initialize_device();
}

Device::Device(unsigned int index, DeviceIdentifierType device_reference_type)
  : device_identifier_(std::to_string(index)),
    device_identifier_type_(DeviceIdentifierType::device_index),
    device_reference_type_(device_reference_type)
{
    initialize_device();
}

namespace
{
std::string handle_to_reference(nvmlDevice_t& device,
                                DeviceIdentifierType reference_type)
{
    std::string e_prefix = "Failed to get ";

    if (reference_type == DeviceIdentifierType::device_index)
    {
        unsigned int index;
        nvml_tools::ensure_call_nvml(
            e_prefix + "index", nvmlDeviceGetIndex, device, &index);
        return std::to_string(index);
    }
    else
    {
        // According to documentation getUUID returns at most 96 characters
        // including NULL terminator, 30 for getSerial
        constexpr unsigned int max_id_length = 96;
        std::array<char, max_id_length> identifier;

        if (reference_type == DeviceIdentifierType::device_serial)
        {
            nvml_tools::ensure_call_nvml(e_prefix + "serial",
                                         nvmlDeviceGetSerial,
                                         device,
                                         identifier.data(),
                                         max_id_length);
        }
        else if (reference_type == DeviceIdentifierType::device_uuid)
        {
            nvml_tools::ensure_call_nvml(e_prefix + "uuid",
                                         nvmlDeviceGetUUID,
                                         device,
                                         identifier.data(),
                                         max_id_length);
        }
        return std::string(identifier.data());
    }
}
}  // namespace

void Device::initialize_device()
{
    auto fail_msg =
        std::string("Failed to get handle for GPU " + device_identifier_);

    if (device_identifier_type_ == DeviceIdentifierType::device_index)
    {
        unsigned int index = std::stoi(device_identifier_);
        nvml_tools::ensure_call_nvml(
            fail_msg, nvmlDeviceGetHandleByIndex_v2, index, &device_handle_);
    }
    else if (device_identifier_type_ == DeviceIdentifierType::device_serial)
    {
        nvml_tools::ensure_call_nvml(fail_msg,
                                     nvmlDeviceGetHandleBySerial,
                                     device_identifier_.c_str(),
                                     &device_handle_);
    }
    else if (device_identifier_type_ == DeviceIdentifierType::device_uuid)
    {
        nvml_tools::ensure_call_nvml(fail_msg,
                                     nvmlDeviceGetHandleByUUID,
                                     device_identifier_.c_str(),
                                     &device_handle_);
    }
    generate_descriptive_name();
}

std::optional<nvmlUtilization_t> Device::get_utilization() const
{
    nvmlUtilization_t utilization;
    auto ret = nvml_tools::call_nvml(build_error_message("utilization"),
                                     nvmlDeviceGetUtilizationRates,
                                     device_handle_,
                                     &utilization);
    if (ret.success_ == true)
    {
        return utilization;
    }
    else
    {
        ROS_ERROR_STREAM(ret.error_message_);
        return std::nullopt;
    }
}

std::optional<unsigned int> Device::get_temperature() const
{
    unsigned int temperature = 0;
    auto ret = nvml_tools::call_nvml(build_error_message("temperature"),
                                     nvmlDeviceGetTemperature,
                                     device_handle_,
                                     NVML_TEMPERATURE_GPU,
                                     &temperature);
    if (ret.success_ == true)
    {
        return temperature;
    }
    else
    {
        ROS_ERROR_STREAM(ret.error_message_);
        return std::nullopt;
    }
}

std::string Device::build_error_message(const std::string& error_type) const
{
    auto fail_msg_prefix = std::string("Failed to get GPU ");
    auto fail_msg_suffix = std::string(" for device ") + device_identifier_;
    return fail_msg_prefix + error_type + fail_msg_suffix;
}

clock_data Device::get_clock_data() const
{
    clock_data clockdata;
    unsigned int clock_value;

    auto ret = nvml_tools::call_nvml(build_error_message("graphic clock"),
                                     nvmlDeviceGetClockInfo,
                                     device_handle_,
                                     NVML_CLOCK_GRAPHICS,
                                     &clock_value);
    if (ret.success_ == true)
        clockdata.graphics_clock_ = clock_value;
    else
        ROS_WARN_STREAM(ret.error_message_);

    ret = nvml_tools::call_nvml(build_error_message("sm clock"),
                                nvmlDeviceGetClockInfo,
                                device_handle_,
                                NVML_CLOCK_SM,
                                &clock_value);
    if (ret.success_ == true)
        clockdata.sm_clock_ = clock_value;
    else
        ROS_WARN_STREAM(ret.error_message_);

    ret = nvml_tools::call_nvml(build_error_message("memory clock"),
                                nvmlDeviceGetClockInfo,
                                device_handle_,
                                NVML_CLOCK_MEM,
                                &clock_value);
    if (ret.success_ == true)
        clockdata.memory_clock_ = clock_value;
    else
        ROS_WARN_STREAM(ret.error_message_);

    ret = nvml_tools::call_nvml(build_error_message("video clock"),
                                nvmlDeviceGetClockInfo,
                                device_handle_,
                                NVML_CLOCK_VIDEO,
                                &clock_value);
    if (ret.success_ == true)
        clockdata.memory_clock_ = clock_value;
    else
        ROS_WARN_STREAM(ret.error_message_);

    return clockdata;
}

std::string Device::get_device_descriptive_name() const
{
    return device_descriptive_name_;
}

std::string Device::get_device_identifier() const
{
    return device_identifier_;
}

void Device::generate_descriptive_name()
{
    std::string prefix = std::string("GPU with ");
    std::string id_type = std::string("unknown type id");
    std::string device_reference_name =
        handle_to_reference(device_handle_, device_reference_type_);

    if (auto id_type_found = identifier_reference_to_text())
    {
        id_type = *id_type_found;
    }
    device_descriptive_name_ = prefix + id_type + " " + device_reference_name;
}

std::optional<const std::string> Device::identifier_reference_to_text() const
{
    static const std::unordered_map<DeviceIdentifierType, std::string>
        text_map = { { DeviceIdentifierType::device_index, "index" },
                     { DeviceIdentifierType::device_uuid, "UUID" },
                     { DeviceIdentifierType::device_serial, "serial" } };

    auto found = text_map.find(device_reference_type_);

    if (found != text_map.end())
    {
        return found->second;
    }
    else
    {
        ROS_ERROR("Idenditifier type %i is unknown.", device_identifier_type_);
        return std::nullopt;
    }
}

std::optional<unsigned int> Device::get_power_consumption_watts() const
{
    unsigned int power_milliwatts = 0;
    auto ret = nvml_tools::call_nvml(build_error_message("power consumption"),
                                     nvmlDeviceGetPowerUsage,
                                     device_handle_,
                                     &power_milliwatts);
    if (ret.success_ == true)
    {
        unsigned int power_watts = power_milliwatts / 1000;
        return power_watts;
    }
    else
    {
        ROS_ERROR_STREAM(ret.error_message_);
        return std::nullopt;
    }
}

std::optional<nvmlMemory_t> Device::get_memory_info() const
{
    nvmlMemory_t mem_info;
    auto ret = nvml_tools::call_nvml(build_error_message("Memory informations"),
                                     nvmlDeviceGetMemoryInfo,
                                     device_handle_,
                                     &mem_info);

    if (ret.success_ == true)
    {
        return mem_info;
    }
    else
    {
        ROS_ERROR_STREAM(ret.error_message_);
        return std::nullopt;
    }
}

}  // namespace ros_nvml_diagnostics
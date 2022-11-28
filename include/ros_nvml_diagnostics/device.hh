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

#pragma once

#include <optional>
#include <string>

#include <nvml.h>

namespace ros_nvml_diagnostics
{
struct clock_data
{
    std::optional<unsigned int> graphics_clock_ = std::nullopt;
    std::optional<unsigned int> sm_clock_ = std::nullopt;
    std::optional<unsigned int> memory_clock_ = std::nullopt;
    std::optional<unsigned int> video_clock_ = std::nullopt;
};

enum class DeviceIdentifierType
{
    device_index,
    device_uuid,
    device_serial
};

class Device
{
  public:
    /**
     * @brief
     *
     * @param serial GPU serial
     * @param device_reference_type indentifier type to use for gpu descriptive
     * name
     * @return Device
     */
    static Device build_from_serial(const std::string& serial,
                                    DeviceIdentifierType device_reference_type =
                                        DeviceIdentifierType::device_serial);
    /**
     * @brief
     * Hand
     * @param uuid GPU uuid
     * @param device_reference_type indentifier type to use for gpu descriptive
     * name
     * @return Device
     */
    static Device build_from_uuid(const std::string& uuid,
                                  DeviceIdentifierType device_reference_type =
                                      DeviceIdentifierType::device_uuid);
    /**
     * @brief
     *
     * @param index Gpu device index
     * @param device_reference_type indentifier type to use for gpu descriptive
     * name
     * @return Device
     */
    static Device build_from_index(unsigned int index,
                                   DeviceIdentifierType device_reference_type =
                                       DeviceIdentifierType::device_index);

    std::optional<nvmlUtilization_t> get_utilization() const;
    std::optional<unsigned int> get_temperature() const;
    std::optional<unsigned int> get_power_consumption_watts() const;
    std::optional<nvmlMemory_t> get_memory_info() const;
    clock_data get_clock_data() const;

    std::string get_device_descriptive_name() const;
    std::string get_device_identifier() const;

  private:
    Device(unsigned int index, DeviceIdentifierType device_reference_type);
    Device(const std::string& device_identifier,
           DeviceIdentifierType device_identifier_type,
           DeviceIdentifierType device_reference_type);
    void initialize_device();

    std::string prepare_error_message() const;
    std::optional<const std::string> identifier_reference_to_text() const;
    void generate_descriptive_name();
    std::string build_error_message(const std::string& error_type) const;

    nvmlDevice_t device_handle_;
    std::string device_identifier_;
    DeviceIdentifierType device_identifier_type_;
    DeviceIdentifierType device_reference_type_;
    std::string device_descriptive_name_;
};

}  // namespace ros_nvml_diagnostics
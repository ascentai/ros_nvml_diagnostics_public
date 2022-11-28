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

#include <diagnostic_updater/diagnostic_updater.h>

#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "ros_nvml_diagnostics/device.hh"

namespace ros_nvml_diagnostics
{
class DeviceDiagnosticPublisher
{
  public:
    explicit DeviceDiagnosticPublisher(const Device& device);
    void emit_diagnostics();

  private:
    void diagnostics_callback(
        diagnostic_updater::DiagnosticStatusWrapper& stat) const;
    void populate_usage_info(diagnostic_updater::DiagnosticStatusWrapper& stat,
                             std::vector<std::string>& diag_message) const;
    void
    populate_temperature_info(diagnostic_updater::DiagnosticStatusWrapper& stat,
                              std::vector<std::string>& diag_message) const;
    void populate_clock_info(diagnostic_updater::DiagnosticStatusWrapper& stat,
                             std::vector<std::string>& diag_message) const;
    void populate_power_info(diagnostic_updater::DiagnosticStatusWrapper& stat,
                             std::vector<std::string>& diag_message) const;
    void populate_memory_info(diagnostic_updater::DiagnosticStatusWrapper& stat,
                              std::vector<std::string>& diag_message) const;

    Device device_;
    diagnostic_updater::Updater diagnostics_updater_;
};

}  // namespace ros_nvml_diagnostics
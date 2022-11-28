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
#include <stdexcept>
#include <string>
#include <utility>

#include <nvml.h>

struct nvml_call_status_t
{
    bool success_;
    std::string error_message_ = "";
};

namespace nvml_error_manager
{
std::optional<std::string> get_error_string(nvmlReturn_t nvml_return);
}

namespace nvml_tools
{
template <typename FuncType, typename... Params>
nvml_call_status_t call_nvml(const std::string& extra_fail_info,
                             const FuncType& nvml_func,
                             Params... parameters)
{
    auto call_return = nvml_func(parameters...);
    auto error_message = nvml_error_manager::get_error_string(call_return);
    nvml_call_status_t to_return;

    if (error_message.has_value())
    {
        to_return.error_message_ =
            std::string(error_message.value() + extra_fail_info);
        to_return.success_ = false;
        return to_return;
    }
    else
    {
        to_return.success_ = true;
        return to_return;
    }
}

template <typename FuncType, typename... Params>
void ensure_call_nvml(const std::string& extra_fail_info,
                      const FuncType& nvml_func,
                      Params... parameters)
{
    auto res = call_nvml(extra_fail_info, nvml_func, parameters...);
    if (!res.success_)
    {
        throw std::runtime_error(res.error_message_);
    }
}

}  // namespace nvml_tools

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

#include <optional>
#include <string>
#include <unordered_map>

#include <nvml.h>

namespace nvml_error_manager
{
std::optional<std::string> get_error_string(nvmlReturn_t nvml_return)
{
    static const std::unordered_map<nvmlReturn_t, std::string> error_string = {
        { NVML_ERROR_DRIVER_NOT_LOADED, "Nvidia driver is not loaded. " },
        { NVML_ERROR_NO_PERMISSION,
          "No permission to communicate with driver. " },
        { NVML_ERROR_UNINITIALIZED,
          "NVML does not appear to have been initialized. " },
        { NVML_ERROR_LIB_RM_VERSION_MISMATCH,
          "Driver Library version mismatch. " },
        { NVML_ERROR_UNKNOWN, "Unknown error. " },
        { NVML_ERROR_GPU_IS_LOST,
          "GPU is lost, the target "
          "GPU has fallen off the "
          "bus or is otherwise "
          "inaccessible. " },
        { NVML_ERROR_NOT_SUPPORTED, " Device does not support this feature. " },
        { NVML_ERROR_INVALID_ARGUMENT, "Invalid device or data. " }
    };

    auto found = error_string.find(nvml_return);

    if (nvml_return != NVML_SUCCESS)
    {
        if (found != error_string.end())
            return found->second;
        else
            return "Unmanaged error. ";
    }
    else
        return std::nullopt;
}

}  // namespace nvml_error_manager

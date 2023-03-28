// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#pragma once

#include <memory>
#include <mivins/common/camera_fwd.h>
//#include <mivins/vio_common/logging.hpp>
#include <../../vins_backend/include/vins_backend_interface.h>

namespace mivins
{

    namespace vins_backend_factory
    {

        std::shared_ptr<VinsBackendInterface> makeBackend(
            const std::string config_file, const CameraBundlePtr &camera_bundle, const int cam_size);

    } // namespace vins_backend_factory
} // namespace mivins

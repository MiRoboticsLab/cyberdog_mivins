// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <glog/logging.h>
#include <functional>

namespace std
{

    template <typename Scalar, int Rows, int Cols>
    struct hash<Eigen::Matrix<Scalar, Rows, Cols>>
    {
        size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols> &matrix) const
        {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i)
            {
                Scalar elem = *(matrix.data() + i);
                seed ^=
                    std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

} // namespace std

namespace mivins
{

    // e.g. rpg::Aligned<std::vector, Eigen::Vector2d>
    // Consider using Matrix2Xd instead in such cases.
    template <template <typename Type, typename Allocator> class StlContainer,
              typename EigenType>
    using Aligned = StlContainer<
        EigenType, Eigen::aligned_allocator<EigenType>>;

    namespace aligned
    {

        template <typename KeyType, typename EigenType>
        using Map =
            std::map<KeyType, EigenType, std::less<KeyType>,
                     Eigen::aligned_allocator<std::pair<const KeyType, EigenType>>>;

        template <typename KeyType, typename EigenType>
        using UnorderedMap =
            std::unordered_map<KeyType, EigenType, std::hash<KeyType>,
                               std::equal_to<KeyType>,
                               Eigen::aligned_allocator<std::pair<const KeyType, EigenType>>>;

        template <typename EigenType, typename ValueType>
        using UnorderedMapEigenKey =
            std::unordered_map<EigenType, ValueType, std::hash<EigenType>,
                               std::equal_to<EigenType>,
                               Eigen::aligned_allocator<std::pair<const EigenType, ValueType>>>;

        template <typename EigenType>
        using UnorderedSet =
            std::unordered_set<EigenType, std::hash<EigenType>,
                               std::equal_to<EigenType>, Eigen::aligned_allocator<EigenType>>;

        // See cols_vec.h for the inverse.
        template <typename Scalar, int Rows>
        Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> ToMat(
            const Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> &container)
        {
            Eigen::Matrix<Scalar, Rows, Eigen::Dynamic> result(Rows, container.size());
            for (size_t i = 0u; i < container.size(); ++i)
            {
                result.col(i) = container[i];
            }
            return result;
        }

    } // namespace aligned
} // namespace mivins

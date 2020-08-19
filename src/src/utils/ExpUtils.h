//
// Copyright (c) 2020 Terry Chen <ty6chen@uwaterloo.ca>
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the
// Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
// Boston, MA  02110-1301, USA.
//

#ifndef SGMWSMCPP_EXPUTILS_H
#define SGMWSMCPP_EXPUTILS_H

#include <string>
#include <vector>

#include "knot/data/KnotDataReader.h"

namespace sgm
{
struct ExpUtilsConfig
{
    static int concrete_particles;
    static int max_virtual_particles;
    static std::string output_path;
    static double lambda;
    static double tol;
    static bool exact_sampling;
    static bool sequential_matching;
};

namespace ExpUtils
{
std::vector<std::vector<KnotDataReader::Segment>> read_test_boards(const std::vector<std::string> &boards,
                                                                   bool reverse_sequence);

template <typename KnotType>
std::vector<std::vector<std::pair<std::vector<edge_type_base<KnotType>>,
                                  std::vector<node_type_base<KnotType>>>>> unpack(
    const std::vector<std::vector<KnotDataReader::Segment>> &instances
) {
    std::vector<std::vector<std::pair<std::vector<edge_type_base<KnotType>>,
                                      std::vector<node_type_base<KnotType>>>>> data;
    for (auto &instance : instances) {
        std::vector<std::pair<std::vector<edge_type_base<KnotType>>, std::vector<node_type_base<KnotType>>>> datum;
        for (auto &segment : instance) {
            std::vector<edge_type_base<KnotType>> edges;
            for (auto &matching : segment.label_to_edge()) {
                edges.emplace_back(matching.second);
            }
            datum.emplace_back(std::move(edges), segment.knots());
        }
        data.emplace_back(std::move(datum));
    }
    return data;
}

template <typename KnotType>
std::vector<std::pair<std::vector<edge_type_base<KnotType>>, std::vector<node_type_base<KnotType>>>> pack(
    const std::vector<std::vector<std::pair<std::vector<edge_type_base<KnotType>>,
                                            std::vector<node_type_base<KnotType>>>>>
    &instances
) {
    std::vector<std::pair<std::vector<edge_type_base<KnotType>>,
                          std::vector<node_type_base<KnotType>>>> packed_instances;

    for (const auto &instance : instances) {
        packed_instances.insert(packed_instances.end(), instance.begin(), instance.end());
    }

    return packed_instances;
}
}
}

#endif //SGMWSMCPP_EXPUTILS_H

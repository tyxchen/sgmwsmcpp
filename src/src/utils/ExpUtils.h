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

namespace ExpUtils
{
std::vector<KnotDataReader::Segment> read_test_boards(const std::vector<std::string> &boards,
                                                      bool reverse_sequence);

template <typename KnotType>
std::vector<std::pair<std::vector<edge_type_base<KnotType>>, std::vector<node_type_base<KnotType>>>> unpack(
    const std::vector<KnotDataReader::Segment> &instances
) {
    std::vector<std::pair<std::vector<edge_type_base<KnotType>>,
                          std::vector<node_type_base<KnotType>>>> data;
    for (auto &segment : instances) {
        data.emplace_back(std::vector<edge_type_base<KnotType>>(), segment.knots());
        auto &edges = data.back().first;
        for (auto &matching : segment.label_to_edge()) {
            edges.emplace_back(matching.second);
        }
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

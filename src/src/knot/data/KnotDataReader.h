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

#ifndef SGMWSMCPP_KNOTDATAREADERCONFIG_H
#define SGMWSMCPP_KNOTDATAREADERCONFIG_H

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "utils/types.h"
#include "knot/data/EllipticalKnot.h"

namespace sgm
{

namespace KnotDataReader
{

class Segment
{
public:
    using node_type = node_type_base<EllipticalKnot>;
    using edge_type = edge_type_base<EllipticalKnot>;
    using map_type = typename sgm::map_t<int, edge_type>;

private:
    int m_id;
    map_type m_label_to_edge;
    std::vector<node_type> m_knots;

public:
    explicit Segment(int id);
    int id() const;
    map_type &label_to_edge();
    const map_type &label_to_edge() const;
    std::vector<node_type> &knots();
    const std::vector<node_type> &knots() const;
    void add_node(int label, const node_type& knot);
};

Segment read_segmented_test_board(const std::string &file);
}
}

#endif //SGMWSMCPP_KNOTDATAREADERCONFIG_H

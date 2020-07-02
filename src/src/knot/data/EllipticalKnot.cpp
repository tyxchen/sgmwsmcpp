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

#include "knot/data/EllipticalKnot.h"

namespace sgm
{
EllipticalKnot::EllipticalKnot(int m_pidx, int m_idx,
                               double x, double y, double z, double n,
                               double var_x, double var_y, double cov,
                               int boundary_axis0, int boundary_axis1,
                               double yaxis, double zaxis,
                               double area_over_axis)
    : m_idx { m_idx },
      m_pidx { m_pidx },
      m_node_features {{ "x", x },
                       { "y", y },
                       { "z", z },
                       { "n", n },
                       { "var_x", var_x },
                       { "var_y", var_y },
                       { "cov", cov },
                       { "boundary_axis0", boundary_axis0 },
                       { "boundary_axis1", boundary_axis1 },
                       { "yaxis", yaxis },
                       { "zaxis", zaxis },
                       { "area_over_axis", area_over_axis }} {}

int EllipticalKnot::idx() const {
    return m_idx;
}

int EllipticalKnot::partition_idx() const {
    return m_pidx;
}

const Counter<std::string> &EllipticalKnot::node_features() const {
    return m_node_features;
}

bool EllipticalKnot::operator<(const GraphNode<std::string> &other) const {
    return m_node_features.get("x") < other.node_features().get("x");
}

bool EllipticalKnot::operator>(const GraphNode<std::string> &other) const {
    return m_node_features.get("x") > other.node_features().get("x");
}

std::ostream &operator<<(std::ostream &out, const EllipticalKnot &knot) {
    out << "(" << knot.m_pidx << ", " << knot.m_idx << ")";
    return out;
}

}

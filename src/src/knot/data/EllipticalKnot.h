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

#ifndef SGMWSMC_ELLIPTICALKNOT_H
#define SGMWSMC_ELLIPTICALKNOT_H

#include <iostream>
#include <string>

#include "utils/container/Counter.h"
#include "knot/data/Knot.h"

namespace sgm
{

class EllipticalKnot : public Knot<std::string>
{
    int m_idx;
    int m_pidx;
    Counter<std::string> m_node_features;

public:
    EllipticalKnot(int m_pidx, int m_idx,
                   double x, double y, double z, double n,
                   double var_x, double var_y, double cov,
                   int boundary_axis0, int boundary_axis1,
                   double yaxis, double zaxis,
                   double area_over_axis);

    int idx() const override;

    int partition_idx() const override;

    const Counter<std::string> &node_features() const override;

    bool operator<(const GraphNode<std::string> &other) const override;

    bool operator>(const GraphNode<std::string> &other) const override;

    friend std::ostream &operator<<(std::ostream &out, const EllipticalKnot &knot);
};

}

#endif //SGMWSMC_ELLIPTICALKNOT_H

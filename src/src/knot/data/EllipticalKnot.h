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

#include "utils/types.h"
#include "utils/hash.h"
#include "utils/container/Counter.h"
#include "knot/data/Knot.h"

namespace sgm
{

namespace EllipticalKnotFeatureNames
{

// For speed, we provide a custom hash.
static constexpr string_t X { "x", 1 };
static constexpr string_t Y { "y", 2 };
static constexpr string_t Z { "z", 3 };
static constexpr string_t N { "n", 4 };
static constexpr string_t VAR_X { "var_x", 5 };
static constexpr string_t VAR_Y { "var_y", 6 };
static constexpr string_t COV { "cov", 7 };
static constexpr string_t BOUNDARY_AXIS0 { "boundary_axis0", 8 };
static constexpr string_t BOUNDARY_AXIS1 { "boundary_axis1", 9 };
static constexpr string_t YAXIS { "yaxis", 10 };
static constexpr string_t ZAXIS { "zaxis", 11 };
static constexpr string_t AREA_OVER_AXIS { "area_over_axis", 12 };

}

class EllipticalKnot : public Knot<string_t>
{
    int m_idx;
    int m_pidx;
    Counter<string_t> m_node_features;

public:
    EllipticalKnot(int m_pidx, int m_idx,
                   double x, double y, double z, double n,
                   double var_x, double var_y, double cov,
                   int boundary_axis0, int boundary_axis1,
                   double yaxis, double zaxis,
                   double area_over_axis);

    int idx() const override;

    int pidx() const override;

    const Counter<string_t> &node_features() const override;

    bool operator<(const GraphNode<string_t> &other) const override;

    bool operator>(const GraphNode<string_t> &other) const override;

    bool operator==(const GraphNode<string_t> &other) const override;

    friend std::ostream &operator<<(std::ostream &out, const EllipticalKnot &knot);

    friend struct hash<EllipticalKnot>;
};

template <>
struct hash<EllipticalKnot>
{
    size_t operator()(const EllipticalKnot &obj) const noexcept {
        // Returns a 10-bit hash, where the upper 2 bits are the partition index and the lower 8 bits are the index
        return (static_cast<size_t>(obj.m_pidx) << 8u) | (static_cast<size_t>(obj.m_idx) & 0xFFul);
    }
};

}

#endif //SGMWSMC_ELLIPTICALKNOT_H

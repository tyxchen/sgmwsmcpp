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

// For speed, custom hashing is done with the first character.
static constexpr const char* X = "0_x";
static constexpr const char* Y = "1_y";
static constexpr const char* Z = "2_z";
static constexpr const char* N = "3_n";
static constexpr const char* VAR_X = "4_var_x";
static constexpr const char* VAR_Y = "5_var_y";
static constexpr const char* COV = "6_cov";
static constexpr const char* BOUNDARY_AXIS0 = "7_boundary_axis0";
static constexpr const char* BOUNDARY_AXIS1 = "8_boundary_axis1";
static constexpr const char* YAXIS = "9_yaxis";
static constexpr const char* ZAXIS = "a_zaxis";
static constexpr const char* AREA_OVER_AXIS = "b_area_over_axis";

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

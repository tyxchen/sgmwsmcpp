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

class EllipticalKnot : public GraphNode
{
    int m_idx;
    int m_pidx;
    double m_x;
    double m_y;
    double m_z;
    double m_n;
    double m_var_x;
    double m_var_y;
    double m_cov;
    int m_boundary_axis0;
    int m_boundary_axis1;
    double m_yaxis;
    double m_zaxis;
    double m_area_over_axis;

public:
    EllipticalKnot(int m_pidx, int m_idx,
                   double x, double y, double z, double n,
                   double var_x, double var_y, double cov,
                   int boundary_axis0, int boundary_axis1,
                   double yaxis, double zaxis,
                   double area_over_axis);

    int idx() const override;

    int pidx() const override;

    double x() const override;

    inline double y() const { return m_y; }

    inline double z() const { return m_z; }

    inline double n() const { return m_n; }

    inline double var_x() const { return m_var_x; }

    inline double var_y() const { return m_var_y; }

    inline double cov() const { return m_cov; }

    inline int boundary_axis0() const { return m_boundary_axis0; }

    inline int boundary_axis1() const { return m_boundary_axis1; }

    inline double yaxis() const { return m_yaxis; }

    inline double zaxis() const { return m_zaxis; }

    inline double area_over_axis() const { return m_area_over_axis; }

    bool operator<(const GraphNode &other) const override;

    bool operator>(const GraphNode &other) const override;

    bool operator==(const GraphNode &other) const override;

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

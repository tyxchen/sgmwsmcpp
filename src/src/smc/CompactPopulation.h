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

#ifndef SGMWSMCPP_COMPACTPOPULATION_H
#define SGMWSMCPP_COMPACTPOPULATION_H

#include <cstddef>
#include <limits>

namespace sgm
{
namespace smc
{

class CompactPopulation
{
    size_t m_num_particles = 0;
    double m_log_sum = -std::numeric_limits<double>::infinity();
    double m_log_sum_of_squares = -std::numeric_limits<double>::infinity();

public:
    size_t num_particles() const;

    double log_sum() const;

    double log_sum_of_squares() const;

    void insert_log_weight(double log_weight);

    double ess() const;

    double logZ_estimate() const;

    void clear();
};

}
}

#endif //SGMWSMCPP_COMPACTPOPULATION_H

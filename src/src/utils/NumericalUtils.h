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

#ifndef SGMWSMCPP_NUMERICALUTILS_H
#define SGMWSMCPP_NUMERICALUTILS_H

#include <cmath>
#include <limits>

namespace sgm
{
namespace NumericalUtils
{

double log_add(double x, double y);

template <typename ForwardIterator>
double log_add(ForwardIterator begin, ForwardIterator end) {
    auto max = -std::numeric_limits<double>::infinity();
    auto max_index = 0;
    auto it = begin;
    auto i = 0;
    for (; it != end; ++it, ++i) {
        if (*it > max) {
            max = *it;
            max_index = i;
        }
    }
    if (max == -std::numeric_limits<double>::infinity()) return max;
    // compute the negative difference
    auto threshold = max - 20;
    auto sum_negative_diffs = 0.0;
    for (it = begin, i = 0; it != end; ++it, ++i) {
        if (i != max_index && *it > threshold) {
            sum_negative_diffs += std::exp(*it - max);
        }
    }
    if (sum_negative_diffs > 0.0) {
        return max + std::log(1.0 + sum_negative_diffs);
    } else {
        return max;
    }
}

}
}
#endif //SGMWSMCPP_NUMERICALUTILS_H

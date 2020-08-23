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

#include "utils/NumericalUtils.h"

#include <utility>

using namespace sgm;

double NumericalUtils::log_add(double x, double y) {
    // make x the max
    if (y > x) {
        std::swap(x, y);
    }
    // now x is bigger
    if (x == Consts::NEGATIVE_INFINITY) {
        return x;
    }
    auto neg_diff = y - x;
    if (neg_diff < -20) {
        return x;
    }
    return x + std::log(1.0 + std::exp(neg_diff));
}


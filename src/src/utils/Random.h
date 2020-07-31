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

#ifndef SGMWSMCPP_RANDOM_H
#define SGMWSMCPP_RANDOM_H

#include <random>

namespace sgm
{
class Random
{
    std::minstd_rand m_rng;
public:
    explicit Random(int seed);
    const std::minstd_rand &rng() const;
    std::minstd_rand &rng();
    std::minstd_rand::result_type next(size_t bits);
    std::minstd_rand::result_type operator()();
    int next_int(int bound);
    double next_double();
};
}


#endif //SGMWSMCPP_RANDOM_H

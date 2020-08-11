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
public:
    using device = std::minstd_rand;
    using result_type = device::result_type;

private:
    device m_rng;

public:
    explicit Random(int seed);
    const device &rng() const;
    device &rng();
    result_type next(size_t bits);
    result_type operator()();
    int next_int(int bound);
    double next_double();

    static constexpr result_type min() {
        return Random::device::min();
    }

    static constexpr result_type max() {
        return Random::device::max();
    }

#ifndef NDEBUG
private:
    size_t m_num_calls = 0;

public:
    size_t num_calls() const;
#endif
};
}


#endif //SGMWSMCPP_RANDOM_H

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

#include <cstdint>
#include <cstddef>

namespace sgm
{
class Random
{
public:
    using seed_type = unsigned long long;
    using result_type = std::uint32_t;

    static constexpr seed_type multiplier = 0x5DEECE66Dull;
    static constexpr seed_type increment = 0xBull;
    static constexpr seed_type modulus = 1ull << 48u;

private:
    static constexpr seed_type modulus_mask = modulus - 1;

    seed_type m_state;

public:
    explicit Random(seed_type seed);
    result_type next(std::uint32_t bits);
    result_type operator()();
    result_type next_int(int bound);
    long double next_double();

    static constexpr result_type min() {
        return 0;
    }

    static constexpr result_type max() {
        return 0xFFFFFFFFull;
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

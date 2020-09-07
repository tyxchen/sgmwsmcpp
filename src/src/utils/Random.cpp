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

#include "Random.h"

using namespace sgm;


Random::Random(seed_type seed) : m_state((seed ^ multiplier) & modulus_mask) {}

Random::result_type Random::next(std::uint32_t bits) {
#ifndef NDEBUG
    ++m_num_calls;
#endif
    m_state = (m_state * multiplier + increment) & modulus_mask;
    return static_cast<result_type>(m_state >> (48 - bits));
}

Random::result_type Random::operator()() {
#ifndef NDEBUG
    ++m_num_calls;
#endif
    return next(32);
}

Random::result_type Random::next_int(int bound) {
    // Use the "Java algorithm" to generate bounded random integers from an unbounded result
    auto r = next(31);
    auto m = static_cast<seed_type>(bound - 1);
    if ((static_cast<seed_type>(bound) & m) == 0)  // bound is a power of 2, so we can simply bitmask the lower bits
        r = static_cast<int>((static_cast<seed_type>(bound) * static_cast<seed_type>(r)) >> 31u);
    else {
        for (int u = r; u - (r = u % bound) + m < 0; u = next(31));
    }
    return r;
}

long double Random::next_double() {
    // Again uses the "Java algorithm" to generate random doubles
    return ((static_cast<seed_type>(next(26)) << 27u) + next(27)) / static_cast<long double>(1ull << 53u);
}

#ifndef NDEBUG
size_t Random::num_calls() const { return m_num_calls; }
#endif

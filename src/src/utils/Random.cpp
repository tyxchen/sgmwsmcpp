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

#include <boost/random/uniform_smallint.hpp>

using namespace sgm;


Random::Random(int seed) : m_rng(seed) {}

const Random::device & Random::rng() const {
    return m_rng;
}

Random::device & Random::rng() {
    return m_rng;
}

Random::result_type Random::next(size_t bits) {
#ifndef NDEBUG
    ++m_num_calls;
#endif
    return m_rng() >> (32 - bits);
}

Random::result_type Random::operator()() {
#ifndef NDEBUG
    ++m_num_calls;
#endif
    return m_rng();
}

int Random::next_int(int bound) {
   /* // Use the "Java algorithm" to generate bounded random integers from an unbounded result
    auto r = next(31);
    auto m = bound - 1;
    if ((bound & m) == 0)  // bound is a power of 2, so we can simply bitmask the lower bits
        r = static_cast<int>(bound * static_cast<long long>(r) >> 31);
    else {
        for (int u = r; u - (r = u % bound) + m < 0; u = next(31));
    }
    return r;*/
   boost::uniform_smallint<int> rng(0, bound - 1);
   return rng(m_rng);
}

double Random::next_double() {
    // Again uses the "Java algorithm" to generate random doubles
//    return ((static_cast<long long>(next(26)) << 27) + next(27)) / static_cast<double>(1ULL << 53);
    return m_01rng(m_rng);
}

#ifndef NDEBUG
size_t Random::num_calls() const { return m_num_calls; }
#endif

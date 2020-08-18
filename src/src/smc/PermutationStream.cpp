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

#include "PermutationStream.h"

#include <algorithm>
#include <numeric>

using namespace sgm;

smc::PermutationStream::PermutationStream(int size, const Random &random)
    : m_size(size), m_indices(size), m_random(random) {
    std::iota(m_indices.begin(), m_indices.end(), 0);
}

void smc::PermutationStream::reshuffle() {
    std::shuffle(m_indices.begin(), m_indices.end(), m_random);
}

int smc::PermutationStream::index() {
    auto idx = m_num_calls++ % m_size;
    if (idx == 0) {
        reshuffle();
    }
    return m_indices[idx];
}

int smc::PermutationStream::size() const {
    return m_size;
}

int smc::PermutationStream::num_calls() const {
    return m_num_calls;
}

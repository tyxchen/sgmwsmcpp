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

#include "catch.hpp"
#include "utils/container/Counter.h"
#include <iostream>

struct C {
    int key;

    explicit C (int key) : key { key } {}
    C (const C &other) : key { other.key } {
        std::cerr<< "copy cc" << std::endl;
    }
};

std::ostream &operator<<(std::ostream &out, const C &c) {
    out << c.key;
    return out;
}

TEST_CASE("counter operations on a hashable type") {
    sgm::Counter<int> counter;

    SECTION("can store") {
        counter.set(5, 0);
        REQUIRE(counter.size() == 1);
        REQUIRE(counter.get(5) == 0);
    }
}

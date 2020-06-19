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

#include <string>
#include <utility>
#include <boost/container_hash/hash.hpp>
#include "catch.hpp"
#include "utils/type_traits.h"


struct C {
    int key;
};

std::size_t hash_value(const C &c) {
    boost::hash<int> hasher;
    return hasher(c.key);
}

TEST_CASE("is_hashable") {
    SECTION("with standard types") {
        REQUIRE(sgm::is_hashable_v<int>);
        REQUIRE(sgm::is_hashable_v<std::string>);
    }

    SECTION("with types hashable via boost::hash") {
        REQUIRE(sgm::is_hashable_v<std::pair<int, int>>);
    }

    SECTION("using custom hash") {
        REQUIRE(sgm::is_hashable_v<C>);
    }
}

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

#include <catch.hpp>
#include <memory>

#include "utils/container/vector_list.h"

struct A
{
    int x, y;
    A(int _x, int _y) : x(_x), y(_y) {}
};

struct B : A
{
    B(int x, int y) : A(x, y) {}
    B(const B &other) : A(0, 1) {
        throw 1;
    }
    B(B &&other) : A(0, 1) {
        throw 2;
    }
};

TEST_CASE("basic functionality with primitive types") {
    sgm::vector_list<int> vl;

    SECTION("empty vector has size 0") {
        REQUIRE(vl.size() == 0);
    }

    SECTION("vector push/emplace updates vector") {
        vl.push_back(1);
        vl.emplace_back(3);
        vl.insert(1, 2);

        REQUIRE(vl.size() == 3);
        REQUIRE(vl.capacity() > 2);

        vl.insert(0, 0);

        REQUIRE(vl.size() == 4);

        for (auto i = 0; i < 4; ++i) {
            REQUIRE(vl[i] == i);
        }
    }

    SECTION("removal") {
        vl.push_back(1);
        vl.push_back(2);
        vl.push_back(3);

        vl.pop_back();
        REQUIRE(vl.size() == 2);
        vl.erase(0);
        REQUIRE(vl.size() == 1);
        vl.pop_back();
        REQUIRE(vl.empty());
    }

    SECTION("iterators") {
        vl.push_back(1);
        vl.push_back(2);
        vl.push_back(3);
        auto i = 1;
        for (auto &j : vl) {
            REQUIRE(i == j);
            ++i;
        }
    }

    SECTION("accessors") {
        vl.push_back(1);
        vl.push_back(2);
        vl.push_back(3);

        REQUIRE(vl.front() == 1);
        REQUIRE(vl.back() == 3);
    }

    SECTION("copy constructor/assignment") {
        vl.push_back(1);
        vl.push_back(2);
        vl.push_back(3);

        // universal assignment means that this does copy-and-swap
        auto vll = vl;

        REQUIRE(vll.size() == vl.size());
        REQUIRE(vll.capacity() == vl.capacity());

        for (auto s = vl.size(), i = 0ul; i < s; ++i) {
            REQUIRE(vl[i] == vll[i]);
        }
    }

    SECTION("move constructor/assignment") {
        vl.push_back(1);
        vl.push_back(2);
        vl.push_back(3);

        auto _s = vl.size();
        auto _c = vl.capacity();
        // universal assignment means that this does move-and-swap
        auto vll = std::move(vl);

        REQUIRE(vll.size() == _s);
        REQUIRE(vll.capacity() == _c);

        for (auto s = vll.size(), i = 0ul; i < s; ++i) {
            REQUIRE(vll[i] == i + 1);
        }
    }
}

TEST_CASE("basic functionality with classes") {
    sgm::vector_list<A> vl;

    SECTION("on push") {
        vl.push_back(A(1, 2));
        vl.emplace_back(3, 4);
        vl.insert(1, A(2, 3));
        vl.emplace(0, 0, 1);

        REQUIRE(vl.size() == 4);
    }

    SECTION("on erase") {
        vl.emplace_back(1, 2);
        vl.emplace_back(2, 3);
        vl.emplace_back(3, 4);

        vl.pop_back();
        vl.erase(0);

        REQUIRE(vl.size() == 1);
    }

    SECTION("accessors") {
        vl.emplace_back(1, 2);
        vl.emplace_back(2, 3);
        vl.emplace_back(3, 4);

        REQUIRE(vl[0].x == 1);
        REQUIRE(vl[0].y == 2);
        REQUIRE(vl[1].x == 2);
        REQUIRE(vl[1].y == 3);
        REQUIRE(vl[2].x == 3);
        REQUIRE(vl[2].y == 4);
    }

    SECTION("copy constructor/assignment") {
        vl.emplace_back(1, 2);
        vl.emplace_back(2, 3);
        vl.emplace_back(3, 4);

        // universal assignment means that this does copy-and-swap, if no RVO is done
        auto vll = vl;

        REQUIRE(vll.size() == vl.size());
        REQUIRE(vll.capacity() == vl.capacity());

        for (auto s = vl.size(), i = 0ul; i < s; ++i) {
            REQUIRE(vl[i].x == vll[i].x);
            REQUIRE(vl[i].y == vll[i].y);
        }
    }

    SECTION("move constructor/assignment") {
        vl.emplace_back(1, 2);
        vl.emplace_back(2, 3);
        vl.emplace_back(3, 4);

        auto _s = vl.size();
        auto _c = vl.capacity();
        // universal assignment means that this does move-and-swap, if no RVO is done
        auto vll = std::move(vl);

        REQUIRE(vll.size() == _s);
        REQUIRE(vll.capacity() == _c);

        for (auto s = vll.size(), i = 0ul; i < s; ++i) {
            REQUIRE(vll[i].x == i + 1);
            REQUIRE(vll[i].y == i + 2);
        }
    }
}

TEST_CASE("vector_list doesn't leak memory") {
    sgm::vector_list<std::shared_ptr<int>> vl;
    auto p1 = std::make_shared<int>(1);
    auto p2 = std::make_shared<int>(2);

    SECTION("on push") {
        vl.push_back(p1);
        vl.emplace_back(p2);
        vl.emplace_back(std::make_shared<int>(3));
        auto p3 = std::move(vl.back());

        REQUIRE(p1.use_count() == 2);
        REQUIRE(p2.use_count() == 2);
        REQUIRE(p3.use_count() == 1);
    }

    SECTION("on erase") {
        vl.emplace_back(p1);
        vl.emplace_back(p2);

        vl.pop_back();
        vl.erase(0);

        REQUIRE(p1.use_count() == 1);
        REQUIRE(p2.use_count() == 1);
    }
    SECTION("throwing constructor") {
        sgm::vector_list<B> vll, vlll;
        vll.emplace_back(0, 1);
        vll.emplace_back(0, 1);
        vll.emplace_back(0, 1);

        try {
            vlll = vll;
            REQUIRE(1 == 0);
        } catch (...) {}
    }
}

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

#ifndef SGMWSMCPP_TYPES_HASH_H
#define SGMWSMCPP_TYPES_HASH_H

#include <functional>
#include <iostream>
#include "utils/hash.h"
#include "types_base.h"

namespace sgm
{

template <typename T>
struct hash<edge_t<T>>
{
    static constexpr unsigned long long a = 0x1234321ull;
    static constexpr unsigned long long m = (1ull << 32u) - 1;
    hash<T> hasher;

    size_t operator()(const edge_t<T> &obj) const noexcept {
        auto hash_v = 0ull;

        // definitely not perfect, but it's fast
        for (auto &thing : obj) {
            hash_v = (a * hash_v + hasher(thing)) & m;
        }

        return hash_v;
    }
};

//template <typename K, typename T>
//struct hash<map_t<K, T>>
//{
//    hash<K> key_hasher;
//    hash<T> val_hasher;
//    size_t operator()(const map_t<K, T> &obj) const noexcept {
//        auto hash_v = 0l;
//
//        // definitely not perfect, can be improved
//        for (auto &thing : obj) {
//            hash_v ^= key_hasher(thing.first) + val_hasher(thing.second);
//        }
//
//        return hash_v;
//    }
//};

}

#endif //SGMWSMCPP_TYPES_HASH_H

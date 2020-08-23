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
    hash<T> hasher;
    size_t operator()(const edge_t<T> &obj) const noexcept {
        auto hash_v = 0l;

        // definitely not perfect, can be improved
        for (auto &thing : obj) {
            hash_v += hasher(thing);
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

/**
 * Hash the FIRST character of a const char *.
 */
template <>
struct hash<string_t>
{
public:
    size_t operator()(const string_t &str) const noexcept {
        // WARNING: We are assuming that the given string is null-terminated.
        // This is OK as we ONLY use null-terminated compile-time strings, but BEWARE when extending code.
        return str[0];
    }
};

}

#endif //SGMWSMCPP_TYPES_HASH_H

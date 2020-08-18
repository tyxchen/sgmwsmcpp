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

#ifndef SGMWSMCPP_HASH_H
#define SGMWSMCPP_HASH_H

#include <functional>
#include <memory>

namespace sgm
{

template <typename T>
struct hash;

template <>
struct hash<int>
{
    std::hash<int> hasher;
    size_t operator()(int obj) const noexcept {
        return hasher(obj);
    }
};

template <>
struct hash<double>
{
    std::hash<double> hasher;
    size_t operator()(double obj) const noexcept {
        return hasher(obj);
    }
};

template <typename T>
struct hash<std::shared_ptr<T>>
{
    hash<T> hasher;
    size_t operator()(const std::shared_ptr<T> &obj) const noexcept {
        return hasher(*obj);
    }
};

}

#endif //SGMWSMCPP_HASH_H

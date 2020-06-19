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

#ifndef SGMWSMCPP_TYPE_TRAITS_H
#define SGMWSMCPP_TYPE_TRAITS_H

#include <functional>
#include <boost/container_hash/hash_fwd.hpp>
#include <type_traits>

namespace sgm
{

/**
 * Determines if a type is hashable with boost::hash's default hashers
 */
template<typename T, typename = std::void_t<>>
struct is_boost_hashable : std::false_type
{
};

template<typename T>
struct is_boost_hashable<T, std::void_t<decltype(boost::hash_value(std::declval<T>()))>>
    : std::true_type
{
};

template<typename T>
constexpr bool is_boost_hashable_v = is_boost_hashable<T>::value;

/**
 * Determines if a type is hashable with boost::hash with a custom hasher
 */
template<typename T, typename = std::void_t<>>
struct is_own_hashable : std::false_type
{
};

template<typename T>
struct is_own_hashable<T, std::void_t<decltype(hash_value(std::declval<T>()))>>
    : std::true_type
{
};

template<typename T>
constexpr bool is_own_hashable_v = is_own_hashable<T>::value;

template<typename T>
constexpr bool is_hashable_v = is_boost_hashable_v<T> || is_own_hashable_v<T> ;
}

#endif //SGMWSMCPP_TYPE_TRAITS_H

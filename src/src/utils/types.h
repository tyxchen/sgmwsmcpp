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

#ifndef SGMWSMCPP_TYPES_H
#define SGMWSMCPP_TYPES_H

#include <memory>

#ifdef DEBUG
#include <unordered_set>
#include <unordered_map>
#else
#include <parallel_hashmap/phmap.h>
#endif

namespace sgm
{
template <typename T>
#ifdef DEBUG
using set_t = typename std::unordered_set<T>;
#else
using set_t = typename phmap::flat_hash_set<T>;
#endif

template <typename K, typename T>
#ifdef DEBUG
using map_t = typename std::unordered_map<K, T>;
#else
using map_t = typename phmap::flat_hash_map<K, T>;
#endif

template <typename NodeType>
using node_type_base = typename std::shared_ptr<NodeType>;

template <typename NodeType>
using edge_type_base = typename std::shared_ptr<set_t<node_type_base<NodeType>>>;
}
#endif //SGMWSMCPP_TYPES_H

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

#ifndef SGMWSMCPP_TYPES_BASE_H
#define SGMWSMCPP_TYPES_BASE_H

#include <functional>
#include <memory>

#include <parallel_hashmap/phmap.h>

#include "utils/hash.h"
#include "utils/conststr.h"

namespace sgm
{

template <typename T>
using set_t = typename phmap::parallel_flat_hash_set<T, hash<T>, std::equal_to<T>>;

template <typename K, typename T>
using map_t = typename phmap::parallel_flat_hash_map<K, T, hash<K>, std::equal_to<K>>;

template <typename T>
using edge_t = typename phmap::flat_hash_set<T, hash<T>, std::equal_to<T>>;

using string_t = conststr;

template <typename NodeType>
using node_type_base = typename std::shared_ptr<NodeType>;

template <typename NodeType>
using edge_type_base = typename std::shared_ptr<edge_t<node_type_base<NodeType>>>;

}

#endif //SGMWSMCPP_TYPES_BASE_H

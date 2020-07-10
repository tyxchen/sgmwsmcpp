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
#include <parallel_hashmap/phmap_fwd_decl.h>

namespace sgm
{
template <typename NodeType>
using node_type_base = typename std::shared_ptr<NodeType>;

template <typename NodeType>
using edge_type_base = typename std::shared_ptr<phmap::flat_hash_set<node_type_base<NodeType>>>;
}
#endif //SGMWSMCPP_TYPES_H

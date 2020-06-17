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

#ifndef SGMWSMC_GRAPHNODE_H
#define SGMWSMC_GRAPHNODE_H

#include "utils/container/Counter.h"

namespace sgm
{
template <typename F>
class GraphNode
{
public:
    virtual int idx() const = 0;
    virtual int partition_idx() const = 0;
    virtual const Counter<F> &node_features() const = 0;

    virtual bool operator<(const GraphNode<F> &other) const = 0;
    virtual bool operator>(const GraphNode<F> &other) const = 0;

    virtual ~GraphNode() = default;
};
}

#endif //SGMWSMC_GRAPHNODE_H

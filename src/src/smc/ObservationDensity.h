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

#ifndef SGMWSMCPP_OBSERVATIONDENSITY_H
#define SGMWSMCPP_OBSERVATIONDENSITY_H

#include "utils/types.h"
#include "common/graph/GraphMatchingState_fwd.h"

namespace sgm
{

namespace smc
{

template <typename F, typename NodeType>
struct ObservationDensity
{
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;

    double log_density(const GraphMatchingState<F, NodeType> &latent, const node_type &emission) {
        return _log_density(latent, emission);
    }

    double log_weight_correction(const GraphMatchingState<F, NodeType> &cur_latent,
                                 const GraphMatchingState<F, NodeType> &old_latent) {
        return _log_weight_correction(cur_latent, old_latent);
    }

    virtual ~ObservationDensity() = default;

private:
    virtual double _log_density(const GraphMatchingState<F, NodeType> &latent, const node_type &emission) = 0;

    virtual double _log_weight_correction(const GraphMatchingState<F, NodeType> &cur_latent,
                                          const GraphMatchingState<F, NodeType> &old_latent) = 0;
};

}

}

#endif //SGMWSMCPP_OBSERVATIONDENSITY_H

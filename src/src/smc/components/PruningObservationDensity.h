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

#ifndef SGMWSMCPP_PRUNINGOBSERVATIONDENSITY_H
#define SGMWSMCPP_PRUNINGOBSERVATIONDENSITY_H

#include <limits>
#include <vector>
#include <parallel_hashmap/phmap.h>

#include "utils/types.h"
#include "common/graph/GraphMatchingState.h"

namespace sgm
{

namespace smc
{

namespace detail
{
template <typename State>
bool in_support(const State &subset, const State &superset) {
    for (const auto &v : subset) {
        for (const auto &w : *v.second) {
            if (!superset.at(v.first)->count(w)) {
                return false;
            }
        }
    }
    return true;
}
}

template <typename F, typename NodeType>
class PruningObservationDensity
{
public:
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;

private:
    phmap::flat_hash_map<node_type, edge_type> m_target_state;

public:
    explicit PruningObservationDensity(const std::vector<edge_type> &matching) {
        for (const auto &edge : matching) {
            for (const auto &v : *edge) {
                m_target_state.emplace(v, edge);
            }
        }
    }

    double log_density(const GraphMatchingState<F, NodeType> &latent, const node_type &emission) {
        if (!detail::in_support(latent.node_to_edge_view(), m_target_state)) {
            return -std::numeric_limits<double>::infinity();
        }
        return 0;
    }

    double log_weight_correction(const GraphMatchingState<F, NodeType> &cur_latent,
        const GraphMatchingState<F, NodeType> &old_latent) {
        /*
        auto num_parents = command.get_decision_model().num_parents(cur_latent);
        return -std::log(num_parents) - cur_latent.log_forward_proposal();
        */
        //return -cur_latent.log_forward_proposal();
        return 0;
    }
};

}
}

#endif //SGMWSMCPP_PRUNINGOBSERVATIONDENSITY_H

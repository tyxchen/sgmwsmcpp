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

#include "utils/debug.h"
#include "utils/types.h"
#include "smc/ObservationDensity.h"
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
        auto superset_ref = superset.at(v.first);
        for (const auto &w : *v.second) {
            if (!superset_ref->count(w)) {
                return false;
            }
        }
    }
    return true;
}
}

template <typename F, typename NodeType>
class PruningObservationDensity : public ObservationDensity<F, NodeType>
{
public:
    using node_type = typename ObservationDensity<F, NodeType>::node_type;
    using edge_type = typename ObservationDensity<F, NodeType>::edge_type;

private:
    map_t<node_type, edge_type> m_target_state;

public:
    explicit PruningObservationDensity(const std::vector<edge_type> &matching) {
        for (const auto &edge : matching) {
            for (const auto &v : *edge) {
                m_target_state.emplace(v, edge);
            }
        }
//        sgm::logger << "ObservationDensity::m_target_state\n" << m_target_state << "\n------------\n";
    }

    double _log_density(const GraphMatchingState<F, NodeType> &latent, const node_type &emission) override {
        if (!detail::in_support(latent.node_to_edge_view(), m_target_state)) {
            return -std::numeric_limits<double>::infinity();
        }
        return 0;
    }

    double _log_weight_correction(const GraphMatchingState<F, NodeType> &cur_latent,
                                  const GraphMatchingState<F, NodeType> &old_latent) override {
        /*
        auto num_parents = cur_latent.num_parents();
        return -std::log(num_parents) - cur_latent.log_forward_proposal();
        */
        //return -cur_latent.log_forward_proposal();
        return 0;
    }
};

}
}

#endif //SGMWSMCPP_PRUNINGOBSERVATIONDENSITY_H

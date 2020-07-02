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

#ifndef SGMWSMCPP_DECISIONMODEL_H
#define SGMWSMCPP_DECISIONMODEL_H

#include <memory>
#include <vector>
#include <parallel_hashmap/phmap.h>

#include "common/graph/GraphMatchingState.h"

namespace sgm
{
template <typename F, typename NodeType>
class DecisionModel
{
public:
    using node_type = typename std::shared_ptr<NodeType>;

    std::vector<phmap::flat_hash_set<node_type>> get_decisions(const node_type &node,
                                                               const GraphMatchingState<F, NodeType>
                                                               &state) {
        return _get_decisions(node, state);
    }

    std::vector<phmap::flat_hash_set<node_type>> get_decisions(const node_type &node,
                                                               const std::vector<node_type> &candidate_nodes,
                                                               const phmap::flat_hash_set<node_type> &covered_nodes,
                                                               const std::vector<
                                                                   phmap::flat_hash_set<node_type>> &matching,
                                                               const phmap::flat_hash_map<node_type,
                                                                                          phmap::flat_hash_set<
                                                                                              node_type>> &node_to_edge) {
        return _get_decisions(node, candidate_nodes, covered_nodes, matching, node_to_edge);
    }

    bool in_support(const GraphMatchingState<F, NodeType> &state) {
        return _in_support(state);
    }

    bool path_exists(const GraphMatchingState<F, NodeType> &cur_state,
                     const phmap::flat_hash_map<node_type, phmap::flat_hash_set<node_type>> &final_state) {
        return _path_exists(cur_state, final_state);
    }

    int num_parents(const GraphMatchingState<F, NodeType> &cur_latent) {
        return _num_parents(cur_latent);
    }

protected:
    virtual std::vector<phmap::flat_hash_set<node_type>> _get_decisions(const node_type &node,
                                                                        const GraphMatchingState<F, NodeType>
                                                                        &state) = 0;

    virtual std::vector<phmap::flat_hash_set<node_type>> _get_decisions(const node_type &node,
                                                                        const std::vector<node_type> &candidate_nodes,
                                                                        const phmap::flat_hash_set<node_type>
                                                                        &covered_nodes,
                                                                        const std::vector<
                                                                            phmap::flat_hash_set<node_type>> &matching,
                                                                        const phmap::flat_hash_map<node_type,
                                                                                                   phmap::flat_hash_set<
                                                                                                       node_type>>
                                                                        &node_to_edge) = 0;

    virtual bool _in_support(const GraphMatchingState<F, NodeType> &state) = 0;

    virtual bool _path_exists(const GraphMatchingState<F, NodeType> &cur_state,
                              const phmap::flat_hash_map<node_type, phmap::flat_hash_set<node_type>> &final_state) = 0;

    virtual int _num_parents(const GraphMatchingState<F, NodeType> &cur_latent) = 0;
};
}

#endif //SGMWSMCPP_DECISIONMODEL_H

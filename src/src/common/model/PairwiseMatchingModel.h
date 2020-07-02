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

#ifndef SGMWSMCPP_PAIRWISEMATCHINGMODEL_H
#define SGMWSMCPP_PAIRWISEMATCHINGMODEL_H

#include <memory>
#include <utility>
#include <parallel_hashmap/phmap.h>

#include "common/model/DecisionModel.h"

namespace sgm
{

namespace
{
template <typename NodeType>
bool is_candidate_edge(const NodeType &node, const phmap::flat_hash_set<NodeType> &edge) {
    for (const auto &v : edge) {
        if (v->partition_idx() == node->partition_idx()) {
            return false;
        }
    }
    return true;
}
}

template <typename F, typename NodeType>
class PairwiseMatchingModel : public DecisionModel<F, NodeType>
{
public:
    using node_type = typename DecisionModel<F, NodeType>::node_type;
    constexpr static int H_SPAN = 100;

    std::vector<phmap::flat_hash_set<node_type>> _get_decisions(const node_type &node,
                                                                const GraphMatchingState<F, NodeType> &state)
    override {
        return _get_decisions(node, state.unvisited_nodes(), state.covered_nodes(), state.matchings(), state
            .node_to_edge_view());
    }

    std::vector<phmap::flat_hash_set<node_type>> _get_decisions(const node_type &node,
                                                                const std::vector<node_type> &candidate_nodes,
                                                                const phmap::flat_hash_set<node_type> &covered_nodes,
                                                                const std::vector<phmap::flat_hash_set<node_type>>
                                                                &matching,
                                                                const phmap::flat_hash_map<node_type,
                                                                                           phmap::flat_hash_set<
                                                                                               node_type>>
                                                                &node_to_edge) override {
        std::vector<phmap::flat_hash_set<node_type>> decisions;

        if (covered_nodes.count(node)) {
            // if the node is already covered, then the only decision that is available is an empty decision for now
            decisions.emplace_back();
            return decisions;
        } else {
            // this node is not yet covered so consider all existing edges as long as the edge does contain a node from the same partition as the node being considered
            for (const auto &edge : matching) {
                if (edge.size() >= 3) continue;
                if (is_candidate_edge(node, edge)) {
                    decisions.emplace_back(edge);
                }
            }
        }

        for (const auto &other_node : candidate_nodes) {
            if (covered_nodes.count(other_node)) continue;
            if (node->partition_idx() == other_node->partition_idx()) continue;

            decisions.emplace_back(std::initializer_list<node_type>({ other_node }));
        }

        return std::move(decisions);
    }

    // TODO: implement
    bool _in_support(const GraphMatchingState<F, NodeType> &state) override {
        return true;
    }

    bool _path_exists(const GraphMatchingState<F, NodeType> &cur_state,
                      const phmap::flat_hash_map<node_type, phmap::flat_hash_set<node_type>> &final_state)
    override {
        return false;
    }

    int _num_parents(const GraphMatchingState<F, NodeType> &cur_latent) override {
        return 1;
    }
};

}

#endif //SGMWSMCPP_PAIRWISEMATCHINGMODEL_H

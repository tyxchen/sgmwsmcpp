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

#include "utils/types.h"
#include "common/model/DecisionModel.h"

namespace sgm
{

template <typename F, typename NodeType>
class PairwiseMatchingModel : public DecisionModel<F, NodeType>
{
public:
    using node_type = typename DecisionModel<F, NodeType>::node_type;
    using edge_type = typename DecisionModel<F, NodeType>::edge_type;

private:
    std::vector<edge_type> _decisions(const node_type &node,
                                      const GraphMatchingState <F, NodeType> &state)
    override {
        return _decisions(node, state.unvisited_nodes(), state.covered_nodes(), state.matchings(), state
            .node_to_edge_view());
    }

    std::vector<edge_type> _decisions(const node_type &node,
                                      const std::vector<node_type> &candidate_nodes,
                                      const set_t<node_type> &covered_nodes,
                                      const set_t<edge_type> &matching,
                                      const map_t<node_type, edge_type> &node_to_edge) override {
        using edge_element_type = typename edge_type::element_type;
        std::vector<edge_type> decisions;

        if (covered_nodes.count(node)) {
            // if the node is already covered, then the only decision that is available is an empty decision for now
            decisions.emplace_back(std::make_shared<edge_element_type>());
            return decisions;
        } else {
            // this node is not yet covered so consider all existing edges as long as the edge does not contain a node
            // from the same partition as the node being considered
            for (const auto &edge : matching) {
                if (edge->size() >= 3 || edge->size() == 0) continue;
                // the first node in the edge is not on the same partition as our chosen node, and either the edge is a
                // singleton or the second node is also not on the same partition
                else if (node->pidx() != (*edge->begin())->pidx() &&
                         (edge->size() == 1 || node->pidx() != (*(++edge->begin()))->pidx())) {  // loop unrolling
                    decisions.emplace_back(edge);
                }
            }
        }

        for (const auto &other_node : candidate_nodes) {
            if (other_node == nullptr) continue;
            if (covered_nodes.count(other_node)) continue;
            if (node->pidx() == other_node->pidx()) continue;

            // FIXME: Creates completely new edges with completely new hashes every time.
            decisions.emplace_back(
                std::make_shared<edge_element_type>(std::initializer_list<node_type>({ other_node }))
            );
        }

        if (decisions.empty()) {
            decisions.emplace_back(std::make_shared<edge_element_type>());
        }

        return decisions;
    }

//    bool _in_support(const GraphMatchingState <F, NodeType> &state) override {
//        return true;
//    }
//
//    bool _path_exists(const GraphMatchingState <F, NodeType> &cur_state,
//                      const map_t<node_type, set_t<node_type>> &final_state)
//    override {
//        return false;
//    }
//
//    int _num_parents(const GraphMatchingState <F, NodeType> &cur_latent) override {
//        return 1;
//    }
};

}

#endif //SGMWSMCPP_PAIRWISEMATCHINGMODEL_H

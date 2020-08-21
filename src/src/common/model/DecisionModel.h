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

#include "utils/types.h"
#include "utils/container/vector_list.h"
#include "common/graph/GraphMatchingState_fwd.h"

namespace sgm
{

template <typename F, typename NodeType>
std::vector<edge_type_base<NodeType>> generate_decisions(const node_type_base<NodeType> &node,
                                                         const GraphMatchingState<F, NodeType> &state) {
    return generate_decisions(node, state.unvisited_nodes(), state.covered_nodes(), state.matchings(),
                              state.node_to_edge_view());
}

template <typename NodeType>
std::vector<edge_type_base<NodeType>> generate_decisions(const node_type_base<NodeType> &node,
                                                         const vector_list<node_type_base<NodeType>> &candidate_nodes,
                                                         const set_t<node_type_base<NodeType>> &covered_nodes,
                                                         const set_t<edge_type_base<NodeType>> &matching,
                                                         const map_t<node_type_base<NodeType>,
                                                                     edge_type_base<NodeType>> &node_to_edge) {
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;
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
        if (covered_nodes.count(other_node)) continue;
        if (node->pidx() == other_node->pidx()) continue;

        decisions.emplace_back(
            std::make_shared<edge_element_type>(std::initializer_list<node_type>({ other_node }))
        );
    }

    if (decisions.empty()) {
        decisions.emplace_back(std::make_shared<edge_element_type>());
    }

    return decisions;
}

}

#endif //SGMWSMCPP_DECISIONMODEL_H

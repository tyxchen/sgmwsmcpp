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

#ifndef SGMWSMCPP_GRAPHMATCHINGSTATE_H
#define SGMWSMCPP_GRAPHMATCHINGSTATE_H

#include <random>
#include <vector>
#include <parallel_hashmap/phmap.h>

#include "utils/container/Counter.h"
#include "common/model/Command_fwd.h"

namespace sgm
{
template<typename F, typename NodeType>
class GraphMatchingState
{
public:
    using node_type = std::shared_ptr<NodeType>;

private:
    phmap::flat_hash_set<phmap::flat_hash_set<node_type>> m_matching;
    phmap::flat_hash_set<node_type> m_covered_nodes;
    std::vector<node_type> m_visited_nodes;
    std::vector<node_type> m_unvisited_nodes;
    phmap::flat_hash_map<node_type, phmap::flat_hash_set<node_type>> m_node_to_matching;
    std::vector<phmap::flat_hash_set<node_type>> m_decisions;

    double m_log_density = 0.0;
    Counter<F> m_log_gradient;

public:
    explicit GraphMatchingState(std::vector<node_type> nodes) : m_unvisited_nodes(std::move(nodes)) {}

    bool has_next_step() {
    }

//    std::vector<GraphMatchingState<F, NodeType>> execute_move(
//        const sgm::MultinomialLogisticModel<F, NodeType> &model,
//        const sgm::DecisionModel<F, NodeType> &decision_model,
//        const phmap::flat_hash_map<node_type, phmap::flat_hash_set<node_type>> &final_state) {
//    }

    double log_density() const {
    }

    const Counter<F> &log_gradient() const {
    }

    const node_type &example_node() {
    }

//    double sample_next_state(
//        Random random,
//        sgm::Command<F, NodeType> command,
//        bool sequential, bool exact_sampling) {
//    }

    const std::vector<node_type> &unvisited_nodes() const {
    }

    const std::vector<node_type> &visited_nodes() const {
    }

    phmap::flat_hash_set<node_type> visited_nodes_as_set() const {
    }

    const phmap::flat_hash_set<node_type> &covered_nodes() const {
    }

    const std::vector<phmap::flat_hash_set<node_type>> &decisions() const {
    }

    const std::vector<phmap::flat_hash_set<node_type>> &matchings() const {
    }

    bool covers(NodeType node) const {
    }

//    void shuffle_nodes(Random random) {
//    }

    const phmap::flat_hash_map<node_type, phmap::flat_hash_set<node_type>> &node_to_edge_view() const {
    }

//    std::vector<GraphMatchingState<F, NodeType>> generate_descendants(
//        sgm::MultinomialLogisticModel<F, NodeType> model,
//        sgm::DecisionModel<F, NodeType> decision_model, bool sequential) {
//    }

    double log_forward_proposal() const {
    }
};
}

#endif //SGMWSMCPP_GRAPHMATCHINGSTATE_H

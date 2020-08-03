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

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "utils/debug.h"
#include "utils/types.h"
#include "utils/NumericalUtils.h"
#include "utils/Random.h"
#include "utils/container/Counter.h"
#include "common/model/Command_fwd.h"
#include "common/model/DecisionModel_fwd.h"
#include "common/model/MultinomialLogisticModel.h"

namespace sgm
{
template <typename F, typename NodeType>
class GraphMatchingState
{
public:
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;

private:
    set_t<edge_type> m_matchings;
    set_t<node_type> m_covered_nodes;
    std::vector<node_type> m_visited_nodes;
    std::vector<node_type> m_unvisited_nodes;
    map_t<node_type, edge_type> m_node_to_matching;
    std::vector<edge_type> m_decisions;

    double m_log_density = 0.0;
    double m_log_forward_proposal = 0.0;
    Counter<F> m_log_gradient;

    std::pair<double, double> sample_decision(Random &random,
                                              DecisionModel<F, NodeType> &decision_model,
                                              MultinomialLogisticModel<F, NodeType> &model,
                                              const node_type &node,
                                              bool use_exact_sampling) {
        auto decisions = decision_model.decisions(node, *this);
        auto decisions_size = decisions.size();
        auto idx = 0;
        auto log_prob = 0.0;
        auto log_norm = 0.0;

        if (use_exact_sampling) {
            std::vector<double> log_probs(decisions_size);
            for (auto i = 0; i < decisions_size; ++i) {
                log_probs[i] = model.log_prob(node, decisions[i]).first;
            }

            log_norm = NumericalUtils::log_add(log_probs.begin(), log_probs.end());

            // normalize
            auto sum = 0.0;
            auto probs = log_probs;
            auto max_prob = -std::numeric_limits<double>::infinity();
            for (auto prob : probs) {
                max_prob = std::max(max_prob, prob);
            }
            for (auto &prob : probs) {
                prob = std::exp(prob - max_prob);
                sum += prob;
            }
            for (auto &prob : probs) {
                prob /= sum;
            }

            // sample
            auto v = random.next_double();
            sum = 0.0;
            for (auto prob : probs) {
                sum += prob;
                if (v < sum) break;
                ++idx;
            }
            if (idx == probs.size()) {
                throw std::runtime_error("Could not sample.");
            }

            log_prob = log_probs[idx];
            m_log_forward_proposal = log_prob - log_norm;
        } else {
            idx = random.next_int(decisions_size);
            m_log_forward_proposal = -std::log(decisions_size);
            log_prob = model.log_prob(node, decisions[idx]).first;
        }

        // insert the chosen decision
        m_decisions.push_back(decisions[idx]);

        auto edge = decisions[idx];

        if (edge->size() == 0) {
            // this is do nothing decision, which means to either form a singleton or the node already belongs to an edge
            // in the latter case, we need to get the edge that this node belongs to
            if (m_node_to_matching.count(node)) {
                edge = m_node_to_matching[node];
            }
        } else if (m_matchings.count(edge)) {
            auto edge_removed = m_matchings.erase(edge);
            if (!edge_removed)
                throw std::runtime_error("Edge failed to be removed");
        }

        auto new_edge = std::make_shared<typename edge_type::element_type>(*edge);
        new_edge->insert(node);

        m_matchings.insert(new_edge);
        for (const auto &n : *new_edge) {
            m_covered_nodes.insert(n);
            m_node_to_matching[n] = new_edge;
        }

        return std::make_pair(log_prob, log_norm);
    }

public:
    explicit GraphMatchingState(std::vector<node_type> nodes) : m_unvisited_nodes(std::move(nodes)) {}

    bool has_next_step() {
    }

//    std::vector<GraphMatchingState<F, NodeType>> execute_move(
//        const sgm::MultinomialLogisticModel<F, NodeType> &model,
//        const sgm::DecisionModel<F, NodeType> &decision_model,
//        const map_t<node_type, set_t<node_type>> &final_state) {
//    }

    double log_density() const {
    }

    const Counter<F> &log_gradient() const {
    }

    const node_type &example_node() {
    }

    double sample_next_state(Random &random,
                             Command<F, NodeType> &command,
                             bool use_sequential_sampling, bool use_exact_sampling) {
        auto idx = 0;
        if (!use_sequential_sampling) {
            idx = random.next_int(m_unvisited_nodes.size());
        }

        auto node = std::move(m_unvisited_nodes[idx]);
        m_unvisited_nodes.erase(m_unvisited_nodes.begin() + idx);

        auto cur_model = command.current_model();
        auto sample = sample_decision(random, command.decision_model(), cur_model, node, use_exact_sampling);
        auto log_prob = sample.first;
        m_log_density += log_prob - sample.second;
        m_visited_nodes.emplace_back(std::move(node));
        return log_prob - sample.second;
    }

    const std::vector<node_type> &unvisited_nodes() const {
        return m_unvisited_nodes;
    }

    const std::vector<node_type> &visited_nodes() const {
        return m_visited_nodes;
    }

    set_t<node_type> visited_nodes_as_set() const {
    }

    const set_t<node_type> &covered_nodes() const {
        return m_covered_nodes;
    }

    const std::vector<edge_type> &decisions() const {
        return m_decisions;
    }

    const set_t<edge_type> &matchings() const {
        return m_matchings;
    }

    bool covers(NodeType node) const {
    }

//    void shuffle_nodes(Random random) {
//    }

    const map_t<node_type, edge_type> &node_to_edge_view() const {
        return m_node_to_matching;
    }

//    std::vector<GraphMatchingState<F, NodeType>> generate_descendants(
//        sgm::MultinomialLogisticModel<F, NodeType> model,
//        sgm::DecisionModel<F, NodeType> decision_model, bool sequential) {
//    }

    double log_forward_proposal() const {
        return m_log_forward_proposal;
    }

#ifdef DEBUG
    friend std::ostream &operator<<(std::ostream &out, const GraphMatchingState<F, NodeType> &state) {
        out << "matchings:" << "\n  " << print_wrapper(state.m_matchings) << "\n";
        out << "covered nodes:" << "\n  " << print_wrapper(state.m_covered_nodes) << "\n";
        out << "visited nodes:" << "\n  " << print_wrapper(state.m_visited_nodes) << "\n";
        out << "unvisited nodes:" << "\n  " << print_wrapper(state.m_unvisited_nodes) << "\n";
        out << "node to edge view:" << "\n  " << print_wrapper(state.m_node_to_matching, ",\n   ") << "\n";
        out << "decisions:" << "\n  " << print_wrapper(state.m_decisions) << "\n";
        out << "log density: " << state.m_log_density << "\n";
        out << "log forward proposal: " << state.m_log_forward_proposal << "\n";
        return out;
    }
#endif
};
}

#endif //SGMWSMCPP_GRAPHMATCHINGSTATE_H

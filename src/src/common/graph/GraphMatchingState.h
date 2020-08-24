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
#include <stdexcept>
#include <vector>

#include "utils/debug.h"
#include "utils/consts.h"
#include "utils/types.h"
#include "utils/NumericalUtils.h"
#include "utils/Random.h"
#include "utils/container/Counter.h"
#include "utils/container/vector_list.h"
#include "common/model/Command_fwd.h"
#include "common/model/DecisionModel.h"
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
    // TODO: implement a ptr_vector sparse container that preserves insertion order, and fast removal and insertion
    std::vector<node_type> m_visited_nodes;
    vector_list<node_type> m_unvisited_nodes;
    map_t<node_type, edge_type> m_node_to_matching;
    std::vector<edge_type> m_decisions;

    double m_log_density = 0.0;
    double m_log_forward_proposal = 0.0;
    Counter<F> m_log_gradient;

public:
    explicit GraphMatchingState(const std::vector<node_type> &nodes)
        : m_unvisited_nodes(nodes.begin(), nodes.end()) {}

private:
    std::pair<double, double> sample_decision(Random &random,
                                              const MultinomialLogisticModel<F, NodeType> &model,
                                              const node_type &node,
                                              bool use_exact_sampling) {
        // TODO: make sure this works correctly
        auto decisions = generate_decisions(node, *this);
        auto decisions_size = decisions.size();
        auto idx = 0u;
        auto log_prob = 0.0;
        auto log_norm = 0.0;

        if (use_exact_sampling) {
            std::vector<double> log_probs(decisions_size);
            for (auto i = 0u; i < decisions_size; ++i) {
                log_probs[i] = model.log_prob(node, decisions[i]);
            }

            log_norm = NumericalUtils::log_add(log_probs.begin(), log_probs.end());

            // exp normalize
            auto sum = 0.0;
            auto probs = log_probs;
            auto max_prob = Consts::NEGATIVE_INFINITY;
            for (auto prob : probs) {
                max_prob = std::max(max_prob, prob);
            }
            for (auto &prob : probs) {
                prob = std::exp(prob - max_prob);
                sum += prob;
            }

            // normalize
//            if (sum == 0) throw sgm::runtime_error("Normalization should be positive.");
            if (sum != 1) {
                for (auto &prob : probs) {
                    prob /= sum;
                }
            }

            // sample
            auto v = random.next_double();
            sum = 0.0;
            for (idx = 0; idx < probs.size(); ++idx) {
                sum += probs[idx];
                if (v < sum) {
                    break;
                }
            }
            if (idx == probs.size()) {
                throw std::runtime_error("Could not sample.");
            }

            log_prob = log_probs[idx];
            m_log_forward_proposal = log_prob - log_norm;
        } else {
            idx = random.next_int(decisions_size);
            m_log_forward_proposal = -std::log(decisions_size);
            log_prob = model.log_prob(node, decisions[idx]);
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
//    bool has_next_step() {
//    }

//    std::vector<GraphMatchingState<F, NodeType>> execute_move(
//        const sgm::MultinomialLogisticModel<F, NodeType> &model,
//        const sgm::DecisionModel<F, NodeType> &decision_model,
//        const map_t<node_type, set_t<node_type>> &final_state) {
//    }

    void evaluate_decision(const edge_type &decision, const MultinomialLogisticModel<F, NodeType> &model) {
        auto log_norm = Consts::NEGATIVE_INFINITY;
        Counter<F> suff;
        Counter<F> features;

        auto node = std::move(m_unvisited_nodes.front());
        m_unvisited_nodes.erase(0);
        m_visited_nodes.push_back(node);

        auto decision_set = generate_decisions(node, *this);
        auto decision_is_subset = [&decision](const edge_type &superset) -> bool {
            for (auto &d : *decision) {
                if (!superset->count(d)) return false;
            }
            return true;
        };

        auto decision_features = Counter<F>();

        for (auto &d : decision_set) {
            // compute the quantities needed for evaluating the log-likelihood and gradient of log-likelihood
            auto log_prob = model.log_prob(node, d, decision_features);
            log_norm = NumericalUtils::log_add(log_norm, log_prob);
            for (auto &f : decision_features) {
                suff.increment(f.first, std::exp(log_prob) * decision_features.get(f.first));
            }

            auto e = m_node_to_matching.count(node)
                ? m_node_to_matching[node]
                : edge_type(nullptr);
            auto new_edge = std::make_shared<typename edge_type::element_type>(*d);

            new_edge->emplace(node);

            if (e) {
                new_edge->insert(e->begin(), e->end());
            }

            if (decision_is_subset(d)) {
                if (e) {
                    m_matchings.erase(e);
                }

                for (auto &n : *new_edge) {
                    m_covered_nodes.emplace(n);
                    m_node_to_matching[n] = new_edge;
                }

                m_matchings.erase(d);
                m_matchings.emplace(new_edge);
                m_log_density += log_prob;
                features = decision_features;
            }
        }

        m_log_density -= log_norm;
        for (auto &f : suff) {
            m_log_gradient.increment(f.first, features.get(f.first) - f.second / std::exp(log_norm));
        }


        /* This code is here as an attempted correction for when log_norm is too large in magnitude, resulting in
         * us storing a nan inside m_log_gradient. However, applying these corrections breaks the optimization step.
         * Why is the algorithm happier with a counter full of nan's than a counter full of actual numerical values?
         * Who knows.

        // log_norm may be large, causing exp(log_norm) to be too big to store in a double, causing a nan as we may
        // divide it by infinity
        // workaround: here, we clamp log_norm to be within double range
        auto log_norm_exp = std::exp(log_norm);
        if (log_norm_exp == std::numeric_limits<double>::infinity()) {
            sgm::logger << "log_norm exceeded double limit\n";
            log_norm_exp = std::numeric_limits<double>::max();
        }

        for (auto &f : suff) {
            auto val = features.get(f.first) - f.second / log_norm_exp;
            // This is an attempt to fail gracefully on a nan result. Root cause is log_norm's magnitude being too big
            if (std::isnan(val)) {
                sgm::logger << "log_norm is negative and large\n";
                auto fallback = 0.0;
                if (f.second == 0) {
                    // case 1: f.second is 0 --> 0
                    fallback = features.get(f.first);
                    m_log_gradient.increment(f.first, fallback);
                } else {
                    // case 2: f.second is infinity --> -f.second
                    fallback = -1 * f.second;
                    m_log_gradient.set(f.first, fallback);
                }
            } else {
                m_log_gradient.increment(f.first, val);
            }
        }*/

//        return std::make_pair(m_log_density, m_log_gradient);
    }

    double log_density() const {
        return m_log_density;
    }

    const Counter<F> &log_gradient() const {
        return m_log_gradient;
    }

    Counter<F> &log_gradient() {
        return m_log_gradient;
    }

//    const node_type &example_node() {
//    }

    double sample_next_state(Random &random,
                             const Command<F, NodeType> &command,
                             bool use_sequential_sampling, bool use_exact_sampling) {
        auto idx = 0;
        if (!use_sequential_sampling) {
            // TODO: is correct?
            idx = random.next_int(m_unvisited_nodes.size());
        }

        auto node = std::move(m_unvisited_nodes[idx]);
        m_unvisited_nodes.erase(idx);

        auto cur_model = command.current_model();
        auto sample = sample_decision(random, cur_model, node, use_exact_sampling);
        auto log_prob = sample.first;
        m_log_density += log_prob - sample.second;
        m_visited_nodes.emplace_back(std::move(node));
        return log_prob - sample.second;
    }

    int num_parents() const {
        auto num_p = 0;
        auto num_singletons = 0;
        for (auto &e : m_matchings) {
            if (e->size() == 1) {
                ++num_singletons;
            }
        }

        auto singleton_exists = num_singletons > 0;
        num_p += num_singletons;

        for (auto &e : m_matchings) {
            if (e->size() == 1) continue;

            int num_visited = 0;
            for (auto &node : *e) {
                for (auto &vn : m_visited_nodes) {
                    if (vn == node) {
                        ++num_visited;
                        break;
                    }
                }
            }

            if (e->size() == 2) {
                if (singleton_exists) {
                    if (num_visited == 1) {
                        return 1;
                    } else if (num_visited == 2) {
                        num_p += 2;
                    } else {
                        throw sgm::runtime_error("");
                    }
                } else {
                    if (num_visited == 1) {
                        num_p += 1;
                    } else if (num_visited == 2) {
                        num_p += 2;
                    } else {
                        throw sgm::runtime_error("");
                    }
                }
            } else if (e->size() == 3) {
                if (singleton_exists) {
                    if (num_visited == 2) {
                        return num_singletons;
                    } else if (num_visited == 3) {
                        num_p += 3;
                    } else {
                        throw sgm::runtime_error("");
                    }
                } else {
                    if (num_visited == 2) {
                        num_p += 2;
                    } else if (num_visited == 3) {
                        num_p += 6;
                    } else {
                        throw sgm::runtime_error("");
                    }
                }
            }
        }

        return num_p;
    }

    const vector_list<node_type> &unvisited_nodes() const {
        return m_unvisited_nodes;
    }

    const std::vector<node_type> &visited_nodes() const {
        return m_visited_nodes;
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

//    bool covers(NodeType node) const {
//    }

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

    friend std::ostream &operator<<(std::ostream &out, const GraphMatchingState<F, NodeType> &state) {
        std::vector<std::vector<node_type>> sorted_matchings;
        sorted_matchings.reserve(state.m_matchings.size());

        for (const auto &m : state.m_matchings) {
            std::vector<node_type> sorted_matching(m->begin(), m->end());
            std::sort(sorted_matching.begin(), sorted_matching.end(), [](const auto &lhs, const auto &rhs) {
//                return lhs->pidx() < rhs->pidx() || (lhs->pidx() == rhs->pidx() && lhs->idx() < rhs->idx());
                return *lhs < *rhs;
            });
            sorted_matchings.emplace_back(std::move(sorted_matching));
        }

        std::sort(sorted_matchings.begin(), sorted_matchings.end(), [](const auto &lhs, const auto &rhs) {
            auto &min_lhs = *std::min_element(lhs.begin(), lhs.end());
            auto &min_rhs = *std::min_element(rhs.begin(), rhs.end());
//            return min_lhs->pidx() < min_rhs->pidx() ||
//                (min_lhs->pidx() == min_rhs->pidx() && min_lhs->idx() < min_rhs->idx());
            return *min_lhs < *min_rhs;
        });

        out << "matchings:" << "\n  " << print_wrapper(sorted_matchings) << "\n";
        out << "covered nodes:" << "\n  " << print_wrapper(state.m_covered_nodes) << "\n";
        out << "visited nodes:" << "\n  " << print_wrapper(state.m_visited_nodes) << "\n";
        out << "unvisited nodes:" << "\n  " << print_wrapper(state.m_unvisited_nodes) << "\n";
        out << "node to edge view:" << "\n  " << print_wrapper(state.m_node_to_matching, ",\n   ") << "\n";
        out << "decisions:" << "\n  " << print_wrapper(state.m_decisions) << "\n";
        out << "log density: " << state.m_log_density << "\n";
        out << "log forward proposal: " << state.m_log_forward_proposal;
        return out;
    }
};
}

#endif //SGMWSMCPP_GRAPHMATCHINGSTATE_H

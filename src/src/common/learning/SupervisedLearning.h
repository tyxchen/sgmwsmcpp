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

#ifndef SGMWSMCPP_SUPERVISEDLEARNING_H
#define SGMWSMCPP_SUPERVISEDLEARNING_H

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <Eigen/Dense>
#include <parallel_hashmap/phmap.h>
#include <LBFGS.h>

#include "utils/debug.h"
#include "utils/types.h"
#include "utils/opt/LBFGSMinimizer.h"
#include "utils/NumericalUtils.h"
#include "utils/Random.h"
#include "common/model/Command.h"
#include "common/graph/GraphMatchingState.h"
#include "smc/components/GenericMatchingLatentSimulator.h"
#include "smc/components/PruningObservationDensity.h"
#include "smc/components/SequentialGraphMatchingSampler.h"
#include "smc/StreamingParticleFilter.h"

namespace sgm
{

struct SupervisedLearningConfig
{
    static bool parallel;
    static int num_lbfgs_iters;
};

namespace SupervisedLearning
{

namespace detail
{

template <typename F, typename NodeType>
std::pair<double, Counter<F>> evaluate(Command<F, NodeType> &command, Counter<F> &params,
                                       std::pair<std::vector<edge_type_base<NodeType>>,
                                                 std::vector<node_type_base<NodeType>>> &instance) {
    auto &decision_model = command.decision_model();
    MultinomialLogisticModel<F, NodeType> model(command.feature_extractor(), params);

    auto &permutation = instance.second;
    auto &decisions = instance.first;
    GraphMatchingState<F, NodeType> state(permutation);

    for (auto &decision : decisions) {
        state.evaluate_decision(decision, decision_model, model);
    }

    return std::make_pair(state.log_density(), std::move(state.log_gradient()));
}

}

template <typename F, typename NodeType>
class ObjectiveFunction
{
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;

    std::vector<std::pair<std::vector<edge_type>, std::vector<node_type>>> m_latent_decisions;
    std::reference_wrapper<Command<F, NodeType>> m_command;

    double m_log_density = 0.0;
    Counter<F> m_log_gradient;

    Eigen::VectorXd m_curr_x;

public:
    explicit ObjectiveFunction(Command<F, NodeType> &command) : m_command(command) {}

    ObjectiveFunction(Command<F, NodeType> &command, size_t n)
        : m_command(command), m_curr_x(Eigen::VectorXd::Zero(n)) {}

    bool requires_computation(const Eigen::VectorXd &x) {
        return m_curr_x.rows() == 0 || x != m_curr_x;
    }

    double value_at(const Eigen::VectorXd &x) {
        if (!requires_computation(x)) return -1 * m_log_density;

        if (m_curr_x.rows() == 0 || std::isnan(x(0)))
            m_curr_x = x;

        m_log_density = 0.0;
        Counter<F> params;
        for (auto i = 0l, r = x.rows(); i < r; ++i) {
            m_log_gradient.set(m_command.get().indexer().i2o(i), 0.0);
            params.set(m_command.get().indexer().i2o(i), x(i));
        }

        // TODO: implement parallelization
        for (auto &instance : m_latent_decisions) {
            auto ret = detail::evaluate(m_command.get(), params, instance);
            m_log_density += ret.first;
            m_log_gradient.increment_all(ret.second);
        }

        return -1 * m_log_density;
    }

    Eigen::VectorXd derivative_at(const Eigen::VectorXd &x) {
        if (requires_computation(x)) {
            value_at(x);
        }

        auto ret = Eigen::VectorXd(x.rows());
        auto &indexer = m_command.get().indexer();

        for (auto j = 0l, r = x.rows(); j < r; ++j) {
            ret(j) = m_log_gradient.get(indexer.i2o(j));
            ret(j) *= -1;
        }

        return ret;
    }

    double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
        grad = derivative_at(x);
        std::cerr << m_log_density << "\n";
        return -m_log_density;
    }

    void add_instances(const std::vector<GraphMatchingState<F, NodeType>> &samples) {
        for (const auto &sample : samples) {
            m_latent_decisions.emplace_back(sample.decisions(), sample.visited_nodes());
        }
    }

    int dim() {
        return m_command.get().feature_extractor().dim();
    }
};

namespace detail
{

template <typename F, typename NodeType>
std::vector<GraphMatchingState<F, NodeType>> generate_samples(
    Random &random,
    const std::pair<std::vector<edge_type_base<NodeType>>, std::vector<node_type_base<NodeType>>> &instance,
    const std::vector<node_type_base<NodeType>> &emissions,
    GraphMatchingState<F, NodeType> &initial,
    Command<F, NodeType> &command,
    int num_concrete_particles,
    int max_virtual_particles,
    bool use_SPF
) {
    smc::GenericMatchingLatentSimulator<F, NodeType> transition_density(command, initial, false, true);
    smc::PruningObservationDensity<F, NodeType> obs_density(instance.first);
    smc::SequentialGraphMatchingSampler<F, NodeType> smc(transition_density, obs_density, emissions, use_SPF);
    smc.sample(random, num_concrete_particles, max_virtual_particles);
    auto &samples = smc.samples();

    // FIXME: Because of C++'s lack of a universal hash function and how we hash edges by their memory address,
    //  duplicates will appear in each samples' matchings. Until we manage to figure out a hash specialization, we
    //  will have to accept the performance penalty here.
    for (auto &sample : samples) {
        sample.remove_match_duplicates();
    }

    return std::move(samples);
}

template <typename F, typename NodeType>
void gradient_checker(ObjectiveFunction<F, NodeType> &objective, Eigen::VectorXd initial) {
    auto h = 1e-7;
    auto dim = initial.rows();
    auto val1 = objective.value_at(initial);
    auto grad1 = objective.derivative_at(initial);
    auto grad2 = Eigen::VectorXd(dim);
    auto diff = 0.0;

    for (auto k = 0; k < dim; ++k) {
        initial(k) += h;
        auto val2 = objective.value_at(initial);
        initial(k) -= h;
        grad2(k) = (val1 - val2) / h;

        auto val = std::abs(grad1(k) - grad2(k));
        sgm::logger << grad1(k) << "-" << grad2(k) << "=" << val << "\n";
        diff += val;
    }
    diff /= static_cast<double>(dim);
    sgm::logger << "Gradient check: " << diff;
    if (diff > 0.01) {
        throw sgm::runtime_error("Gradient check failed: diff greater than 0.01");
    }
}

}

bool check_convergence(double old_nllk, double new_nllk, double tolerance);

template <typename F, typename NodeType>
/*std::pair<double, Eigen::VectorXd>*/ void MAP_via_MCEM(
    Random &random,
    int rep_id,
    Command<F, NodeType> &command,
    std::vector<std::pair<std::vector<edge_type_base<NodeType>>,
                          std::vector<node_type_base<NodeType>>>> &instances,
    int max_iter,
    int num_concrete_particles,
    int num_implicit_particles,
    const Eigen::VectorXd &initial,
    double tolerance,
    bool check_gradient,
    bool use_spf
) {
    std::vector<std::shared_ptr<std::vector<node_type_base<NodeType>>>> emissions_list;
    std::vector<GraphMatchingState<F, NodeType>> initial_states;

    emissions_list.reserve(instances.size());
    initial_states.reserve(instances.size());

    for (auto &instance : instances) {
        emissions_list.emplace_back(&instance.second, [](auto *) {});
        initial_states.emplace_back(instance.second);
    }

    auto iter = 0;
    auto converged = false;

    LBFGSMinimizer minimizer(SupervisedLearningConfig::num_lbfgs_iters, true);

    auto w = initial;
    auto nllk = 0.0;

    // Track MCEM diagnostics when debugging
#ifndef NDEBUG
    std::vector<Eigen::VectorXd> param_trajectory;
    std::vector<double> vars;
    std::vector<double> means;

    param_trajectory.emplace_back(w);
#endif

    while (!converged && iter < max_iter) {
        ObjectiveFunction<F, NodeType> objective(command);
        std::vector<ObjectiveFunction<F, NodeType>> objs;
        auto instances_size = instances.size();

        // TODO: can this be parallelized?
        for (auto i = 0u; i < instances_size; ++i) {
            auto samples = detail::generate_samples(random, instances[i], *emissions_list[i], initial_states[i],
                                                    command, num_concrete_particles, num_implicit_particles, use_spf);
            ObjectiveFunction<F, NodeType> obj2(command);

            objective.add_instances(samples);
            obj2.add_instances(samples);
            objs.emplace_back(std::move(obj2));
            /* DEBUG */
            break;
        }

        if (check_gradient && iter == 0) {
            detail::gradient_checker(objective, initial);
        }

        Eigen::VectorXd random_w(w.rows());
        for (auto i = 0l, dim = w.rows(); i < dim; ++i) {
            random_w(i) = random.next_double();
        }

        sgm::logger << "done random vector" << std::endl;

        auto w_new = minimizer.minimize(objective, w, tolerance);
        auto nllk_new = objective.value_at(w_new);

        sgm::logger << "done minimizing" << std::endl;

        sgm::logger << "curr nllk: " << nllk_new << "\n";
        sgm::logger << "new w:\n" << w_new << "\n";

        converged = check_convergence(nllk, nllk_new, tolerance);
        w = w_new;
        nllk = nllk_new;

        iter++;
        break;
    }
}
}
}

#endif //SGMWSMCPP_SUPERVISEDLEARNING_H

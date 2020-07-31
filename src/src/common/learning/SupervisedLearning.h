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

#include <memory>
#include <utility>
#include <Eigen/Dense>
#include <parallel_hashmap/phmap.h>
#include <LBFGS.h>

#include "utils/types.h"
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
template <typename F, typename NodeType>
class ObjectiveFunction
{
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;

    std::vector<std::pair<std::vector<edge_type>, edge_type>> m_latent_decisions;
    Command<F, NodeType> &m_command;

    double m_log_density = 0.0;
    Counter<F> m_log_gradient;

    Eigen::VectorXd m_curr_x;
    double m_lambda = 0.0;

public:
    ObjectiveFunction(Command<F, NodeType> &command, size_t n)
        : m_command(command), m_curr_x(Eigen::VectorXd::Zero(n)) {}
};

namespace detail
{
template <typename F, typename NodeType, typename Random>
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
    return std::move(smc.samples());
}
}

bool check_convergence(double old_nllk, double new_nllk, double tolerance);

template <typename F, typename NodeType, typename Random>
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

    LBFGSpp::LBFGSParam<double> param;
    param.epsilon = tolerance;
    param.max_iterations = SupervisedLearningConfig::num_lbfgs_iters;

    LBFGSpp::LBFGSSolver<double> minimizer(param);

    // Track MCEM diagnostics when debugging
#ifdef DEBUG
    std::vector<Eigen::VectorXd> param_trajectory;
    std::vector<double> vars;
    std::vector<double> means;

    param_trajectory.emplace_back(initial);
#endif

    while (!converged && iter < max_iter) {
        ObjectiveFunction<F, NodeType> objective(command, initial.size());
        std::vector<ObjectiveFunction<F, NodeType>> objs;
        auto instances_size = instances.size();

        for (auto i = 0; i < instances_size; ++i) {
            auto samples = detail::generate_samples(random, instances[i], *emissions_list[i], initial_states[i],
                                                    command, num_concrete_particles, num_implicit_particles, use_spf);
            converged = true;
            /* DEBUG */
            return;
        }
    }
}
}
}

#endif //SGMWSMCPP_SUPERVISEDLEARNING_H

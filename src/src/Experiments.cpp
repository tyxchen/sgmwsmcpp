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

#include "Experiments.h"

#include <tbb/tbb.h>

#include "utils/debug.h"
#include "utils/consts.h"
#include "utils/types.h"
#include "knot/data/EllipticalKnot.h"
#include "common/learning/SupervisedLearning.h"
#include "utils/ExpUtils.h"
#include "utils/Random.h"
#include "knot/model/features/EllipticalKnotFeatureExtractor.h"
#include "common/model/Command.h"
#include "common/graph/GraphMatchingState.h"
#include "smc/components/ExactProposalObservationDensity.h"
#include "smc/components/GenericMatchingLatentSimulator.h"

using namespace sgm;

using node_type = node_type_base<EllipticalKnot>;
using edge_type = edge_type_base<EllipticalKnot>;
using datum_type = typename std::vector<std::pair<std::vector<edge_type>, std::vector<node_type>>>;

TrainAndPredictResult::TrainAndPredictResult(int _pidx, int _idx, int _matching)
    : pidx(_pidx), idx(_idx), matching(_matching) {}

std::tuple<Counter<string_t>, Counter<string_t>, Counter<string_t>> sgm::train(
    const std::vector<std::string> &training_boards,
    int concrete_particles, int max_implicit_particles,
    int max_em_iter, int max_lbfgs_iter,
    sgm::Random::seed_type seed, double tol, bool use_spf, bool parallelize
) {
    auto random = Random(seed);

    SupervisedLearningConfig::parallel = parallelize;
    SupervisedLearningConfig::num_lbfgs_iters = max_lbfgs_iter;

    auto training_instances = ExpUtils::read_test_boards(training_boards, false);
    auto training_data = ExpUtils::unpack<EllipticalKnot>(training_instances);

    EllipticalKnotFeatureExtractor fe;
    auto fe_dim = fe.dim();
    Command<string_t, EllipticalKnot> command(fe);

    // compute the features on each of these data points and standardization
    std::vector<std::vector<double>> standardizations(fe_dim);
    auto phi = fe.default_parameters();

    for (const auto &datum : training_data) {
        for (const auto &e : datum.first) {
            phi = fe.default_parameters();
            fe.extract_features(e, phi);
            for (const auto &vals : phi) {
                standardizations[command.indexer().o2i(vals.first)].push_back(vals.second);
            }
        }
    }

    Counter<string_t> mean;
    Counter<string_t> sd;

    for (auto i = 0u; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        auto &standardization = standardizations[i];
        auto n = static_cast<double>(standardization.size());

        // calculate mean
        auto m = 0.0;
        for (auto s : standardization) {
            m += s;
        }
        m /= n;

        // calculate variance
        auto var = 0.0;
        for (auto s : standardization) {
            var += (s - m) * (s - m);
        }

        mean.set(f, m);
        sd.set(f, std::sqrt(var / (n - 1)));
    }

    for (auto i = 0u; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        sgm::logger << "Feature " << i << ":" << std::endl;
        sgm::logger << "  name: " << f << std::endl;
        sgm::logger << "  mean: " << mean.get(f) << std::endl;
        sgm::logger << "  sd:   " << sd.get(f) << std::endl;
    }

    fe.standardize(mean, sd);

    /*
     * Train the model parameters and run SMC on test data
     */
    auto initial = Eigen::VectorXd::Zero(fe_dim);
    auto ret = SupervisedLearning::MAP_via_MCEM(random, 0, command, training_data, max_em_iter, concrete_particles,
                                                max_implicit_particles, initial, tol, false, use_spf);

    sgm::logger << ret.first << "\n" << ret.second << std::endl;

    command.update_model_parameters(ret.second);

    return std::make_tuple(std::move(command.model_parameters()),
                           std::move(mean), std::move(sd));
}

std::vector<std::vector<TrainAndPredictResult>> sgm::predict(
    const std::vector<std::vector<node_type_base<EllipticalKnot>>> &test_instances,
    const Counter<string_t> &params,
    const Counter<string_t> &mean,
    const Counter<string_t> &sd,
    int target_ess,
    Random::seed_type seed,
    bool use_spf,
    bool parallelize
) {
    auto matchings_by_dataset = std::vector<std::vector<TrainAndPredictResult>>();
    EllipticalKnotFeatureExtractor fe;
    fe.standardize(mean, sd);
    Command<string_t, EllipticalKnot> command(fe);
    command.model_parameters() = params;

    auto random = Random(seed);

    SupervisedLearningConfig::parallel = parallelize;

    // The maximum number of GraphMatchingStates that will be active at any given step of sample generation is equal
    // to 1000 (hardcoded), and the number of GraphMatchingStates saved from a single round of sample
    // generation is equal to target_ess. Thus, we calculate the memory pool for a GraphMatchingState as
    // having size
    //     (instances_size - 1) * target_ess + 1000
    // We reallocate here after initial allocation in case we need an increase in the size of the memory pool.
    GraphMatchingState<string_t, EllipticalKnot>::allocate(
        (test_instances.size() - 1) * target_ess + 1000
    );

    for (auto &emissions : test_instances) {
        if (emissions.empty()) {
            sgm::logger << "Board has no knots, skipping.\n";
            matchings_by_dataset.emplace_back();
            continue;
        }

        auto initial_state = std::make_shared<GraphMatchingState<string_t, EllipticalKnot>>(emissions);
        auto transition_density = smc::GenericMatchingLatentSimulator<string_t, EllipticalKnot>(command,
                                                                                                initial_state,
                                                                                                false, true);
        auto observation_density = smc::ExactProposalObservationDensity<string_t, EllipticalKnot>();
        auto samples = std::vector<std::shared_ptr<GraphMatchingState<string_t, EllipticalKnot>>>();

        auto timer = performance_timer(true);
        smc::sample(transition_density, observation_density, emissions, random(), target_ess, 1000, use_spf, samples);
        timer.end();

        sgm::logger << "Run time: " << timer.diff() / 1000 << "ms\n";

        auto best_log_density = Consts::NEGATIVE_INFINITY;
        auto best_sample_idx = -1;

        for (size_t size = samples.size(), j = 0; j < size; ++j) {
            if (samples[j]->log_density() > best_log_density) {
                best_log_density = samples[j]->log_density();
                best_sample_idx = j;
            }
        }

        auto &best_sample = samples[best_sample_idx];
        auto match_idx = 0;
        auto matchings = std::vector<TrainAndPredictResult>();

        for (auto &matching : best_sample->matchings()) {
            for (auto &knot : *matching) {
                matchings.emplace_back(knot->pidx(), knot->idx(), match_idx);
            }
            ++match_idx;
        }

        matchings_by_dataset.emplace_back(std::move(matchings));
    }

    return matchings_by_dataset;
}

std::vector<std::vector<TrainAndPredictResult>> sgm::predict(
    const std::vector<std::string> &test_boards,
    const Counter<string_t> &params,
    const Counter<string_t> &mean,
    const Counter<string_t> &sd,
    int target_ess,
    Random::seed_type seed,
    bool use_spf,
    bool parallelize
) {
    auto test_instances = ExpUtils::read_test_boards(test_boards, false);
    auto instances = std::vector<std::vector<node_type_base<EllipticalKnot>>>();
    instances.reserve(test_instances.size());
    for (auto &test_inst : test_instances) {
        instances.emplace_back(std::move(test_inst.knots()));
    }
    return predict(instances, params, mean, sd, target_ess, seed, use_spf, parallelize);
}

std::vector<std::vector<TrainAndPredictResult>>
sgm::train_and_predict(
    const std::vector<std::string> &training_boards,
    const std::vector<std::string> &test_boards,
    int concrete_particles,
    int max_implicit_particles,
    int target_ess,
    int max_em_iter,
    int max_lbfgs_iter,
    Random::seed_type seed,
    double tol,
    bool use_spf,
    bool parallelize
) {
    auto ret = train(training_boards, concrete_particles, max_implicit_particles, max_em_iter, max_lbfgs_iter, seed,
                     tol, use_spf, parallelize);
    return predict(test_boards, std::get<0>(ret), std::get<1>(ret), std::get<2>(ret), target_ess, seed, use_spf,
                   parallelize);
}

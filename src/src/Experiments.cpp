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

#include <boost/filesystem.hpp>

#include "utils/debug.h"
#include "utils/types.h"
#include "knot/data/EllipticalKnot.h"
#include "common/learning/SupervisedLearning.h"
#include "utils/ExpUtils.h"
#include "utils/Random.h"
#include "common/model/PairwiseMatchingModel.h"
#include "knot/model/features/EllipticalKnotFeatureExtractor.h"
#include "common/model/Command.h"
#include "common/graph/GraphMatchingState.h"
#include "smc/components/ExactProposalObservationDensity.h"
#include "smc/components/GenericMatchingLatentSimulator.h"

using namespace sgm;

namespace fs = boost::filesystem;
using node_type = node_type_base<EllipticalKnot>;
using edge_type = edge_type_base<EllipticalKnot>;
using datum_type = typename std::vector<std::pair<std::vector<edge_type>, std::vector<node_type>>>;

void sgm::train_and_predict(const std::vector<std::string> &training_directories,
                            const std::vector<std::string> &test_directories,
                            const std::string &output_directory, const std::string &log_directory,
                            int concrete_particles, int max_implicit_particles, int target_ess,
                            int max_em_iter, int max_lbfgs_iter,
                            sgm::Random::seed_type seed, double tol, bool use_spf, bool parallelize) {
    auto random = Random(seed);
    std::vector<fs::path> BOARDS;
    std::vector<fs::path> TEST_BOARDS;

    for (const auto &dir : training_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            throw sgm::runtime_error("Error: Data directory `" + dir + "` is not a directory.");
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            BOARDS.emplace_back(path / board);
        }
        // Sort for compatibility with reference impl
        std::sort(BOARDS.begin(), BOARDS.end());
    }

    for (const auto &dir : test_directories) {
        auto path = fs::path(dir);
        if (!fs::is_directory(path)) {
            throw sgm::runtime_error("Error: Test data directory `" + dir + "` is not a directory.");
        }
        auto dir_iter = fs::directory_iterator(path);
        for (const auto &dir_item : dir_iter) {
            auto board = dir_item.path().filename();
            if (board.string()[0] == '.') continue;
            TEST_BOARDS.emplace_back(path / board);
        }
        // Sort for compatibility with reference impl
        std::sort(TEST_BOARDS.begin(), TEST_BOARDS.end());
    }

    SupervisedLearningConfig::parallel = parallelize;
    SupervisedLearningConfig::num_lbfgs_iters = max_lbfgs_iter;

    sgm::logger << "Current working dir: " << fs::current_path() << std::endl;

    auto training_instances = ExpUtils::read_test_boards(BOARDS, false);
    auto training_data = ExpUtils::unpack<EllipticalKnot>(training_instances);

    auto test_instances = ExpUtils::read_test_boards(TEST_BOARDS, false);
    sgm::logger << "Test instances size: " << test_instances.size() << std::endl;

    PairwiseMatchingModel<std::string, EllipticalKnot> decision_model;
    EllipticalKnotFeatureExtractor fe;
    auto fe_dim = fe.dim();
    Command<std::string, EllipticalKnot> command(decision_model, fe);

    // compute the features on each of these data points and standardization
    std::vector<std::vector<double>> standardizations(fe_dim);

    for (const auto &data : training_data) {
        for (const auto &datum : data) {
            for (const auto &e : datum.first) {
                auto phi = fe.extract_features(e);
                for (const auto &vals : phi) {
                    standardizations[command.indexer().o2i(vals.first)].push_back(phi.get(vals.first));
                }
            }
        }
    }

    Counter<std::string> mean;
    Counter<std::string> sd;

    for (auto i = 0; i < fe_dim; ++i) {
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

    fe.standardize(mean, sd);

    /*
     * Train the model parameters and run SMC on test data
     */
    const auto &fe_mean = fe.mean();
    const auto &fe_sd = fe.sd();
    for (auto i = 0; i < fe_dim; ++i) {
        auto f = command.indexer().i2o(i);
        sgm::logger << "Feature " << i << ":" << std::endl;
        sgm::logger << "  name: " << f << std::endl;
        sgm::logger << "  mean: " << fe_mean.get(f) << std::endl;
        sgm::logger << "  sd:   " << fe_sd.get(f) << std::endl;
    }

    auto initial = Eigen::VectorXd::Zero(fe_dim);
    auto instances = ExpUtils::pack<EllipticalKnot>(training_data);
    auto ret = SupervisedLearning::MAP_via_MCEM(random, 0, command, instances, max_em_iter, concrete_particles,
                                                max_implicit_particles, initial, tol, log_directory, false, use_spf);

    sgm::logger << ret.first << "\n" << ret.second << std::endl;

    command.update_model_parameters(ret.second);

    fs::create_directories(output_directory);

    for (auto j = 0ul, size = test_instances.size(); j < size; ++j) {
        auto segment = test_instances[j][0];
        auto initial_state = GraphMatchingState<std::string, EllipticalKnot>(segment.knots());
        auto transition_density = smc::GenericMatchingLatentSimulator<std::string, EllipticalKnot>(command,
                                                                                                   initial_state,
                                                                                                   false, true);
        auto observation_density = smc::ExactProposalObservationDensity<std::string, EllipticalKnot>(command);
        auto emissions = std::vector<node_type>(segment.knots());

        auto smc = smc::SequentialGraphMatchingSampler<std::string, EllipticalKnot>(transition_density,
                                                                                    observation_density,
                                                                                    emissions, use_spf);

        Timers::start("testing data");
        smc.sample(random, target_ess, 1000);
        Timers::end("testing data");

        sgm::logger << "Run time: " << Timers::diff("testing data") << "ms\n";

        auto &samples = smc.samples();
        auto best_log_density = -std::numeric_limits<double>::infinity();
        auto best_sample = samples[0];

        for (auto &sample : samples) {
            if (sample.log_density() > best_log_density) {
                best_log_density = sample.log_density();
                best_sample = sample;
            }
        }

        auto filename = TEST_BOARDS[j].filename();
        auto output_filename = output_directory / filename;
        auto log_output_filename = log_directory / filename;
        auto output_csv = std::ofstream(output_filename.string());
        auto log_output_csv = std::ofstream(log_output_filename.string());
        auto idx = 0;

        for (auto &matching : best_sample.matchings()) {
            for (auto &knot : *matching) {
                output_csv << knot->pidx() << "," << knot->idx() << "," << idx << std::endl;
                log_output_csv << knot->pidx() << "," << knot->idx() << "," << idx << std::endl;
            }
            ++idx;
        }
        output_csv.close();
        log_output_csv.close();
    }
}

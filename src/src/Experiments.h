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

#ifndef SGMWSMCPP_EXPERIMENTS_H
#define SGMWSMCPP_EXPERIMENTS_H

#include <vector>
#include <string>

#include "utils/Random.h"
#include "knot/data/KnotDataReader.h"

namespace sgm
{

/**
 * Represents a single knot in a prediction.
 */
struct TrainAndPredictResult
{
    int pidx;
    int idx;
    int matching;
    TrainAndPredictResult(int _pidx, int _idx, int _matching);
};

/**
 * Train the decision model on the provided data.
 *
 * @param training_files List of CSV filenames containing the training data.
 * @param concrete_particles The minimum number of particles per training sample.
 * @param max_implicit_particles The maximum number of particles per training sample.
 * @param max_em_iter The maximum number of iterations for MCEM.
 * @param max_lbfgs_iter The maximum number of iterations for LBFGS.
 * @param seed The seed to use for the random number generator.
 * @param tol The tolerance to use for the LBFGS minimizer.
 * @param use_spf Should we use a streaming particle filter process?
 * @param parallelize Should the experiment be run with parallelism?
 * @return A tuple of the optimized values of each parameter, the sample means, and the sample standard deviations in
 *         that order.
 */
std::tuple<Counter<string_t>, Counter<string_t>, Counter<string_t>> train(
    const std::vector<std::string> &training_boards,
    int concrete_particles,
    int max_implicit_particles,
    int max_em_iter,
    int max_lbfgs_iter,
    Random::seed_type seed,
    double tol,
    bool use_spf,
    bool parallelize
);

/**
 * Predict matchings on test boards.
 *
 * @param test_instance List of list of knots, with each inner list corresponding to a single board.
 * @param concrete_particles The minimum number of particles per training sample.
 * @param max_implicit_particles The maximum number of particles per training sample.
 * @param target_ess The minimum number of particles per test sample.
 * @param max_em_iter The maximum number of iterations for MCEM.
 * @param max_lbfgs_iter The maximum number of iterations for LBFGS.
 * @param seed The seed to use for the random number generator.
 * @param tol The tolerance to use for the LBFGS minimizer.
 * @param use_spf Should we use a streaming particle filter process?
 * @param parallelize Should the experiment be run with parallelism?
 * @return For each board, returns the complete set of matchings associated with that board.
 */
std::vector<std::vector<TrainAndPredictResult>> predict(
    const std::vector<std::vector<node_type_base<EllipticalKnot>>> &test_instances,
    const Counter<string_t> &params,
    const Counter<string_t> &mean,
    const Counter<string_t> &sd,
    int target_ess,
    Random::seed_type seed,
    bool use_spf,
    bool parallelize
);

/**
 * Predict matchings on test data.
 *
 * @param test_boards List of CSV filenames containing the test data.
 * @param concrete_particles The minimum number of particles per training sample.
 * @param max_implicit_particles The maximum number of particles per training sample.
 * @param target_ess The minimum number of particles per test sample.
 * @param max_em_iter The maximum number of iterations for MCEM.
 * @param max_lbfgs_iter The maximum number of iterations for LBFGS.
 * @param seed The seed to use for the random number generator.
 * @param tol The tolerance to use for the LBFGS minimizer.
 * @param use_spf Should we use a streaming particle filter process?
 * @param parallelize Should the experiment be run with parallelism?
 * @return For each file, returns the complete set of matchings associated with that file.
 */
std::vector<std::vector<TrainAndPredictResult>> predict(
    const std::vector<std::string> &test_boards,
    const Counter<string_t> &params,
    const Counter<string_t> &mean,
    const Counter<string_t> &sd,
    int target_ess,
    Random::seed_type seed,
    bool use_spf,
    bool parallelize
);

/**
 * Run the train and predict experiment.
 *
 * @param training_files List of CSV filenames containing the training data.
 * @param test_files List of CSV filenames containing the test data.
 * @param concrete_particles The minimum number of particles per training sample.
 * @param max_implicit_particles The maximum number of particles per training sample.
 * @param target_ess The minimum number of particles per test sample.
 * @param max_em_iter The maximum number of iterations for MCEM.
 * @param max_lbfgs_iter The maximum number of iterations for LBFGS.
 * @param seed The seed to use for the random number generator.
 * @param tol The tolerance to use for the LBFGS minimizer.
 * @param use_spf Should we use a streaming particle filter process?
 * @param parallelize Should the experiment be run with parallelism?
 * @return For each test file, returns the complete set of matchings associated with that file.
 */
std::vector<std::vector<TrainAndPredictResult>> train_and_predict(
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
);

}

#endif //SGMWSMCPP_EXPERIMENTS_H

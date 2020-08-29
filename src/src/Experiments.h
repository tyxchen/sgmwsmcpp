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

struct TrainAndPredictResult
{
    int pidx;
    int idx;
    int matching;
    TrainAndPredictResult(int _pidx, int _idx, int _matching);
};

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

std::vector<std::vector<TrainAndPredictResult>> predict(
    const std::vector<KnotDataReader::Segment> &test_instances,
    const Counter<string_t> &params,
    const Counter<string_t> &mean,
    const Counter<string_t> &sd,
    int target_ess,
    Random::seed_type seed,
    bool use_spf,
    bool parallelize
);

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

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

#ifndef SGMWSMCPP_COMMAND_H
#define SGMWSMCPP_COMMAND_H

#include <functional>
#include <memory>
#include <vector>
#include <utility>
#include <Eigen/Core>

#include "utils/Random.h"
#include "utils/container/Counter.h"
#include "utils/container/Indexer.h"
#include "common/graph/GraphMatchingState_fwd.h"
#include "common/model/GraphFeatureExtractor.h"
#include "common/model/MultinomialLogisticModel.h"

namespace sgm
{
template <typename F, typename NodeType>
class Command
{
    std::reference_wrapper<GraphFeatureExtractor<F, NodeType>> m_fe;
    Counter<F> m_params;
    Indexer<F> m_indexer;

public:
    Command(GraphFeatureExtractor<F, NodeType> &fe)
        : m_fe(fe), m_params(fe.default_parameters()), m_indexer(m_params.keys()) {}

    const Indexer<F> &indexer() const {
        return m_indexer;
    }

    Indexer<F> &indexer() {
        return m_indexer;
    }

    void sample_next(Random &random,
        GraphMatchingState<F, NodeType> &state,
        bool use_sequential_sampling,
        bool use_exact_sampling) {
        state.sample_next_state(random, *this, use_sequential_sampling, use_exact_sampling);
    }

    MultinomialLogisticModel<F, NodeType> current_model() const {
        return MultinomialLogisticModel<F, NodeType>(m_fe.get(), m_params);
    }

    const GraphFeatureExtractor<F, NodeType> &feature_extractor() const {
        return m_fe;
    }

    GraphFeatureExtractor<F, NodeType> &feature_extractor() {
        return m_fe;
    }

    const Counter<F> &model_parameters() const {
        return m_params;
    }

    Counter<F> &model_parameters() {
        return m_params;
    }

    void update_model_parameters(const Eigen::VectorXd &w) {
        for (auto i = 0l, r = w.rows(); i < r; ++i) {
            m_params.set(m_indexer.i2o(i), w(i));
        }
    }
};
}

#endif //SGMWSMCPP_COMMAND_H

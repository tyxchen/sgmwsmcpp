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

#include "utils/Random.h"
#include "utils/container/Counter.h"
#include "utils/container/Indexer.h"
#include "common/graph/GraphMatchingState_fwd.h"
#include "common/model/DecisionModel.h"
#include "common/model/GraphFeatureExtractor.h"
#include "common/model/MultinomialLogisticModel.h"

namespace sgm
{
template <typename F, typename NodeType>
class Command
{
    std::reference_wrapper<DecisionModel<F, NodeType>> m_decision_model;
    std::reference_wrapper<GraphFeatureExtractor<F, NodeType>> m_fe;
    Counter<F> m_params;
    Indexer<F> m_indexer;

public:
    Command(DecisionModel<F, NodeType> &decision_model, GraphFeatureExtractor<F, NodeType> &fe)
        : m_decision_model(decision_model), m_fe(fe),
          m_params(fe.default_parameters()), m_indexer(m_params.keys()) {}

    Indexer<F> &indexer() {
        return m_indexer;
    }

    void sample_next(Random &random,
        GraphMatchingState<F, NodeType> &state,
        bool use_sequential_sampling,
        bool use_exact_sampling) {
        state.sample_next_state(random, *this, use_sequential_sampling, use_exact_sampling);
    }

    const DecisionModel<F, NodeType> &decision_model() const {
        return m_decision_model;
    }

    DecisionModel<F, NodeType> &decision_model() {
        return m_decision_model;
    }

    MultinomialLogisticModel<F, NodeType> current_model() {
        return MultinomialLogisticModel<F, NodeType>(m_fe.get(), m_params);
    }

    GraphFeatureExtractor<F, NodeType> &feature_extractor() {
        return m_fe;
    }
};
}

#endif //SGMWSMCPP_COMMAND_H

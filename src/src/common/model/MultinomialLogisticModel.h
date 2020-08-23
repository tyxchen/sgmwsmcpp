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

#ifndef SGMWSMCPP_MULTINOMIALLOGISTICMODEL_H
#define SGMWSMCPP_MULTINOMIALLOGISTICMODEL_H

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <parallel_hashmap/phmap.h>

#include "utils/debug.h"
#include "utils/types.h"
#include "utils/container/Counter.h"
#include "common/graph/GraphMatchingState_fwd.h"
#include "common/graph/GraphNode.h"
#include "common/model/GraphFeatureExtractor.h"

namespace sgm
{
template <typename F, typename NodeType>
class MultinomialLogisticModel
{
public:
    using node_type = node_type_base<NodeType>;
    using edge_type = edge_type_base<NodeType>;

private:
    std::reference_wrapper<const GraphFeatureExtractor<F, NodeType>> m_fe;
    std::reference_wrapper<const Counter<F>> m_params;

public:
    MultinomialLogisticModel(const GraphFeatureExtractor<F, NodeType> &fe, const Counter<F> &params)
        : m_fe{fe}, m_params{params} {
        if (m_fe.get().dim() != m_params.get().size()) {
            throw std::runtime_error("Feature and parameter dimensions do not match.");
        }
    }

    std::pair<double, Counter<F>> log_prob(const node_type &node, const edge_type &decision) const {
        auto features = m_fe.get().extract_features(node, decision);
        auto prob = 0.0;
        for (const auto &f : features) {
            prob += f.second * m_params.get().get(f.first);
#ifndef NDEBUG
//            std::cerr << "MLM::log_prob " << sgm::count << "\n";
            if (std::isnan(prob)) throw std::runtime_error("");
#endif
        }
        return std::make_pair(prob, std::move(features));
    }

    int num_variables() const {
        return m_fe.get().dim();
    }
};
}

#endif //SGMWSMCPP_MULTINOMIALLOGISTICMODEL_H

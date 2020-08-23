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

#ifndef SGMWSMCPP_GENERICMATCHINGLATENTSIMULATOR_H
#define SGMWSMCPP_GENERICMATCHINGLATENTSIMULATOR_H

#include <functional>
#include <memory>

#include "utils/Random.h"
#include "common/model/Command_fwd.h"
#include "common/graph/GraphMatchingState_fwd.h"

namespace sgm
{
namespace smc
{

template <typename F, typename NodeType>
class GenericMatchingLatentSimulator
{
    std::reference_wrapper<const Command<F, NodeType>> m_command;
    std::shared_ptr<GraphMatchingState<F, NodeType>> m_initial;
    bool m_use_sequential_sampling = true;
    bool m_use_exact_sampling = true;

public:
    GenericMatchingLatentSimulator(const Command<F, NodeType> &command,
                                   const std::shared_ptr<GraphMatchingState<F, NodeType>> &initial,
                                   bool use_sequential_sampling,
                                   bool use_exact_sampling)
        : m_command(command), m_initial(initial),
          m_use_sequential_sampling(use_sequential_sampling), m_use_exact_sampling(use_exact_sampling) {}

    std::shared_ptr<GraphMatchingState<F, NodeType>> sample_initial(Random &random) const {
        return sample_forward_transition(random, m_initial);
    }

    std::shared_ptr<GraphMatchingState<F, NodeType>>
    sample_forward_transition(Random &random, const std::shared_ptr<GraphMatchingState<F, NodeType>> &state) const {
        auto next = std::make_shared<GraphMatchingState<F, NodeType>>(*state);
        next->sample_next_state(random, m_command.get(), m_use_sequential_sampling, m_use_exact_sampling);
        return next;
    }

    size_t iterations() const {
        return m_initial->unvisited_nodes().size();
    }
};

}
}

#endif //SGMWSMCPP_GENERICMATCHINGLATENTSIMULATOR_H

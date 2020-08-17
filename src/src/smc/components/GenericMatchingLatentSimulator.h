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
    std::reference_wrapper<Command<F, NodeType>> m_command;
    std::reference_wrapper<GraphMatchingState<F, NodeType>> m_initial;
    bool m_use_sequential_sampling = true;
    bool m_use_exact_sampling = true;

public:
    GenericMatchingLatentSimulator(Command<F, NodeType> &command,
                                   GraphMatchingState<F, NodeType> &initial,
                                   bool use_sequential_sampling,
                                   bool use_exact_sampling)
        : m_command(command), m_initial(initial), m_use_sequential_sampling(use_sequential_sampling),
          m_use_exact_sampling(use_exact_sampling) {}

    GraphMatchingState<F, NodeType> sample_initial(Random &random) {
        return sample_forward_transition(random, m_initial);
    }

    GraphMatchingState<F, NodeType> sample_forward_transition(Random &random,
                                                              const GraphMatchingState<F, NodeType> &state) {
        auto next = state;
        m_command.get().sample_next(random, next, m_use_sequential_sampling, m_use_exact_sampling);
        return next;
    }

    size_t iterations() const {
        return m_initial.get().unvisited_nodes().size();
    }
};

}
}

#endif //SGMWSMCPP_GENERICMATCHINGLATENTSIMULATOR_H

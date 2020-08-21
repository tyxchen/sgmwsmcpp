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

#ifndef SGMWSMCPP_EXACTPROPOSALOBSERVATIONDENSITY_H
#define SGMWSMCPP_EXACTPROPOSALOBSERVATIONDENSITY_H

#include <cmath>
#include <functional>

#include "utils/types.h"
#include "common/model/Command_fwd.h"
#include "common/graph/GraphMatchingState_fwd.h"
#include "smc/ObservationDensity.h"

namespace sgm
{

namespace smc
{

template <typename F, typename NodeType>
class ExactProposalObservationDensity : public ObservationDensity<F, NodeType>
{
public:
    using node_type = typename ObservationDensity<F, NodeType>::node_type;
    using edge_type = typename ObservationDensity<F, NodeType>::edge_type;

    explicit ExactProposalObservationDensity(Command<F, NodeType> &command) : m_command(command) {}

private:
    std::reference_wrapper<Command<F, NodeType>> m_command;

    double _log_density(const GraphMatchingState<F, NodeType> &latent, const node_type &emission) override {
        return latent.log_density();
    }

    double _log_weight_correction(const GraphMatchingState<F, NodeType> &cur_latent,
                                  const GraphMatchingState<F, NodeType> &old_latent) override {
        auto num_parents = cur_latent.num_parents();
        return -1 * std::log(num_parents) - cur_latent.log_forward_proposal();
    }
};

}

}

#endif //SGMWSMCPP_EXACTPROPOSALOBSERVATIONDENSITY_H
